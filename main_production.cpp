/*******************************************************************************
 * Teensy 4.1 CNC/Microscope Motion Controller
 * 
 * Version: 1.0
 * Date: December 2025
 * 
 * Description:
 *   4-axis stepper motor controller for robotic microscope on 3018 CNC frame.
 *   Supports sequential homing, coordinated multi-axis moves, jogging, and
 *   limit switch protection. Designed for long-running autonomous operation.
 * 
 * Hardware:
 *   - Teensy 4.1 (i.MX RT1062, 600MHz)
 *   - 4x TB6600 stepper drivers (8 microsteps, inverted ENA logic)
 *   - 4x NEMA17 steppers (200 steps/rev)
 *   - 8x NO limit switches (min/max per axis)
 *   - T8 lead screws (8mm/rev) on X, Y, Z axes
 * 
 * Commands:
 *   HOME              - Sequential homing (Z→Y→X→Focus)
 *   MOVE X=? Y=? Z=? F=? - Coordinated absolute move (mm or focus units)
 *   JOG X +5.5        - Incremental move on single axis
 *   STATUS            - Report positions and state
 *   STOP              - Emergency stop all motion
 *   PING              - Connectivity test
 * 
 * Notes:
 *   - All speeds set to 25% for initial testing
 *   - Sequential homing prevents mechanical conflicts
 *   - All motion obeys limit switches and soft limits
 *   - Non-blocking architecture for smooth operation
 ******************************************************************************/

#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

//==============================================================================
// VERSION & BUILD INFO
//==============================================================================
const char* FIRMWARE_VERSION = "v1.0 - 4-Axis Motion Controller";
const char* BUILD_DATE = __DATE__ " " __TIME__;

//==============================================================================
// PIN DEFINITIONS
//==============================================================================
const int LED_PIN = 13;  // Onboard LED

// Common enable (active LOW for TB6600)
const int ENA_ALL_PIN = 33;

// X Axis
const int X_STEP_PIN = 2;
const int X_DIR_PIN = 3;
const int X_MIN_PIN = 4;
const int X_MAX_PIN = 5;

// Y Axis
const int Y_STEP_PIN = 6;
const int Y_DIR_PIN = 7;
const int Y_MIN_PIN = 8;
const int Y_MAX_PIN = 9;

// Z Axis
const int Z_STEP_PIN = 10;
const int Z_DIR_PIN = 11;
const int Z_MIN_PIN = 12;
const int Z_MAX_PIN = 14;  // Pin 13 skipped (LED)

// Focus Axis
const int F_STEP_PIN = 15;
const int F_DIR_PIN = 16;
const int F_MIN_PIN = 17;
const int F_MAX_PIN = 18;

//==============================================================================
// MOTION CONFIGURATION
//==============================================================================
// Speed settings (25% of full speed for testing)
const float MAX_SPEED = 1500.0;        // steps/sec (~56 RPM)
const float ACCEL = 500.0;             // steps/sec²
const float HOMING_SPEED = 800.0;      // steps/sec for initial seek
const float HOMING_CREEP_SPEED = 200.0; // steps/sec for precision approach

// Mechanical parameters (3018 CNC + T8 lead screws)
const float STEPS_PER_MM = 200.0;      // 1600 steps/rev ÷ 8mm/rev = 200 steps/mm
const float BACKOFF_DISTANCE_MM = 5.0; // Back off 5mm after hitting limit
const long BACKOFF_STEPS = (long)(BACKOFF_DISTANCE_MM * STEPS_PER_MM);

// Soft limits (mm from home position)
const float X_MAX_TRAVEL_MM = 300.0;   // 3018 X-axis travel
const float Y_MAX_TRAVEL_MM = 180.0;   // 3018 Y-axis travel
const float Z_MAX_TRAVEL_MM = 45.0;    // 3018 Z-axis travel
const float F_MAX_TRAVEL_MM = 30.0;    // Focus axis (estimated)

// Convert to steps
const long X_MAX_STEPS = (long)(X_MAX_TRAVEL_MM * STEPS_PER_MM);
const long Y_MAX_STEPS = (long)(Y_MAX_TRAVEL_MM * STEPS_PER_MM);
const long Z_MAX_STEPS = (long)(Z_MAX_TRAVEL_MM * STEPS_PER_MM);
const long F_MAX_STEPS = (long)(F_MAX_TRAVEL_MM * STEPS_PER_MM);

// Debounce settings
const unsigned long DEBOUNCE_MS = 300;
const unsigned long VERIFY_DELAY_MS = 20;

// Pulse width
const unsigned int PULSE_WIDTH_US = 10;

//==============================================================================
// STATE MACHINE
//==============================================================================
enum SystemState {
  STATE_IDLE,
  STATE_HOMING_Z,
  STATE_HOMING_Y,
  STATE_HOMING_X,
  STATE_HOMING_F,
  STATE_MOVING,
  STATE_JOGGING,
  STATE_ERROR
};

SystemState currentState = STATE_IDLE;
const char* stateNames[] = {
  "IDLE", "HOMING_Z", "HOMING_Y", "HOMING_X", "HOMING_F",
  "MOVING", "JOGGING", "ERROR"
};

//==============================================================================
// STEPPER OBJECTS
//==============================================================================
AccelStepper xStepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper yStepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper zStepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper fStepper(AccelStepper::DRIVER, F_STEP_PIN, F_DIR_PIN);

MultiStepper multiStepper;

//==============================================================================
// LIMIT SWITCH STATE
//==============================================================================
struct LimitState {
  volatile bool triggered;
  volatile bool isMin;  // true if MIN triggered, false if MAX
  volatile unsigned long lastTriggerTime;
};

LimitState xLimit = {false, false, 0};
LimitState yLimit = {false, false, 0};
LimitState zLimit = {false, false, 0};
LimitState fLimit = {false, false, 0};

//==============================================================================
// HOMING STATE
//==============================================================================
enum HomingPhase {
  HOMING_SEEK,      // Fast approach to limit
  HOMING_BACKOFF,   // Back away from limit
  HOMING_CREEP,     // Slow precision approach
  HOMING_COMPLETE
};

HomingPhase homingPhase = HOMING_SEEK;

//==============================================================================
// COMMAND PARSING
//==============================================================================
char cmdBuffer[128];
int cmdIndex = 0;

//==============================================================================
// TIMING
//==============================================================================
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 2000; // 2 seconds

//==============================================================================
// INTERRUPT SERVICE ROUTINES
//==============================================================================
void xMinISR() {
  unsigned long now = millis();
  if (now - xLimit.lastTriggerTime > DEBOUNCE_MS) {
    xLimit.triggered = true;
    xLimit.isMin = true;
    xLimit.lastTriggerTime = now;
  }
}

void xMaxISR() {
  unsigned long now = millis();
  if (now - xLimit.lastTriggerTime > DEBOUNCE_MS) {
    xLimit.triggered = true;
    xLimit.isMin = false;
    xLimit.lastTriggerTime = now;
  }
}

void yMinISR() {
  unsigned long now = millis();
  if (now - yLimit.lastTriggerTime > DEBOUNCE_MS) {
    yLimit.triggered = true;
    yLimit.isMin = true;
    yLimit.lastTriggerTime = now;
  }
}

void yMaxISR() {
  unsigned long now = millis();
  if (now - yLimit.lastTriggerTime > DEBOUNCE_MS) {
    yLimit.triggered = true;
    yLimit.isMin = false;
    yLimit.lastTriggerTime = now;
  }
}

void zMinISR() {
  unsigned long now = millis();
  if (now - zLimit.lastTriggerTime > DEBOUNCE_MS) {
    zLimit.triggered = true;
    zLimit.isMin = true;
    zLimit.lastTriggerTime = now;
  }
}

void zMaxISR() {
  unsigned long now = millis();
  if (now - zLimit.lastTriggerTime > DEBOUNCE_MS) {
    zLimit.triggered = true;
    zLimit.isMin = false;
    zLimit.lastTriggerTime = now;
  }
}

void fMinISR() {
  unsigned long now = millis();
  if (now - fLimit.lastTriggerTime > DEBOUNCE_MS) {
    fLimit.triggered = true;
    fLimit.isMin = true;
    fLimit.lastTriggerTime = now;
  }
}

void fMaxISR() {
  unsigned long now = millis();
  if (now - fLimit.lastTriggerTime > DEBOUNCE_MS) {
    fLimit.triggered = true;
    fLimit.isMin = false;
    fLimit.lastTriggerTime = now;
  }
}

//==============================================================================
// HELPER FUNCTIONS - FORWARD DECLARATIONS
//==============================================================================
void changeState(SystemState newState);
void emergencyStop();
void handleLimitHit();
void processCommand(const char* cmd);
void printStatus();
void homeAxis(AccelStepper& stepper, const char* axisName, 
              int minPin, int maxPin, LimitState& limitState, SystemState homingState);

//==============================================================================
// SETUP
//==============================================================================
void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(2000);  // Wait for serial connection
  
  // Print banner
  Serial.println("================================================================================");
  Serial.print("Teensy 4.1 Firmware: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("Build: ");
  Serial.println(BUILD_DATE);
  Serial.println("================================================================================");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize enable pin (active LOW for TB6600)
  pinMode(ENA_ALL_PIN, OUTPUT);
  digitalWrite(ENA_ALL_PIN, LOW);  // Enable all drivers
  
  // Initialize limit switches with pull-ups
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);
  pinMode(Z_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MAX_PIN, INPUT_PULLUP);
  pinMode(F_MIN_PIN, INPUT_PULLUP);
  pinMode(F_MAX_PIN, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(X_MIN_PIN), xMinISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(X_MAX_PIN), xMaxISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_MIN_PIN), yMinISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(Y_MAX_PIN), yMaxISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(Z_MIN_PIN), zMinISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(Z_MAX_PIN), zMaxISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(F_MIN_PIN), fMinISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(F_MAX_PIN), fMaxISR, FALLING);
  
  // Configure steppers
  xStepper.setMaxSpeed(MAX_SPEED);
  xStepper.setAcceleration(ACCEL);
  xStepper.setMinPulseWidth(PULSE_WIDTH_US);
  
  yStepper.setMaxSpeed(MAX_SPEED);
  yStepper.setAcceleration(ACCEL);
  yStepper.setMinPulseWidth(PULSE_WIDTH_US);
  
  zStepper.setMaxSpeed(MAX_SPEED);
  zStepper.setAcceleration(ACCEL);
  zStepper.setMinPulseWidth(PULSE_WIDTH_US);
  
  fStepper.setMaxSpeed(MAX_SPEED);
  fStepper.setAcceleration(ACCEL);
  fStepper.setMinPulseWidth(PULSE_WIDTH_US);
  
  // Add steppers to MultiStepper
  multiStepper.addStepper(xStepper);
  multiStepper.addStepper(yStepper);
  multiStepper.addStepper(zStepper);
  multiStepper.addStepper(fStepper);
  
  Serial.println("Hardware Configuration:");
  Serial.println("  - 4x NEMA17 steppers (X, Y, Z, Focus)");
  Serial.println("  - 4x TB6600 drivers (8 microsteps, active LOW enable)");
  Serial.println("  - 8x NO limit switches");
  Serial.println("  - T8 lead screws on X/Y/Z (8mm/rev)");
  Serial.println();
  Serial.println("Motion Parameters:");
  Serial.print("  - Max Speed: "); Serial.print(MAX_SPEED); Serial.println(" steps/sec");
  Serial.print("  - Acceleration: "); Serial.print(ACCEL); Serial.println(" steps/sec²");
  Serial.print("  - Pulse Width: "); Serial.print(PULSE_WIDTH_US); Serial.println("µs");
  Serial.print("  - Steps/mm: "); Serial.println(STEPS_PER_MM);
  Serial.println();
  Serial.println("Soft Limits:");
  Serial.print("  - X: 0 to "); Serial.print(X_MAX_TRAVEL_MM); Serial.println("mm");
  Serial.print("  - Y: 0 to "); Serial.print(Y_MAX_TRAVEL_MM); Serial.println("mm");
  Serial.print("  - Z: 0 to "); Serial.print(Z_MAX_TRAVEL_MM); Serial.println("mm");
  Serial.print("  - Focus: 0 to "); Serial.print(F_MAX_TRAVEL_MM); Serial.println("mm");
  Serial.println();
  Serial.println("Commands: HOME, MOVE X=? Y=? Z=? F=?, JOG X +5.5, STATUS, STOP, PING");
  Serial.println("================================================================================");
  Serial.println("READY - System in IDLE state");
  Serial.println();
  
  changeState(STATE_IDLE);
}

//==============================================================================
// MAIN LOOP
//==============================================================================
void loop() {
  // Critical: Always run steppers
  xStepper.run();
  yStepper.run();
  zStepper.run();
  fStepper.run();
  
  // Blink LED (heartbeat indicator)
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (millis() - lastBlink >= 500) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = millis();
  }
  
  // Check for limit switch hits
  handleLimitHit();
  
  // State machine
  switch (currentState) {
    case STATE_IDLE:
      // Nothing to do, waiting for commands
      break;
      
    case STATE_HOMING_Z:
    case STATE_HOMING_Y:
    case STATE_HOMING_X:
    case STATE_HOMING_F:
      // Homing handled by homeAxis function
      break;
      
    case STATE_MOVING:
      // Check if move complete
      if (xStepper.distanceToGo() == 0 && yStepper.distanceToGo() == 0 &&
          zStepper.distanceToGo() == 0 && fStepper.distanceToGo() == 0) {
        Serial.println("!MOVE_COMPLETE");
        changeState(STATE_IDLE);
      }
      break;
      
    case STATE_JOGGING:
      // Check if jog complete
      if (xStepper.distanceToGo() == 0 && yStepper.distanceToGo() == 0 &&
          zStepper.distanceToGo() == 0 && fStepper.distanceToGo() == 0) {
        Serial.println("!JOG_COMPLETE");
        changeState(STATE_IDLE);
      }
      break;
      
    case STATE_ERROR:
      // Remain in error state until reset
      break;
  }
  
  // Periodic heartbeat
  if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    Serial.print("!HEARTBEAT state=");
    Serial.print(stateNames[currentState]);
    Serial.print(" X=");
    Serial.print(xStepper.currentPosition() / STEPS_PER_MM, 2);
    Serial.print(" Y=");
    Serial.print(yStepper.currentPosition() / STEPS_PER_MM, 2);
    Serial.print(" Z=");
    Serial.print(zStepper.currentPosition() / STEPS_PER_MM, 2);
    Serial.print(" F=");
    Serial.println(fStepper.currentPosition() / STEPS_PER_MM, 2);
    lastHeartbeat = millis();
  }
  
  // Non-blocking serial command parsing
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';
        processCommand(cmdBuffer);
        cmdIndex = 0;
      }
    }
    else if (cmdIndex < 127) {
      cmdBuffer[cmdIndex++] = c;
    }
  }
}

//==============================================================================
// STATE MANAGEMENT
//==============================================================================
void changeState(SystemState newState) {
  if (newState != currentState) {
    Serial.print("STATE: ");
    Serial.print(stateNames[currentState]);
    Serial.print(" -> ");
    Serial.println(stateNames[newState]);
    currentState = newState;
  }
}

//==============================================================================
// EMERGENCY STOP
//==============================================================================
void emergencyStop() {
  Serial.println("!EMERGENCY_STOP");
  
  // Stop all steppers immediately
  xStepper.setCurrentPosition(xStepper.currentPosition());
  yStepper.setCurrentPosition(yStepper.currentPosition());
  zStepper.setCurrentPosition(zStepper.currentPosition());
  fStepper.setCurrentPosition(fStepper.currentPosition());
  
  changeState(STATE_ERROR);
}

//==============================================================================
// LIMIT SWITCH HANDLING
//==============================================================================
void handleLimitHit() {
  // Check each axis for limit hits
  if (xLimit.triggered) {
    xLimit.triggered = false;
    delay(VERIFY_DELAY_MS);
    
    bool minPressed = (digitalRead(X_MIN_PIN) == LOW);
    bool maxPressed = (digitalRead(X_MAX_PIN) == LOW);
    
    if (minPressed || maxPressed) {
      Serial.print("!LIMIT X_");
      Serial.println(xLimit.isMin ? "MIN" : "MAX");
      
      // Stop all axes on any limit hit
      emergencyStop();
    }
  }
  
  if (yLimit.triggered) {
    yLimit.triggered = false;
    delay(VERIFY_DELAY_MS);
    
    bool minPressed = (digitalRead(Y_MIN_PIN) == LOW);
    bool maxPressed = (digitalRead(Y_MAX_PIN) == LOW);
    
    if (minPressed || maxPressed) {
      Serial.print("!LIMIT Y_");
      Serial.println(yLimit.isMin ? "MIN" : "MAX");
      emergencyStop();
    }
  }
  
  if (zLimit.triggered) {
    zLimit.triggered = false;
    delay(VERIFY_DELAY_MS);
    
    bool minPressed = (digitalRead(Z_MIN_PIN) == LOW);
    bool maxPressed = (digitalRead(Z_MAX_PIN) == LOW);
    
    if (minPressed || maxPressed) {
      Serial.print("!LIMIT Z_");
      Serial.println(zLimit.isMin ? "MIN" : "MAX");
      emergencyStop();
    }
  }
  
  if (fLimit.triggered) {
    fLimit.triggered = false;
    delay(VERIFY_DELAY_MS);
    
    bool minPressed = (digitalRead(F_MIN_PIN) == LOW);
    bool maxPressed = (digitalRead(F_MAX_PIN) == LOW);
    
    if (minPressed || maxPressed) {
      Serial.print("!LIMIT F_");
      Serial.println(fLimit.isMin ? "MIN" : "MAX");
      emergencyStop();
    }
  }
}

//==============================================================================
// HOMING FUNCTION
//==============================================================================
void homeAxis(AccelStepper& stepper, const char* axisName, 
              int minPin, int maxPin, LimitState& limitState, SystemState homingState) {
  Serial.print("HOMING ");
  Serial.print(axisName);
  Serial.println("...");
  
  changeState(homingState);
  homingPhase = HOMING_SEEK;
  
  // Phase 1: Fast seek toward MIN limit
  Serial.print("  Phase 1: Seeking ");
  Serial.print(axisName);
  Serial.println("_MIN...");
  
  stepper.setMaxSpeed(HOMING_SPEED);
  stepper.move(-999999);  // Move toward negative (MIN)
  
  // Wait for limit or timeout
  unsigned long startTime = millis();
  const unsigned long TIMEOUT_MS = 120000;  // 2 minute timeout
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    
    // Check if MIN limit hit
    if (limitState.triggered && limitState.isMin) {
      limitState.triggered = false;
      delay(VERIFY_DELAY_MS);
      if (digitalRead(minPin) == LOW) {
        Serial.print("  ");
        Serial.print(axisName);
        Serial.println("_MIN found");
        break;
      }
    }
    
    // Check if MAX limit hit (wrong direction!)
    if (limitState.triggered && !limitState.isMin) {
      limitState.triggered = false;
      delay(VERIFY_DELAY_MS);
      if (digitalRead(maxPin) == LOW) {
        Serial.print("ERROR: ");
        Serial.print(axisName);
        Serial.println("_MAX hit during homing (wrong direction or wiring issue)");
        emergencyStop();
        return;
      }
    }
    
    // Timeout check
    if (millis() - startTime > TIMEOUT_MS) {
      Serial.print("ERROR: ");
      Serial.print(axisName);
      Serial.println(" homing timeout");
      emergencyStop();
      return;
    }
  }
  
  // Stop immediately
  stepper.setCurrentPosition(stepper.currentPosition());
  
  // Phase 2: Back off
  Serial.print("  Phase 2: Backing off ");
  Serial.print(BACKOFF_DISTANCE_MM);
  Serial.println("mm...");
  
  homingPhase = HOMING_BACKOFF;
  stepper.move(BACKOFF_STEPS);
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  Serial.println("  Back-off complete");
  
  // Phase 3: Slow precision approach
  Serial.print("  Phase 3: Precision approach to ");
  Serial.print(axisName);
  Serial.println("_MIN...");
  
  homingPhase = HOMING_CREEP;
  stepper.setMaxSpeed(HOMING_CREEP_SPEED);
  stepper.move(-999999);  // Move slowly toward MIN
  
  startTime = millis();
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    
    if (limitState.triggered && limitState.isMin) {
      limitState.triggered = false;
      delay(VERIFY_DELAY_MS);
      if (digitalRead(minPin) == LOW) {
        Serial.print("  ");
        Serial.print(axisName);
        Serial.println("_MIN re-acquired");
        break;
      }
    }
    
    if (millis() - startTime > TIMEOUT_MS) {
      Serial.print("ERROR: ");
      Serial.print(axisName);
      Serial.println(" precision homing timeout");
      emergencyStop();
      return;
    }
  }
  
  // Stop and set home position
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(MAX_SPEED);  // Restore normal speed
  
  Serial.print("SUCCESS: ");
  Serial.print(axisName);
  Serial.println(" homed at position 0.00mm");
  Serial.println();
  
  homingPhase = HOMING_COMPLETE;
}

//==============================================================================
// COMMAND PROCESSING
//==============================================================================
void processCommand(const char* cmd) {
  Serial.print("RX: ");
  Serial.println(cmd);
  
  // PING - connectivity test
  if (strcasecmp(cmd, "PING") == 0) {
    Serial.println("@OK PONG");
  }
  
  // STATUS - report current state and positions
  else if (strcasecmp(cmd, "STATUS") == 0) {
    printStatus();
  }
  
  // STOP - emergency stop
  else if (strcasecmp(cmd, "STOP") == 0) {
    emergencyStop();
    Serial.println("@OK STOPPED");
  }
  
  // HOME - sequential homing of all axes
  else if (strcasecmp(cmd, "HOME") == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("@ERROR Must be in IDLE state to home");
      return;
    }
    
    Serial.println("@OK Starting sequential homing (Z->Y->X->Focus)");
    Serial.println();
    
    // Home Z first (prevent crashes)
    homeAxis(zStepper, "Z", Z_MIN_PIN, Z_MAX_PIN, zLimit, STATE_HOMING_Z);
    if (currentState == STATE_ERROR) return;
    
    // Home Y
    homeAxis(yStepper, "Y", Y_MIN_PIN, Y_MAX_PIN, yLimit, STATE_HOMING_Y);
    if (currentState == STATE_ERROR) return;
    
    // Home X
    homeAxis(xStepper, "X", X_MIN_PIN, X_MAX_PIN, xLimit, STATE_HOMING_X);
    if (currentState == STATE_ERROR) return;
    
    // Home Focus
    homeAxis(fStepper, "Focus", F_MIN_PIN, F_MAX_PIN, fLimit, STATE_HOMING_F);
    if (currentState == STATE_ERROR) return;
    
    changeState(STATE_IDLE);
    Serial.println("================================================================================");
    Serial.println("HOMING COMPLETE - All axes at 0.00mm");
    Serial.println("================================================================================");
    Serial.println();
  }
  
  // MOVE - coordinated absolute move
  // Format: MOVE X=10.5 Y=20.3 Z=5.0 F=1.5
  else if (strncasecmp(cmd, "MOVE", 4) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("@ERROR Must be in IDLE state to move");
      return;
    }
    
    // Parse coordinates
    float xPos = xStepper.currentPosition() / STEPS_PER_MM;
    float yPos = yStepper.currentPosition() / STEPS_PER_MM;
    float zPos = zStepper.currentPosition() / STEPS_PER_MM;
    float fPos = fStepper.currentPosition() / STEPS_PER_MM;
    
    char* ptr = (char*)cmd + 4;
    while (*ptr) {
      while (*ptr == ' ') ptr++;  // Skip spaces
      
      if (*ptr == 'X' || *ptr == 'x') {
        ptr++;
        if (*ptr == '=') ptr++;
        xPos = atof(ptr);
      }
      else if (*ptr == 'Y' || *ptr == 'y') {
        ptr++;
        if (*ptr == '=') ptr++;
        yPos = atof(ptr);
      }
      else if (*ptr == 'Z' || *ptr == 'z') {
        ptr++;
        if (*ptr == '=') ptr++;
        zPos = atof(ptr);
      }
      else if (*ptr == 'F' || *ptr == 'f') {
        ptr++;
        if (*ptr == '=') ptr++;
        fPos = atof(ptr);
      }
      
      // Skip to next parameter
      while (*ptr && *ptr != ' ') ptr++;
    }
    
    // Check soft limits
    bool withinLimits = true;
    if (xPos < 0 || xPos > X_MAX_TRAVEL_MM) {
      Serial.print("@ERROR X position ");
      Serial.print(xPos);
      Serial.print(" outside limits [0, ");
      Serial.print(X_MAX_TRAVEL_MM);
      Serial.println("]");
      withinLimits = false;
    }
    if (yPos < 0 || yPos > Y_MAX_TRAVEL_MM) {
      Serial.print("@ERROR Y position ");
      Serial.print(yPos);
      Serial.print(" outside limits [0, ");
      Serial.print(Y_MAX_TRAVEL_MM);
      Serial.println("]");
      withinLimits = false;
    }
    if (zPos < 0 || zPos > Z_MAX_TRAVEL_MM) {
      Serial.print("@ERROR Z position ");
      Serial.print(zPos);
      Serial.print(" outside limits [0, ");
      Serial.print(Z_MAX_TRAVEL_MM);
      Serial.println("]");
      withinLimits = false;
    }
    if (fPos < 0 || fPos > F_MAX_TRAVEL_MM) {
      Serial.print("@ERROR Focus position ");
      Serial.print(fPos);
      Serial.print(" outside limits [0, ");
      Serial.print(F_MAX_TRAVEL_MM);
      Serial.println("]");
      withinLimits = false;
    }
    
    if (!withinLimits) return;
    
    // Convert to steps and set targets
    long xTarget = (long)(xPos * STEPS_PER_MM);
    long yTarget = (long)(yPos * STEPS_PER_MM);
    long zTarget = (long)(zPos * STEPS_PER_MM);
    long fTarget = (long)(fPos * STEPS_PER_MM);
    
    xStepper.moveTo(xTarget);
    yStepper.moveTo(yTarget);
    zStepper.moveTo(zTarget);
    fStepper.moveTo(fTarget);
    
    changeState(STATE_MOVING);
    
    Serial.print("@OK Moving to X=");
    Serial.print(xPos, 2);
    Serial.print(" Y=");
    Serial.print(yPos, 2);
    Serial.print(" Z=");
    Serial.print(zPos, 2);
    Serial.print(" F=");
    Serial.println(fPos, 2);
  }
  
  // JOG - incremental move on single axis
  // Format: JOG X +5.5 or JOG Y -2.3
  else if (strncasecmp(cmd, "JOG", 3) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("@ERROR Must be in IDLE state to jog");
      return;
    }
    
    char* ptr = (char*)cmd + 3;
    while (*ptr == ' ') ptr++;
    
    char axis = toupper(*ptr);
    ptr++;
    while (*ptr == ' ') ptr++;
    
    float distance = atof(ptr);
    
    AccelStepper* stepper = nullptr;
    float maxTravel = 0;
    
    if (axis == 'X') {
      stepper = &xStepper;
      maxTravel = X_MAX_TRAVEL_MM;
    }
    else if (axis == 'Y') {
      stepper = &yStepper;
      maxTravel = Y_MAX_TRAVEL_MM;
    }
    else if (axis == 'Z') {
      stepper = &zStepper;
      maxTravel = Z_MAX_TRAVEL_MM;
    }
    else if (axis == 'F') {
      stepper = &fStepper;
      maxTravel = F_MAX_TRAVEL_MM;
    }
    else {
      Serial.println("@ERROR Invalid axis (must be X, Y, Z, or F)");
      return;
    }
    
    // Calculate new position
    float currentPos = stepper->currentPosition() / STEPS_PER_MM;
    float newPos = currentPos + distance;
    
    // Check soft limits
    if (newPos < 0 || newPos > maxTravel) {
      Serial.print("@ERROR Jog would exceed limits [0, ");
      Serial.print(maxTravel);
      Serial.println("]");
      return;
    }
    
    // Execute jog
    long jogSteps = (long)(distance * STEPS_PER_MM);
    stepper->move(jogSteps);
    
    changeState(STATE_JOGGING);
    
    Serial.print("@OK Jogging ");
    Serial.print(axis);
    Serial.print(" ");
    Serial.print(distance, 2);
    Serial.println("mm");
  }
  
  // Unknown command
  else {
    Serial.print("@ERROR Unknown command: ");
    Serial.println(cmd);
  }
}

//==============================================================================
// STATUS REPORTING
//==============================================================================
void printStatus() {
  Serial.println("@STATUS");
  Serial.print("  State: ");
  Serial.println(stateNames[currentState]);
  Serial.print("  X: ");
  Serial.print(xStepper.currentPosition() / STEPS_PER_MM, 2);
  Serial.print("mm (");
  Serial.print(xStepper.currentPosition());
  Serial.println(" steps)");
  Serial.print("  Y: ");
  Serial.print(yStepper.currentPosition() / STEPS_PER_MM, 2);
  Serial.print("mm (");
  Serial.print(yStepper.currentPosition());
  Serial.println(" steps)");
  Serial.print("  Z: ");
  Serial.print(zStepper.currentPosition() / STEPS_PER_MM, 2);
  Serial.print("mm (");
  Serial.print(zStepper.currentPosition());
  Serial.println(" steps)");
  Serial.print("  Focus: ");
  Serial.print(fStepper.currentPosition() / STEPS_PER_MM, 2);
  Serial.print("mm (");
  Serial.print(fStepper.currentPosition());
  Serial.println(" steps)");
  Serial.print("  Limits: X_MIN=");
  Serial.print(digitalRead(X_MIN_PIN) == LOW ? "CLOSED" : "OPEN");
  Serial.print(" X_MAX=");
  Serial.print(digitalRead(X_MAX_PIN) == LOW ? "CLOSED" : "OPEN");
  Serial.print(" Y_MIN=");
  Serial.print(digitalRead(Y_MIN_PIN) == LOW ? "CLOSED" : "OPEN");
  Serial.print(" Y_MAX=");
  Serial.print(digitalRead(Y_MAX_PIN) == LOW ? "CLOSED" : "OPEN");
  Serial.print(" Z_MIN=");
  Serial.print(digitalRead(Z_MIN_PIN) == LOW ? "CLOSED" : "OPEN");
  Serial.print(" Z_MAX=");
  Serial.print(digitalRead(Z_MAX_PIN) == LOW ? "CLOSED" : "OPEN");
  Serial.print(" F_MIN=");
  Serial.print(digitalRead(F_MIN_PIN) == LOW ? "CLOSED" : "OPEN");
  Serial.print(" F_MAX=");
  Serial.println(digitalRead(F_MAX_PIN) == LOW ? "CLOSED" : "OPEN");
}
