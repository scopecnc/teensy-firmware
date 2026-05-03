/*******************************************************************************
 * Teensy 4.1 CNC/Microscope Motion Controller
 * 
 * Version: 1.2 - With Robust Protocol and Diagnostic Menu System
 * Date: December 2025
 * 
 * Description:
 *   4-axis stepper motor controller for robotic microscope on 3018 CNC frame.
 *   Supports sequential homing, coordinated multi-axis moves, jogging, and
 *   limit switch protection. Designed for long-running autonomous operation.
 *   
 *   Features robust master/slave protocol with checksums, ACK/NACK, retries,
 *   watchdog timer, and detailed error reporting. See PROTOCOL.md for details.
 *   
 *   Boot-time diagnostic menu system for manual testing and calibration.
 *   Press 'D' within 3 seconds of boot to enter diagnostic mode.
 * 
 * Hardware:
 *   - Teensy 4.1 (i.MX RT1062, 600MHz)
 *   - 4x TB6600 stepper drivers (1/16 microsteps, inverted ENA logic)
 *   - 4x NEMA17 steppers (200 steps/rev)
 *   - 8x NO limit switches (min/max per axis)
 *   - T8 lead screws (8mm/rev) on X, Y, Z axes
 * 
 * Protocol Commands (Master/Slave mode - default):
 *   !CONNECT          - Establish connection with master
 *   !PING             - Watchdog keepalive
 *   !HOME [axes]      - Sequential homing (Z→Y→X→Focus)
 *   !HOME_FAST        - Parallel homing (all axes simultaneously)
 *   !MOVE X# Y# Z# F# - Coordinated absolute move (mm)
 *   !JOG <axis> <dist>- Incremental move
 *   !STATUS           - Report positions and state
 *   !STOP             - Emergency stop all motion
 *   !DIAG_ENTER       - Enter diagnostic mode
 *   !DIAG_EXIT        - Exit diagnostic mode
 *   See PROTOCOL.md for complete command reference
 * 
 * Diagnostic Mode:
 *   Hierarchical numbered menus for manual control and testing
 *   Global commands: T (top), X (exit), S (stop), U (up), H (help), ? (status)
 * 
 * Notes:
 *   - Robust protocol with checksums, sequence numbers, ACK/NACK
 *   - 5-second watchdog timer with automatic motor shutdown
 *   - Detailed error codes (20+ categories)
 *   - Asynchronous startup (either device can boot first)
 *   - All motion obeys limit switches and soft limits
 *   - Non-blocking architecture for smooth operation
 ******************************************************************************/

#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

//==============================================================================
// VERSION & BUILD INFO
//==============================================================================
const char* FIRMWARE_VERSION = "v1.2";
const char* BUILD_DATE = __DATE__ " " __TIME__;
const int PROTOCOL_VERSION = 1;

// Set to true to see verbose protocol logging on USB Serial
const bool DEBUG_PROTOCOL = false;

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
const int F_MIN_PIN = 22;  // Moved from pin 19/20 (I2C conflict)
const int F_MAX_PIN = 18;

//==============================================================================
// MOTION CONFIGURATION
//==============================================================================
// Speed settings (conservative for testing with long cables)
const float MAX_SPEED = 6000.0;        // steps/sec (~224 RPM, 30 mm/sec)
const float ACCEL = 2000.0;            // steps/sec²
const float HOMING_SPEED = 3000.0;     // steps/sec for initial seek
const float HOMING_CREEP_SPEED = 400.0; // steps/sec for precision approach

// Motor test speeds (reduced for smoother operation)
const float TEST_MOTOR_SPEED = 4500.0;  // 75% of MAX_SPEED
const float TEST_MOTOR_ACCEL = 1500.0;  // 75% of ACCEL

// Mechanical parameters (3018 CNC + T8 lead screws)
const float STEPS_PER_MM = 400.0;      // Calibrated: 100mm command = 50mm actual (1/16 microstepping or T4 screws)
const float BACKOFF_DISTANCE_MM = 5.0; // Back off 5mm after hitting limit
const long BACKOFF_STEPS = (long)(BACKOFF_DISTANCE_MM * STEPS_PER_MM);

// F axis (zoom) uses belt drive with 1:4 ratio - needs different backoff
// 7° zoom ring rotation × 4 (belt ratio) = 28° stepper = ~249 steps
const long F_BACKOFF_STEPS = 249;

// Soft limits (mm from home position)
const float X_MAX_TRAVEL_MM = 270.0;   // 3018 X-axis travel (measured)
const float Y_MAX_TRAVEL_MM = 160.0;   // 3018 Y-axis travel (measured, max 180)
const float Z_MAX_TRAVEL_MM = 30.0;    // 3018 Z-axis travel (measured)
const float F_MAX_TRAVEL_MM = 11.5;    // Zoom axis (measured - belt drive)

// Convert to steps
const long X_MAX_STEPS = (long)(X_MAX_TRAVEL_MM * STEPS_PER_MM);
const long Y_MAX_STEPS = (long)(Y_MAX_TRAVEL_MM * STEPS_PER_MM);
const long Z_MAX_STEPS = (long)(Z_MAX_TRAVEL_MM * STEPS_PER_MM);
const long F_MAX_STEPS = (long)(F_MAX_TRAVEL_MM * STEPS_PER_MM);

// Debounce settings
const unsigned long DEBOUNCE_MS = 50;  // 50ms is sufficient for mechanical switch bounce
const unsigned long VERIFY_DELAY_MS = 20;

// Pulse width (TB6600 minimum is 2.5µs, using 10µs for margin)
const unsigned int PULSE_WIDTH_US = 10;

//==============================================================================
// STATE MACHINE
//==============================================================================
enum SystemState {
  STATE_BOOT,
  STATE_DISCONNECTED,
  STATE_CONNECTED,
  STATE_IDLE,
  STATE_HOMING_Z,
  STATE_HOMING_Y,
  STATE_HOMING_X,
  STATE_HOMING_F,
  STATE_HOMING,       // Generic homing state for protocol
  STATE_MOVING,
  STATE_JOGGING,
  STATE_ERROR,
  STATE_DIAGNOSTIC,
  STATE_COMM_LOST
};

SystemState currentState = STATE_BOOT;
const char* stateNames[] = {
  "BOOT", "DISCONNECTED", "CONNECTED", "IDLE", 
  "HOMING_Z", "HOMING_Y", "HOMING_X", "HOMING_F", "HOMING",
  "MOVING", "JOGGING", "ERROR", "DIAGNOSTIC", "COMM_LOST"
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
  volatile unsigned long debounceUntil;  // Timestamp when we can accept next trigger
  volatile int lastMinPinState;  // Last known state of MIN pin (HIGH/LOW) for hysteresis
  volatile int lastMaxPinState;  // Last known state of MAX pin (HIGH/LOW) for hysteresis
};

LimitState xLimit = {false, false, 0, 0, HIGH, HIGH};
LimitState yLimit = {false, false, 0, 0, HIGH, HIGH};
LimitState zLimit = {false, false, 0, 0, HIGH, HIGH};
LimitState fLimit = {false, false, 0, 0, HIGH, HIGH};

//==============================================================================
// HOMING STATE
//==============================================================================
enum HomingPhase {
  HOMING_IDLE,      // Not homing
  HOMING_SEEK,      // Fast approach to limit
  HOMING_BACKOFF,   // Back away from limit
  HOMING_CREEP,     // Slow precision approach
  HOMING_COMPLETE
};

HomingPhase homingPhase = HOMING_IDLE;

// Per-axis homing phase for parallel homing
HomingPhase xHomingPhase = HOMING_IDLE;
HomingPhase yHomingPhase = HOMING_IDLE;
HomingPhase zHomingPhase = HOMING_IDLE;
HomingPhase fHomingPhase = HOMING_IDLE;
bool fastHomingActive = false;

//==============================================================================
// COMMAND PARSING
//==============================================================================
char cmdBuffer[128];
int cmdIndex = 0;

// Diagnostic mode input buffer (for multi-character commands like "4 10")
char diagInputBuffer[64];
int diagInputIndex = 0;

//==============================================================================
// TIMING
//==============================================================================
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 2000; // 2 seconds

//==============================================================================
// PROTOCOL STATE
//==============================================================================
bool protocolMode = false;         // true = protocol control, false = diagnostic menu
bool masterConnected = false;      // Master has sent !CONNECT
uint8_t lastSeqNum = 0;            // Last sequence number received
uint8_t seqHistory[10] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
int seqHistoryIndex = 0;

// Watchdog timer
unsigned long lastMasterMessage = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 5000;  // 5 seconds
bool watchdogEnabled = false;

// Homing state tracking
bool xHomed = false;
bool yHomed = false;
bool zHomed = false;
bool fHomed = false;

// Error state
char lastErrorCode[32] = "";
char lastErrorMessage[128] = "";

//==============================================================================
// DIAGNOSTIC MODE
//==============================================================================
bool diagnosticMode = false;
bool homedThisSession = false;

enum DiagnosticMenu {
  MENU_MAIN,
  MENU_MOVE,
  MENU_JOG,
  MENU_TEST_MOTORS,
  MENU_SAFE_LIMIT_TEST,
  MENU_SPEED_SETTINGS,
  MENU_CALIBRATION
};

DiagnosticMenu currentMenu = MENU_MAIN;

// For submenus that need state
struct MoveTargets {
  float x, y, z, f;
};
MoveTargets pendingMove = {0, 0, 0, 0};

float lastJogDistance = 1.0;  // Default jog distance
bool testMotorRunning = false;
AccelStepper* quickTestStepper = nullptr;
LimitState* quickTestLimit = nullptr;
int quickTestMinPin = 0;
int quickTestMaxPin = 0;
bool quickTestTowardMax = false;
bool quickTestBackingOff = false;

// Safe limit test state
bool safeLimitTestRunning = false;
AccelStepper* testStepper = nullptr;
const char* testAxisName = "";
int testMinPin = 0;
int testMaxPin = 0;
LimitState* testLimit = nullptr;
bool testExpectMin = true;  // true = expect MIN, false = expect MAX
unsigned long lastStepTime = 0;
const unsigned long STEP_PAUSE_MS = 1000;  // 1 second pause between 2mm moves
const float SAFE_TEST_STEP_MM = 2.0;
const long SAFE_TEST_STEP_STEPS = (long)(SAFE_TEST_STEP_MM * STEPS_PER_MM);

//==============================================================================
// INTERRUPT SERVICE ROUTINES
//==============================================================================
void xMinISR() {
  // Check for HIGH→LOW transition (hysteresis prevents release bounce)
  unsigned long now = millis();
  int pinState = digitalRead(X_MIN_PIN);
  if (now >= xLimit.debounceUntil && pinState == LOW && xLimit.lastMinPinState == HIGH) {
    xLimit.triggered = true;
    xLimit.isMin = true;
    xLimit.lastTriggerTime = now;
    xLimit.debounceUntil = now + DEBOUNCE_MS;
    xLimit.lastMinPinState = LOW;  // Remember we detected LOW
  }
}

void xMaxISR() {
  unsigned long now = millis();
  int pinState = digitalRead(X_MAX_PIN);
  if (now >= xLimit.debounceUntil && pinState == LOW && xLimit.lastMaxPinState == HIGH) {
    xLimit.triggered = true;
    xLimit.isMin = false;
    xLimit.lastTriggerTime = now;
    xLimit.debounceUntil = now + DEBOUNCE_MS;
    xLimit.lastMaxPinState = LOW;
  }
}

void yMinISR() {
  unsigned long now = millis();
  int pinState = digitalRead(Y_MIN_PIN);
  if (now >= yLimit.debounceUntil && pinState == LOW && yLimit.lastMinPinState == HIGH) {
    yLimit.triggered = true;
    yLimit.isMin = true;
    yLimit.lastTriggerTime = now;
    yLimit.debounceUntil = now + DEBOUNCE_MS;
    yLimit.lastMinPinState = LOW;
  }
}

void yMaxISR() {
  unsigned long now = millis();
  int pinState = digitalRead(Y_MAX_PIN);
  if (now >= yLimit.debounceUntil && pinState == LOW && yLimit.lastMaxPinState == HIGH) {
    yLimit.triggered = true;
    yLimit.isMin = false;
    yLimit.lastTriggerTime = now;
    yLimit.debounceUntil = now + DEBOUNCE_MS;
    yLimit.lastMaxPinState = LOW;
  }
}

void zMinISR() {
  unsigned long now = millis();
  int pinState = digitalRead(Z_MIN_PIN);
  if (now >= zLimit.debounceUntil && pinState == LOW && zLimit.lastMinPinState == HIGH) {
    zLimit.triggered = true;
    zLimit.isMin = true;
    zLimit.lastTriggerTime = now;
    zLimit.debounceUntil = now + DEBOUNCE_MS;
    zLimit.lastMinPinState = LOW;
  }
}

void zMaxISR() {
  unsigned long now = millis();
  int pinState = digitalRead(Z_MAX_PIN);
  if (now >= zLimit.debounceUntil && pinState == LOW && zLimit.lastMaxPinState == HIGH) {
    zLimit.triggered = true;
    zLimit.isMin = false;
    zLimit.lastTriggerTime = now;
    zLimit.debounceUntil = now + DEBOUNCE_MS;
    zLimit.lastMaxPinState = LOW;
  }
}

void fMinISR() {
  unsigned long now = millis();
  int pinState = digitalRead(F_MIN_PIN);
  if (now >= fLimit.debounceUntil && pinState == LOW && fLimit.lastMinPinState == HIGH) {
    fLimit.triggered = true;
    fLimit.isMin = true;
    fLimit.lastTriggerTime = now;
    fLimit.debounceUntil = now + DEBOUNCE_MS;
    fLimit.lastMinPinState = LOW;
  }
}

void fMaxISR() {
  unsigned long now = millis();
  int pinState = digitalRead(F_MAX_PIN);
  if (now >= fLimit.debounceUntil && pinState == LOW && fLimit.lastMaxPinState == HIGH) {
    fLimit.triggered = true;
    fLimit.isMin = false;
    fLimit.lastTriggerTime = now;
    fLimit.debounceUntil = now + DEBOUNCE_MS;
    fLimit.lastMaxPinState = LOW;
  }
}

//==============================================================================
// PROTOCOL HELPER FUNCTIONS
//==============================================================================

// Calculate XOR checksum
uint8_t calculateChecksum(const char* message) {
  uint8_t checksum = 0;
  while (*message) {
    checksum ^= *message;
    message++;
  }
  return checksum;
}

// Add checksum to message and send
void sendProtocolMessage(const char* message) {
  // Strip trailing spaces from message before checksum calculation
  char trimmedMsg[256];
  strncpy(trimmedMsg, message, sizeof(trimmedMsg) - 1);
  trimmedMsg[sizeof(trimmedMsg) - 1] = '\0';
  
  // Remove trailing spaces
  int len = strlen(trimmedMsg);
  while (len > 0 && trimmedMsg[len - 1] == ' ') {
    trimmedMsg[--len] = '\0';
  }
  
  uint8_t checksum = calculateChecksum(trimmedMsg);
  
  // Send to RPI4 on Serial1
  Serial1.printf("%s *%02X\n", trimmedMsg, checksum);
  Serial1.flush();
  
  // Echo to USB debug (when enabled)
  if (DEBUG_PROTOCOL) {
    unsigned long timestamp = millis();
    Serial.printf("[%lu] TX→RPI: %s *%02X\n", timestamp, trimmedMsg, checksum);
  }
}

// Send ACK response
void sendAck(uint8_t seq, const char* data = nullptr) {
  char msg[256];
  if (data) {
    snprintf(msg, sizeof(msg), "@ACK %d %s", seq, data);
  } else {
    snprintf(msg, sizeof(msg), "@ACK %d", seq);
  }
  sendProtocolMessage(msg);
}

// Send NACK response
void sendNack(uint8_t seq, const char* errorCode, const char* errorMessage) {
  char msg[256];
  snprintf(msg, sizeof(msg), "@NACK %d %s %s", seq, errorCode, errorMessage);
  sendProtocolMessage(msg);
  
  // Store error for status queries
  strncpy(lastErrorCode, errorCode, sizeof(lastErrorCode) - 1);
  strncpy(lastErrorMessage, errorMessage, sizeof(lastErrorMessage) - 1);
}

// Send diagnostic output message
void sendDiagMessage(const char* text) {
  char msg[256];
  if (text[0] == '\0') {
    // Empty string - send without trailing space
    snprintf(msg, sizeof(msg), "#DIAG");
  } else {
    snprintf(msg, sizeof(msg), "#DIAG %s", text);
  }
  sendProtocolMessage(msg);
}

// Helper function to print a line both locally and to RPI4 (if in diag mode)
void diagPrintln(const char* text) {
  Serial.println(text);
  if (diagnosticMode) sendDiagMessage(text);
}

// Helper for printing with no arguments (blank line)
void diagPrintln() {
  Serial.println();
  if (diagnosticMode) sendDiagMessage("");
}

// Helper for printing formatted strings
void diagPrintf(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
  if (diagnosticMode) {
    // For partial lines, we'll send them immediately
    // (RPI4 will need to handle partial lines or we buffer them)
    sendDiagMessage(buffer);
  }
}

// Send STATUS response
void sendStatus(uint8_t seq) {
  char msg[256];
  char homedStr[16] = "";
  
  if (xHomed) strcat(homedStr, "X");
  if (yHomed) strcat(homedStr, "Y");
  if (zHomed) strcat(homedStr, "Z");
  if (fHomed) strcat(homedStr, "F");
  if (strlen(homedStr) == 0) strcpy(homedStr, "None");
  
  snprintf(msg, sizeof(msg), "@STATUS %d X=%.2f Y=%.2f Z=%.2f F=%.2f STATE=%s HOMED=%s",
           seq,
           xStepper.currentPosition() / STEPS_PER_MM,
           yStepper.currentPosition() / STEPS_PER_MM,
           zStepper.currentPosition() / STEPS_PER_MM,
           fStepper.currentPosition() / STEPS_PER_MM,
           stateNames[currentState],
           homedStr);
  sendProtocolMessage(msg);
}

// Send COMPLETE response
void sendComplete(uint8_t seq, const char* data = nullptr) {
  char msg[256];
  if (data) {
    snprintf(msg, sizeof(msg), "@COMPLETE %d %s", seq, data);
  } else {
    snprintf(msg, sizeof(msg), "@COMPLETE %d", seq);
  }
  sendProtocolMessage(msg);
}

// Send EVENT (unsolicited)
void sendEvent(const char* eventName, const char* data = nullptr) {
  char msg[256];
  if (data) {
    snprintf(msg, sizeof(msg), "#%s %s", eventName, data);
  } else {
    snprintf(msg, sizeof(msg), "#%s", eventName);
  }
  sendProtocolMessage(msg);
}

// Check if sequence number is a duplicate
bool isDuplicateSeq(uint8_t seq) {
  for (int i = 0; i < 10; i++) {
    if (seqHistory[i] == seq) return true;
  }
  return false;
}

// Record sequence number
void recordSeq(uint8_t seq) {
  seqHistory[seqHistoryIndex] = seq;
  seqHistoryIndex = (seqHistoryIndex + 1) % 10;
  lastSeqNum = seq;
}

// Validate checksum of received message
bool validateChecksum(const char* line, char* messageOut, size_t messageOutSize) {
  // Find checksum delimiter
  const char* asterisk = strrchr(line, '*');
  if (!asterisk) {
    return false;  // No checksum
  }
  
  // Extract message part (before *)
  size_t messageLen = asterisk - line;
  if (messageLen >= messageOutSize) {
    return false;  // Message too long
  }
  strncpy(messageOut, line, messageLen);
  messageOut[messageLen] = '\0';
  
  // Parse received checksum
  unsigned int receivedChecksum;
  if (sscanf(asterisk + 1, "%02X", &receivedChecksum) != 1) {
    return false;  // Invalid checksum format
  }
  
  // Calculate expected checksum
  uint8_t calculatedChecksum = calculateChecksum(messageOut);
  
  return (calculatedChecksum == (uint8_t)receivedChecksum);
}

// Disable all motors (safety)
void disableMotors() {
  digitalWrite(ENA_ALL_PIN, HIGH);  // Disable (active LOW)
}

// Enable all motors
void enableMotors() {
  digitalWrite(ENA_ALL_PIN, LOW);  // Enable (active LOW)
}

//==============================================================================
// DIAGNOSTIC MENU FUNCTIONS - FORWARD DECLARATIONS
//==============================================================================
void printMainMenu();
void printMoveMenu();
void printJogMenu();
void printTestMotorsMenu();
void printSpeedSettingsMenu();
void printCalibrationMenu();
void printGlobalCommands();
void processDiagnosticCommand(char cmd);
void processDiagnosticMultiCommand(const char* cmd);  // For multi-char commands like "4 10"
void testSingleMotor(AccelStepper& stepper, const char* name, bool towardMax,
                     int minPin, int maxPin, LimitState& limit);
bool checkHomingWarning();

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
void fastHomeAll();

// Common limit switch helpers
// Function removed - interrupts are never detached with new timestamp-based approach
bool checkLimitTriggered(LimitState* limit, int minPin, int maxPin);

// Protocol command processing
void processProtocolCommand(const char* message, uint8_t seq);

//==============================================================================
// SETUP
//==============================================================================
void setup() {
  // Initialize serial ports
  Serial.begin(115200);   // USB for debug output
  Serial1.begin(115200);  // Hardware UART on pins 0/1 for RPI4
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
  
  // Debug: Check initial pin states
  delay(100);  // Let pullups settle
  Serial.println("Initial limit switch states:");
  Serial.print("  X_MIN (pin "); Serial.print(X_MIN_PIN); Serial.print("): "); 
  Serial.println(digitalRead(X_MIN_PIN) == HIGH ? "HIGH (open)" : "LOW (closed)");
  Serial.print("  X_MAX (pin "); Serial.print(X_MAX_PIN); Serial.print("): "); 
  Serial.println(digitalRead(X_MAX_PIN) == HIGH ? "HIGH (open)" : "LOW (closed)");
  Serial.print("  Y_MIN (pin "); Serial.print(Y_MIN_PIN); Serial.print("): "); 
  Serial.println(digitalRead(Y_MIN_PIN) == HIGH ? "HIGH (open)" : "LOW (closed)");
  Serial.print("  Y_MAX (pin "); Serial.print(Y_MAX_PIN); Serial.print("): "); 
  Serial.println(digitalRead(Y_MAX_PIN) == HIGH ? "HIGH (open)" : "LOW (closed)");
  Serial.print("  Z_MIN (pin "); Serial.print(Z_MIN_PIN); Serial.print("): "); 
  Serial.println(digitalRead(Z_MIN_PIN) == HIGH ? "HIGH (open)" : "LOW (closed)");
  Serial.print("  Z_MAX (pin "); Serial.print(Z_MAX_PIN); Serial.print("): "); 
  Serial.println(digitalRead(Z_MAX_PIN) == HIGH ? "HIGH (open)" : "LOW (closed)");
  Serial.print("  F_MIN (pin "); Serial.print(F_MIN_PIN); Serial.print("): "); 
  Serial.println(digitalRead(F_MIN_PIN) == HIGH ? "HIGH (open)" : "LOW (closed)");
  Serial.print("  F_MAX (pin "); Serial.print(F_MAX_PIN); Serial.print("): "); 
  Serial.println(digitalRead(F_MAX_PIN) == HIGH ? "HIGH (open)" : "LOW (closed)");
  Serial.println();
  
  // Configure steppers
  xStepper.setMaxSpeed(MAX_SPEED);
  xStepper.setAcceleration(ACCEL);
  xStepper.setMinPulseWidth(PULSE_WIDTH_US);
  xStepper.setPinsInverted(true, false, false);  // Invert DIR pin
  
  yStepper.setMaxSpeed(MAX_SPEED);
  yStepper.setAcceleration(ACCEL);
  yStepper.setMinPulseWidth(PULSE_WIDTH_US);
  
  zStepper.setMaxSpeed(MAX_SPEED);
  zStepper.setAcceleration(ACCEL);
  zStepper.setMinPulseWidth(PULSE_WIDTH_US);
  zStepper.setPinsInverted(true, false, false);  // Invert DIR pin
  
  fStepper.setMaxSpeed(MAX_SPEED);
  fStepper.setAcceleration(ACCEL);
  fStepper.setMinPulseWidth(PULSE_WIDTH_US);
  fStepper.setPinsInverted(true, false, false);  // Invert DIR pin
  
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
  
  // Check for diagnostic mode entry
  Serial.println("Press [D] for Diagnostic Mode, or wait for Protocol Mode...");
  Serial.println("(3 second timeout)");
  unsigned long bootStart = millis();
  bool userInput = false;
  
  while (millis() - bootStart < 3000) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'D' || c == 'd') {
        diagnosticMode = true;
        currentState = STATE_DIAGNOSTIC;
        Serial.println();
        Serial.println("*** ENTERING DIAGNOSTIC MODE ***");
        Serial.println();
        delay(500);
        printMainMenu();
        userInput = true;
        break;
      }
    }
    
    // Countdown indicator
    unsigned long elapsed = millis() - bootStart;
    if (elapsed % 1000 < 50) {  // Show once per second
      int remaining = 3 - (elapsed / 1000);
      if (remaining > 0) {
        Serial.print(remaining);
        Serial.print("... ");
        delay(50);  // Debounce the timer display
      }
    }
  }
  
  if (!diagnosticMode) {
    // Protocol mode - send BOOT event
    Serial.println();
    Serial.println("*** PROTOCOL MODE ***");
    Serial.println("RPI4 connection: Serial1 (pins 0/1, 115200 baud)");
    Serial.println("Debug output: USB Serial (this console)");
    Serial.println("Waiting for master connection...");
    Serial.println("See PROTOCOL.md for command reference.");
    Serial.println();
    
    // Send BOOT event
    char bootMsg[128];
    snprintf(bootMsg, sizeof(bootMsg), "BOOT FW=%s AXES=4", FIRMWARE_VERSION);
    sendEvent(bootMsg);
    
    currentState = STATE_DISCONNECTED;
    protocolMode = true;
    watchdogEnabled = false;  // Not enabled until CONNECT
    
    Serial.println("State: DISCONNECTED (waiting for !CONNECT)");
    Serial.println("All protocol messages will be echoed with timestamps.");
    Serial.println();
  }
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
  
  // Protocol mode: Watchdog timer
  if (protocolMode && watchdogEnabled && !diagnosticMode) {
    if (millis() - lastMasterMessage > WATCHDOG_TIMEOUT_MS) {
      // Watchdog expired!
      Serial.println("!!! WATCHDOG TIMEOUT !!!");
      sendEvent("COMM_LOST", "Watchdog timeout");
      
      emergencyStop();
      disableMotors();
      
      currentState = STATE_COMM_LOST;
      watchdogEnabled = false;
      masterConnected = false;
      
      // Auto-recovery: resend BOOT every 10 seconds
      static unsigned long lastBootResend = 0;
      lastBootResend = millis();
      
      // Wait and attempt recovery
      while (currentState == STATE_COMM_LOST) {
        if (millis() - lastBootResend > 10000) {
          char bootMsg[128];
          snprintf(bootMsg, sizeof(bootMsg), "BOOT FW=%s AXES=4", FIRMWARE_VERSION);
          sendEvent(bootMsg);
          Serial.println("Attempting auto-recovery...");
          lastBootResend = millis();
        }
        
        // Check for CONNECT command from RPI4
        if (Serial1.available() > 0) {
          String line = Serial1.readStringUntil('\n');
          line.trim();
          if (DEBUG_PROTOCOL) Serial.printf("[%lu] RX←RPI (recovery): %s\n", millis(), line.c_str());
          if (line.startsWith("!CONNECT")) {
            // Process the CONNECT to recover
            char msg[256];
            if (validateChecksum(line.c_str(), msg, sizeof(msg))) {
              processProtocolCommand(msg, 0);  // Will handle CONNECT
              break;  // Exit recovery loop
            }
          }
        }
        
        delay(100);
      }
    }
  }
  
  // Update limit switch pin states for hysteresis (always track current state)
  // This must happen continuously so ISRs can detect HIGH->LOW transitions
  if (digitalRead(X_MIN_PIN) == HIGH) xLimit.lastMinPinState = HIGH;
  if (digitalRead(X_MAX_PIN) == HIGH) xLimit.lastMaxPinState = HIGH;
  if (digitalRead(Y_MIN_PIN) == HIGH) yLimit.lastMinPinState = HIGH;
  if (digitalRead(Y_MAX_PIN) == HIGH) yLimit.lastMaxPinState = HIGH;
  if (digitalRead(Z_MIN_PIN) == HIGH) zLimit.lastMinPinState = HIGH;
  if (digitalRead(Z_MAX_PIN) == HIGH) zLimit.lastMaxPinState = HIGH;
  if (digitalRead(F_MIN_PIN) == HIGH) fLimit.lastMinPinState = HIGH;
  if (digitalRead(F_MAX_PIN) == HIGH) fLimit.lastMaxPinState = HIGH;
  
  // Check for limit switch hits (production mode only - diagnostic tests handle their own limits)
  if (!diagnosticMode && currentState != STATE_DIAGNOSTIC) {
    handleLimitHit();
  }
  
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
  
  // Non-blocking serial command parsing
  // Read from USB Serial for diagnostic mode (local only)
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Only process USB input in diagnostic mode if not using protocol mode
    // (Protocol diagnostic mode is controlled via Serial1 RPI4 commands only)
    if ((diagnosticMode || currentState == STATE_DIAGNOSTIC) && !protocolMode) {
      // Diagnostic mode: buffer input until Enter is pressed
      if (c == '\n' || c == '\r') {
        if (diagInputIndex > 0) {
          diagInputBuffer[diagInputIndex] = '\0';
          // Check if it's a multi-char command (contains space, like "4 10")
          if (strchr(diagInputBuffer, ' ') != NULL) {
            processDiagnosticMultiCommand(diagInputBuffer);
          } else {
            // Single character command
            processDiagnosticCommand(diagInputBuffer[0]);
          }
          diagInputIndex = 0;
        }
      } else if (diagInputIndex < 63) {
        diagInputBuffer[diagInputIndex++] = c;
        // Echo character back
        Serial.print(c);
      }
    }
    // In protocol mode, ignore USB serial (only for debug output)
  }
  
  // Read from Serial1 (pins 0/1) for RPI4 protocol commands
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    
    if (protocolMode) {
      // Protocol mode: line-buffered commands with checksums (works in both normal and diagnostic mode)
      if (c == '\n' || c == '\r') {
        if (cmdIndex > 0) {
          cmdBuffer[cmdIndex] = '\0';
          
          // Echo received message to USB debug
          if (DEBUG_PROTOCOL) Serial.printf("[%lu] RX←RPI: %s\n", millis(), cmdBuffer);
          
          // Validate checksum
          char message[256];
          if (validateChecksum(cmdBuffer, message, sizeof(message))) {
            // Extract sequence number (if present)
            uint8_t seq = 0;
            if (message[0] == '!') {
              // Parse seq from command if present (optional for now)
              // For simplicity, we'll use lastSeqNum + 1
              seq = (lastSeqNum + 1) % 256;
            }
            
            processProtocolCommand(message, seq);
          } else {
            // Send NACK for checksum failure
            uint8_t seq = (lastSeqNum + 1) % 256;
            lastSeqNum = seq;
            Serial.println("ERROR: Checksum validation failed");
            char response[128];
            snprintf(response, sizeof(response), "@NACK %d ERR_CHECKSUM Checksum validation failed", seq);
            sendProtocolMessage(response);
          }
          
          cmdIndex = 0;
        }
      }
      else if (cmdIndex < 127) {
        cmdBuffer[cmdIndex++] = c;
      }
      else {
        // Buffer overflow protection: discard command and reset
        Serial.printf("[%lu] ERROR: Command buffer overflow, discarding\n", millis());
        cmdIndex = 0;
        // Read until newline to clear the bad command
        while (Serial1.available() > 0 && c != '\n' && c != '\r') {
          c = Serial1.read();
        }
      }
    }
    else {
      // Legacy mode (shouldn't reach here)
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
  
  // Diagnostic mode: Handle quick motor test (runs until limit hit)
  if (diagnosticMode && testMotorRunning) {
    // Use interrupt-based limit detection (with triple-read filtering)
    if (!quickTestBackingOff && checkLimitTriggered(quickTestLimit, quickTestMinPin, quickTestMaxPin)) {
      // Limit hit! Stop and start backoff
      quickTestStepper->setCurrentPosition(quickTestStepper->currentPosition());
      
      // Check if correct limit was hit
      bool hitCorrectLimit = (quickTestTowardMax && !quickTestLimit->isMin) || 
                              (!quickTestTowardMax && quickTestLimit->isMin);
      
      Serial.println();
      if (hitCorrectLimit) {
        Serial.print("✓ ");
        Serial.print(quickTestLimit->isMin ? "MIN" : "MAX");
        Serial.println(" limit reached! Backing off 5mm...");
      } else {
        Serial.print("⚠ WARNING: ");
        Serial.print(quickTestLimit->isMin ? "MIN" : "MAX");
        Serial.print(" limit hit (expected ");
        Serial.print(quickTestTowardMax ? "MAX" : "MIN");
        Serial.println(")! Backing off 5mm...");
        Serial.println("Check wiring polarity or motor direction");
      }
      
      // Back off 5mm in correct direction
      // If hit MIN: back off positive (toward MAX)
      // If hit MAX: back off negative (toward MIN)
      quickTestBackingOff = true;
      long backoffDistance = quickTestLimit->isMin ? BACKOFF_STEPS : -BACKOFF_STEPS;
      quickTestStepper->move(backoffDistance);
      
      quickTestLimit->triggered = false;  // Clear flag
    }
    
    // Interrupts are always enabled now - no need to re-enable
    
    // Check if backoff complete
    if (quickTestBackingOff && quickTestStepper->distanceToGo() == 0) {
      // Restore normal speeds
      xStepper.setMaxSpeed(MAX_SPEED);
      xStepper.setAcceleration(ACCEL);
      yStepper.setMaxSpeed(MAX_SPEED);
      yStepper.setAcceleration(ACCEL);
      zStepper.setMaxSpeed(MAX_SPEED);
      zStepper.setAcceleration(ACCEL);
      fStepper.setMaxSpeed(MAX_SPEED);
      fStepper.setAcceleration(ACCEL);
      
      testMotorRunning = false;
      Serial.println();
      Serial.println("Motor test complete.");
      Serial.println();
      printTestMotorsMenu();
    }
  }
  
  // Diagnostic mode: Handle safe limit test
  if (diagnosticMode && safeLimitTestRunning) {
    // Use interrupt-based limit detection (with triple-read filtering)
    if (checkLimitTriggered(testLimit, testMinPin, testMaxPin)) {
      // Stop immediately
      testStepper->setCurrentPosition(testStepper->currentPosition());
      
      // Restore normal speed
      testStepper->setMaxSpeed(MAX_SPEED);
      testStepper->setAcceleration(ACCEL);
      
      safeLimitTestRunning = false;
      testLimit->triggered = false;  // Clear flag
      
      // Report result
      Serial.println();
      if (testExpectMin && testLimit->isMin) {
        Serial.print("✓ PASS - ");
        Serial.print(testAxisName);
        Serial.println(" to MIN reached expected limit");
      } else if (!testExpectMin && !testLimit->isMin) {
        Serial.print("✓ PASS - ");
        Serial.print(testAxisName);
        Serial.println(" to MAX reached expected limit");
      } else if (testExpectMin && !testLimit->isMin) {
        Serial.print("⚠ WARNING - ");
        Serial.print(testAxisName);
        Serial.println(" to MIN reached MAX limit instead!");
        Serial.println("Check wiring polarity or motor direction");
      } else if (!testExpectMin && testLimit->isMin) {
        Serial.print("⚠ WARNING - ");
        Serial.print(testAxisName);
        Serial.println(" to MAX reached MIN limit instead!");
        Serial.println("Check wiring polarity or motor direction");
      }
      Serial.println();
      printTestMotorsMenu();
      
      // Interrupts stay enabled
    }
    
    // Interrupts are always enabled now
    
    // Check if current move complete - start next step
    if (testStepper->distanceToGo() == 0 && safeLimitTestRunning) {
      unsigned long now = millis();
      if (now - lastStepTime >= STEP_PAUSE_MS) {
        // Start next 2mm move
        long distance = testExpectMin ? -SAFE_TEST_STEP_STEPS : SAFE_TEST_STEP_STEPS;
        testStepper->move(distance);
        lastStepTime = now;
      }
    }
  }
  
  // Production mode: Heartbeat
  if (!diagnosticMode && millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
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
//==============================================================================
// COMMON LIMIT SWITCH HELPERS
//==============================================================================

// Re-enable interrupts for an axis (only if buttons released)
// No longer needed - interrupts are never detached with new timestamp-based approach

// Hybrid interrupt + polling approach with triple-read verification and hysteresis
// This eliminates race conditions while maintaining robust debouncing
bool checkLimitTriggered(LimitState* limit, int minPin, int maxPin) {
  unsigned long now = millis();
  
  // Update pin state tracking for hysteresis (reset to HIGH when released)
  int minState = digitalRead(minPin);
  int maxState = digitalRead(maxPin);
  if (minState == HIGH) limit->lastMinPinState = HIGH;
  if (maxState == HIGH) limit->lastMaxPinState = HIGH;
  
  // First: Check interrupt flag and verify with triple-read
  if (limit->triggered) {
    int limitPin = limit->isMin ? minPin : maxPin;
    // Triple-read verification to reject noise
    if (digitalRead(limitPin) == LOW) {
      delayMicroseconds(100);
      if (digitalRead(limitPin) == LOW) {
        delayMicroseconds(100);
        if (digitalRead(limitPin) == LOW) {
          limit->triggered = false;  // Clear flag for next time
          return true;  // Verified trigger!
        }
      }
    }
    // False trigger from noise
    limit->triggered = false;
  }
  
  // FALLBACK: Direct polling to catch any missed interrupts
  // Only poll if we're past the debounce window and pin transitioned HIGH→LOW
  if (now >= limit->debounceUntil) {
    // Check MIN pin (only if it was HIGH before)
    if (limit->lastMinPinState == HIGH && digitalRead(minPin) == LOW) {
      delayMicroseconds(100);
      if (digitalRead(minPin) == LOW) {
        delayMicroseconds(100);
        if (digitalRead(minPin) == LOW) {
          // Caught a press that interrupt missed!
          limit->triggered = true;
          limit->isMin = true;
          limit->lastTriggerTime = now;
          limit->debounceUntil = now + DEBOUNCE_MS;
          limit->lastMinPinState = LOW;
          return true;
        }
      }
    }
    // Check MAX pin (only if it was HIGH before)
    if (limit->lastMaxPinState == HIGH && digitalRead(maxPin) == LOW) {
      delayMicroseconds(100);
      if (digitalRead(maxPin) == LOW) {
        delayMicroseconds(100);
        if (digitalRead(maxPin) == LOW) {
          // Caught a press that interrupt missed!
          limit->triggered = true;
          limit->isMin = false;
          limit->lastTriggerTime = now;
          limit->debounceUntil = now + DEBOUNCE_MS;
          limit->lastMaxPinState = LOW;
          return true;
        }
      }
    }
  }
  
  return false;
}

//==============================================================================
// PRODUCTION MODE LIMIT HANDLER
//==============================================================================

void handleLimitHit() {
  // Check each axis for limit hits using common helper
  if (checkLimitTriggered(&xLimit, X_MIN_PIN, X_MAX_PIN)) {
    Serial.print("!LIMIT X_");
    Serial.println(xLimit.isMin ? "MIN" : "MAX");
    xLimit.triggered = false;
    emergencyStop();
  }
  
  if (checkLimitTriggered(&yLimit, Y_MIN_PIN, Y_MAX_PIN)) {
    Serial.print("!LIMIT Y_");
    Serial.println(yLimit.isMin ? "MIN" : "MAX");
    yLimit.triggered = false;
    emergencyStop();
  }
  
  if (checkLimitTriggered(&zLimit, Z_MIN_PIN, Z_MAX_PIN)) {
    Serial.print("!LIMIT Z_");
    Serial.println(zLimit.isMin ? "MIN" : "MAX");
    zLimit.triggered = false;
    emergencyStop();
  }
  
  if (checkLimitTriggered(&fLimit, F_MIN_PIN, F_MAX_PIN)) {
    Serial.print("!LIMIT F_");
    Serial.println(fLimit.isMin ? "MIN" : "MAX");
    fLimit.triggered = false;
    emergencyStop();
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
  
  // Determine max travel for this axis for safety checking
  float maxTravel;
  if (&stepper == &xStepper) maxTravel = X_MAX_TRAVEL_MM;
  else if (&stepper == &yStepper) maxTravel = Y_MAX_TRAVEL_MM;
  else if (&stepper == &zStepper) maxTravel = Z_MAX_TRAVEL_MM;
  else maxTravel = 200.0; // F axis conservative estimate
  
  long maxSteps = (long)(maxTravel * STEPS_PER_MM * 1.5); // 150% safety margin
  long startPosition = stepper.currentPosition();
  
  stepper.move(-999999);  // Move toward negative (MIN)
  
  // Wait for limit or timeout
  unsigned long startTime = millis();
  const unsigned long TIMEOUT_MS = 120000;  // 2 minute timeout
  
  while (stepper.distanceToGo() != 0) {
    // CRITICAL: Run ALL steppers to avoid missed steps
    xStepper.run();
    yStepper.run();
    zStepper.run();
    fStepper.run();
    
    // SAFETY: Check if we've traveled too far (limit switch failed to trigger)
    long stepsTraveled = abs(stepper.currentPosition() - startPosition);
    if (stepsTraveled > maxSteps) {
      Serial.print("ERROR: ");
      Serial.print(axisName);
      Serial.print(" traveled ");
      Serial.print(stepsTraveled / STEPS_PER_MM);
      Serial.print("mm without hitting limit! Max expected: ");
      Serial.print(maxTravel);
      Serial.println("mm - LIMIT SWITCH FAILURE or WIRING ISSUE");
      emergencyStop();
      return;
    }
    
    // Re-enable interrupts if button released
    // Interrupts are always enabled now
    
    // Check for limit hit using common helper
    if (checkLimitTriggered(&limitState, minPin, maxPin)) {
      if (limitState.isMin) {
        Serial.print("  ");
        Serial.print(axisName);
        Serial.println("_MIN found");
        limitState.triggered = false;
        break;
      } else {
        // Wrong limit hit
        Serial.print("ERROR: ");
        Serial.print(axisName);
        Serial.println("_MAX hit during homing (wrong direction or wiring issue)");
        limitState.triggered = false;
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
    // CRITICAL: Run ALL steppers
    xStepper.run();
    yStepper.run();
    zStepper.run();
    fStepper.run();
  }
  
  Serial.println("  Back-off complete");
  
  // Phase 3: Slow precision approach
  Serial.print("  Phase 3: Precision approach to ");
  Serial.print(axisName);
  Serial.println("_MIN...");
  
  homingPhase = HOMING_CREEP;
  stepper.setMaxSpeed(HOMING_CREEP_SPEED);
  startPosition = stepper.currentPosition(); // Reset for phase 3 safety check
  stepper.move(-999999);  // Move slowly toward MIN
  
  startTime = millis();
  while (stepper.distanceToGo() != 0) {
    // CRITICAL: Run ALL steppers
    xStepper.run();
    yStepper.run();
    zStepper.run();
    fStepper.run();
    
    // SAFETY: Check step count - should hit limit within backoff distance
    long stepsTraveled = abs(stepper.currentPosition() - startPosition);
    if (stepsTraveled > BACKOFF_STEPS * 2) { // Should find limit within 2x backoff distance
      Serial.print("ERROR: ");
      Serial.print(axisName);
      Serial.print(" precision approach traveled ");
      Serial.print(stepsTraveled / STEPS_PER_MM);
      Serial.println("mm without re-acquiring limit - LIMIT SWITCH FAILURE");
      emergencyStop();
      return;
    }
    
    // Interrupts are always enabled now
    
    // Check for limit hit using common helper
    if (checkLimitTriggered(&limitState, minPin, maxPin)) {
      if (limitState.isMin) {
        Serial.print("  ");
        Serial.print(axisName);
        Serial.println("_MIN re-acquired");
        limitState.triggered = false;
        break;
      } else {
        // Wrong limit hit
        limitState.triggered = false;
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
  
  // Stop immediately at limit
  stepper.setCurrentPosition(stepper.currentPosition());
  
  // Phase 4: Final backoff to release switch
  Serial.print("  Phase 4: Final backoff from ");
  Serial.print(axisName);
  Serial.println("_MIN...");
  
  stepper.setMaxSpeed(MAX_SPEED);  // Restore normal speed
  stepper.move(BACKOFF_STEPS);
  
  while (stepper.distanceToGo() != 0) {
    // CRITICAL: Run ALL steppers
    xStepper.run();
    yStepper.run();
    zStepper.run();
    fStepper.run();
  }
  
  // Set home position (0 is now 5mm away from limit)
  stepper.setCurrentPosition(0);
  
  Serial.print("SUCCESS: ");
  Serial.print(axisName);
  Serial.println(" homed at position 0.00mm");
  Serial.println();
  
  homingPhase = HOMING_COMPLETE;
}

//==============================================================================
// FAST HOME ALL AXES (PARALLEL)
//==============================================================================
void fastHomeAll() {
  const unsigned long TIMEOUT_MS = 120000;  // 2 minute timeout
  
  Serial.println();
  Serial.println("================================================================================");
  Serial.println("FAST HOME ALL AXES (PARALLEL)");
  Serial.println("================================================================================");
  Serial.println("Homing all axes simultaneously: Z, Y, X, F");
  Serial.println();
  
  // Initialize all axes to seek phase
  xHomingPhase = HOMING_SEEK;
  yHomingPhase = HOMING_SEEK;
  zHomingPhase = HOMING_SEEK;
  fHomingPhase = HOMING_SEEK;
  fastHomingActive = true;
  
  // Clear all limit flags
  xLimit.triggered = false;
  yLimit.triggered = false;
  zLimit.triggered = false;
  fLimit.triggered = false;
  
  // Set speeds for fast seek
  xStepper.setMaxSpeed(HOMING_SPEED);
  yStepper.setMaxSpeed(HOMING_SPEED);
  zStepper.setMaxSpeed(HOMING_SPEED);
  fStepper.setMaxSpeed(HOMING_SPEED);
  
  // Start all moving toward MIN
  xStepper.move(-999999);
  yStepper.move(-999999);
  zStepper.move(-999999);
  fStepper.move(-999999);
  
  unsigned long startTime = millis();
  
  // Main homing loop
  while (fastHomingActive) {
    // Run all steppers
    xStepper.run();
    yStepper.run();
    zStepper.run();
    fStepper.run();
    
    // Interrupts are always enabled now - no need to re-enable
    
    // Handle X axis
    if (xHomingPhase == HOMING_SEEK) {
      if (checkLimitTriggered(&xLimit, X_MIN_PIN, X_MAX_PIN) && xLimit.isMin) {
        xStepper.setCurrentPosition(xStepper.currentPosition());
        xStepper.move(BACKOFF_STEPS);
        xHomingPhase = HOMING_BACKOFF;
        xLimit.triggered = false;
      }
    } else if (xHomingPhase == HOMING_BACKOFF) {
      if (xStepper.distanceToGo() == 0) {
        xStepper.setMaxSpeed(HOMING_CREEP_SPEED);
        xStepper.move(-999999);
        xHomingPhase = HOMING_CREEP;
      }
    } else if (xHomingPhase == HOMING_CREEP) {
      if (checkLimitTriggered(&xLimit, X_MIN_PIN, X_MAX_PIN) && xLimit.isMin) {
        xStepper.setCurrentPosition(xStepper.currentPosition());
        xStepper.move(BACKOFF_STEPS);
        xHomingPhase = HOMING_COMPLETE;
        xLimit.triggered = false;
        Serial.println("✓ X homed");
      }
    } else if (xHomingPhase == HOMING_COMPLETE) {
      if (xStepper.distanceToGo() == 0) {
        xStepper.setCurrentPosition(0);
        xStepper.setMaxSpeed(MAX_SPEED);
        xHomingPhase = HOMING_IDLE;
      }
    }
    
    // Handle Y axis
    if (yHomingPhase == HOMING_SEEK) {
      if (checkLimitTriggered(&yLimit, Y_MIN_PIN, Y_MAX_PIN) && yLimit.isMin) {
        yStepper.setCurrentPosition(yStepper.currentPosition());
        yStepper.move(BACKOFF_STEPS);
        yHomingPhase = HOMING_BACKOFF;
        yLimit.triggered = false;
      }
    } else if (yHomingPhase == HOMING_BACKOFF) {
      if (yStepper.distanceToGo() == 0) {
        yStepper.setMaxSpeed(HOMING_CREEP_SPEED);
        yStepper.move(-999999);
        yHomingPhase = HOMING_CREEP;
      }
    } else if (yHomingPhase == HOMING_CREEP) {
      if (checkLimitTriggered(&yLimit, Y_MIN_PIN, Y_MAX_PIN) && yLimit.isMin) {
        yStepper.setCurrentPosition(yStepper.currentPosition());
        yStepper.move(BACKOFF_STEPS);
        yHomingPhase = HOMING_COMPLETE;
        yLimit.triggered = false;
        Serial.println("✓ Y homed");
      }
    } else if (yHomingPhase == HOMING_COMPLETE) {
      if (yStepper.distanceToGo() == 0) {
        yStepper.setCurrentPosition(0);
        yStepper.setMaxSpeed(MAX_SPEED);
        yHomingPhase = HOMING_IDLE;
      }
    }
    
    // Handle Z axis
    if (zHomingPhase == HOMING_SEEK) {
      if (checkLimitTriggered(&zLimit, Z_MIN_PIN, Z_MAX_PIN) && zLimit.isMin) {
        zStepper.setCurrentPosition(zStepper.currentPosition());
        zStepper.move(BACKOFF_STEPS);
        zHomingPhase = HOMING_BACKOFF;
        zLimit.triggered = false;
      }
    } else if (zHomingPhase == HOMING_BACKOFF) {
      if (zStepper.distanceToGo() == 0) {
        zStepper.setMaxSpeed(HOMING_CREEP_SPEED);
        zStepper.move(-999999);
        zHomingPhase = HOMING_CREEP;
      }
    } else if (zHomingPhase == HOMING_CREEP) {
      if (checkLimitTriggered(&zLimit, Z_MIN_PIN, Z_MAX_PIN) && zLimit.isMin) {
        zStepper.setCurrentPosition(zStepper.currentPosition());
        zStepper.move(BACKOFF_STEPS);
        zHomingPhase = HOMING_COMPLETE;
        zLimit.triggered = false;
        Serial.println("✓ Z homed");
      }
    } else if (zHomingPhase == HOMING_COMPLETE) {
      if (zStepper.distanceToGo() == 0) {
        zStepper.setCurrentPosition(0);
        zStepper.setMaxSpeed(MAX_SPEED);
        zHomingPhase = HOMING_IDLE;
      }
    }
    
    // Handle F axis
    if (fHomingPhase == HOMING_SEEK) {
      if (checkLimitTriggered(&fLimit, F_MIN_PIN, F_MAX_PIN) && fLimit.isMin) {
        fStepper.setCurrentPosition(fStepper.currentPosition());
        fStepper.move(F_BACKOFF_STEPS);
        fHomingPhase = HOMING_BACKOFF;
        fLimit.triggered = false;
      }
    } else if (fHomingPhase == HOMING_BACKOFF) {
      if (fStepper.distanceToGo() == 0) {
        fStepper.setMaxSpeed(HOMING_CREEP_SPEED);
        fStepper.move(-999999);
        fHomingPhase = HOMING_CREEP;
      }
    } else if (fHomingPhase == HOMING_CREEP) {
      if (checkLimitTriggered(&fLimit, F_MIN_PIN, F_MAX_PIN) && fLimit.isMin) {
        fStepper.setCurrentPosition(fStepper.currentPosition());
        fStepper.move(F_BACKOFF_STEPS);
        fHomingPhase = HOMING_COMPLETE;
        fLimit.triggered = false;
        Serial.println("✓ F homed");
      }
    } else if (fHomingPhase == HOMING_COMPLETE) {
      if (fStepper.distanceToGo() == 0) {
        fStepper.setCurrentPosition(0);
        fStepper.setMaxSpeed(MAX_SPEED);
        fHomingPhase = HOMING_IDLE;
      }
    }
    
    // Check if all done
    if (xHomingPhase == HOMING_IDLE && yHomingPhase == HOMING_IDLE && 
        zHomingPhase == HOMING_IDLE && fHomingPhase == HOMING_IDLE) {
      fastHomingActive = false;
    }
    
    // Timeout check
    if (millis() - startTime > TIMEOUT_MS) {
      Serial.println("ERROR: Fast homing timeout");
      emergencyStop();
      fastHomingActive = false;
      return;
    }
  }
  
  Serial.println();
  Serial.println("SUCCESS: All axes homed");
  Serial.println();
  printMainMenu();
}

//==============================================================================
// DIAGNOSTIC MENU SYSTEM
//==============================================================================
void printGlobalCommands() {
  const char* line = "Global: [T]op [U]p [X]exit [S]top [H]elp [?]status";
  Serial.println(line);
  if (diagnosticMode) sendDiagMessage(line);
}

void printMainMenu() {
  const char* lines[] = {
    "================================================================================",
    "DIAGNOSTIC MAIN MENU",
    "================================================================================",
    "1 - HOME All Axes (sequential Z->Y->X->F)",
    "2 - FAST HOME All Axes (parallel)",
    "3 - MOVE (absolute positioning)",
    "4 - JOG (incremental moves)",
    "5 - STATUS (detailed report)",
    "6 - TEST LIMIT SWITCHES",
    "7 - TEST MOTORS",
    "8 - SPEED SETTINGS",
    "9 - SYSTEM INFO",
    "",
    NULL  // Marker for global commands
  };
  
  for (int i = 0; lines[i] != NULL; i++) {
    Serial.println(lines[i]);
    if (diagnosticMode) sendDiagMessage(lines[i]);
  }
  
  printGlobalCommands();
  
  const char* footer = "================================================================================";
  Serial.println(footer);
  if (diagnosticMode) sendDiagMessage(footer);
  
  Serial.print("> ");
  if (diagnosticMode) sendDiagMessage(">");
}

void printMoveMenu() {
  char buffer[128];
  
  diagPrintln("================================================================================");
  diagPrintln("MOVE MENU - Absolute Positioning");
  diagPrintln("================================================================================");
  
  snprintf(buffer, sizeof(buffer), "Current position: X=%.2f Y=%.2f Z=%.2f F=%.2f",
           xStepper.currentPosition() / STEPS_PER_MM,
           yStepper.currentPosition() / STEPS_PER_MM,
           zStepper.currentPosition() / STEPS_PER_MM,
           fStepper.currentPosition() / STEPS_PER_MM);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "Target position:  X=%.2f Y=%.2f Z=%.2f F=%.2f",
           pendingMove.x, pendingMove.y, pendingMove.z, pendingMove.f);
  diagPrintln(buffer);
  
  diagPrintln();
  diagPrintln("1 X Y Z F - Set target (e.g. 1 50 100 10 5)");
  diagPrintln("2 - EXECUTE MOVE to target");
  diagPrintln("3 - Reset target to current position");
  diagPrintln();
  printGlobalCommands();
  diagPrintln("================================================================================");
  Serial.print("> ");
  if (diagnosticMode) sendDiagMessage(">");
}

void printJogMenu() {
  char buffer[128];
  
  diagPrintln("================================================================================");
  diagPrintln("JOG MENU - Incremental Moves");
  diagPrintln("================================================================================");
  
  snprintf(buffer, sizeof(buffer), "Current: X=%.2f Y=%.2f Z=%.2f F=%.2f",
           xStepper.currentPosition() / STEPS_PER_MM,
           yStepper.currentPosition() / STEPS_PER_MM,
           zStepper.currentPosition() / STEPS_PER_MM,
           fStepper.currentPosition() / STEPS_PER_MM);
  diagPrintln(buffer);
  
  diagPrintln();
  diagPrintln("1 DIST - Jog X by DIST mm (e.g. 1 5.5 or 1 -10)");
  diagPrintln("2 DIST - Jog Y by DIST mm");
  diagPrintln("3 DIST - Jog Z by DIST mm");
  diagPrintln("4 DIST - Jog F by DIST mm");
  diagPrintln();
  printGlobalCommands();
  diagPrintln("================================================================================");
  Serial.print("> ");
  if (diagnosticMode) sendDiagMessage(">");
}

void printTestMotorsMenu() {
  diagPrintln("================================================================================");
  diagPrintln("MOTOR TEST MENU");
  diagPrintln("================================================================================");
  diagPrintln("Quick Tests (5 seconds):");
  diagPrintln("1 - X to MIN     2 - X to MAX");
  diagPrintln("3 - Y to MIN     4 - Y to MAX");
  diagPrintln("5 - Z to MIN     6 - Z to MAX");
  diagPrintln("7 - F to MIN     8 - F to MAX");
  diagPrintln();
  diagPrintln("9 - Safe Limit Tests (2mm incremental)");
  diagPrintln();
  printGlobalCommands();
  diagPrintln("================================================================================");
  Serial.print("> ");
  if (diagnosticMode) sendDiagMessage(">");
}

void printSafeLimitTestMenu() {
  diagPrintln("================================================================================");
  diagPrintln("SAFE LIMIT TEST MENU");
  diagPrintln("================================================================================");
  diagPrintln("Incremental approach (2mm steps, 1 sec pause):");
  diagPrintln("1 - X to MIN     2 - X to MAX");
  diagPrintln("3 - Y to MIN     4 - Y to MAX");
  diagPrintln("5 - Z to MIN     6 - Z to MAX");
  diagPrintln("7 - F to MIN     8 - F to MAX");
  diagPrintln();
  diagPrintln("Reports PASS if expected limit reached");
  diagPrintln("Reports WARNING if wrong limit (polarity issue)");
  diagPrintln();
  printGlobalCommands();
  diagPrintln("================================================================================");
  Serial.print("> ");
  if (diagnosticMode) sendDiagMessage(">");
}

void printSpeedSettingsMenu() {
  char buffer[128];
  
  diagPrintln("================================================================================");
  diagPrintln("SPEED SETTINGS MENU");
  diagPrintln("================================================================================");
  
  snprintf(buffer, sizeof(buffer), "Current speed: %lu steps/sec (%.0f%% of max)",
           MAX_SPEED, (MAX_SPEED / 6000.0) * 100);
  diagPrintln(buffer);
  
  diagPrintln();
  diagPrintln("Select preset:");
  diagPrintln("1 - 25% (1500 steps/sec, conservative - CURRENT)");
  diagPrintln("2 - 50% (3000 steps/sec, moderate)");
  diagPrintln("3 - 75% (4500 steps/sec, fast)");
  diagPrintln("4 - 100% (6000 steps/sec, full speed)");
  diagPrintln();
  diagPrintln("Note: Speed changes take effect after reboot");
  diagPrintln();
  printGlobalCommands();
  diagPrintln("================================================================================");
  Serial.print("> ");
  if (diagnosticMode) sendDiagMessage(">");
}

void printCalibrationMenu() {
  char buffer[128];
  
  diagPrintln("================================================================================");
  diagPrintln("CALIBRATION TOOLS");
  diagPrintln("================================================================================");
  
  snprintf(buffer, sizeof(buffer), "Current Steps/mm: %.2f", STEPS_PER_MM);
  diagPrintln(buffer);
  
  diagPrintln();
  diagPrintln("1 - Move Fixed Distance (measure with calipers)");
  diagPrintln("2 - Report Steps Taken (for last move)");
  diagPrintln("3 - Calculate Steps/mm (from measurement)");
  diagPrintln("4 - Show Current Soft Limits");
  diagPrintln("5 - Manual Step Control (precise positioning)");
  diagPrintln();
  printGlobalCommands();
  diagPrintln("================================================================================");
  Serial.print("> ");
  if (diagnosticMode) sendDiagMessage(">");
}

void printSystemInfo() {
  char buffer[128];
  
  diagPrintln("================================================================================");
  diagPrintln("SYSTEM INFORMATION");
  diagPrintln("================================================================================");
  
  snprintf(buffer, sizeof(buffer), "Firmware: %s", FIRMWARE_VERSION);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "Build Date: %s", BUILD_DATE);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "Uptime: %lu seconds", millis() / 1000);
  diagPrintln(buffer);
  
  diagPrintln();
  diagPrintln("Hardware:");
  diagPrintln("  - Teensy 4.1 (i.MX RT1062, 600MHz)");
  diagPrintln("  - 4x TB6600 drivers (8 microsteps)");
  diagPrintln("  - 4x NEMA17 steppers");
  diagPrintln("  - 8x NO limit switches");
  diagPrintln();
  diagPrintln("Motion Configuration:");
  
  snprintf(buffer, sizeof(buffer), "  - Max Speed: %lu steps/sec", MAX_SPEED);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  - Acceleration: %lu steps/sec^2", ACCEL);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  - Pulse Width: %uus", PULSE_WIDTH_US);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  - Steps/mm: %.2f", STEPS_PER_MM);
  diagPrintln(buffer);
  
  diagPrintln();
  diagPrintln("Soft Limits:");
  
  snprintf(buffer, sizeof(buffer), "  - X: 0 to %.2fmm", X_MAX_TRAVEL_MM);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  - Y: 0 to %.2fmm", Y_MAX_TRAVEL_MM);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  - Z: 0 to %.2fmm", Z_MAX_TRAVEL_MM);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  - Focus: 0 to %.2fmm", F_MAX_TRAVEL_MM);
  diagPrintln(buffer);
  
  diagPrintln();
  snprintf(buffer, sizeof(buffer), "Homed This Session: %s", homedThisSession ? "YES" : "NO");
  diagPrintln(buffer);
  
  diagPrintln();
  diagPrintln("Press any key to return...");
}

bool checkHomingWarning() {
  if (!homedThisSession) {
    Serial.println();
    Serial.println("⚠ WARNING: Axes not homed this session!");
    Serial.println("Position may be inaccurate. Soft limits disabled.");
    Serial.print("Continue anyway? (Y/N): ");
    
    // Wait for response
    unsigned long timeout = millis() + 10000;  // 10 second timeout
    while (millis() < timeout) {
      if (Serial.available() > 0) {
        char c = Serial.read();
        Serial.println(c);
        if (c == 'Y' || c == 'y') {
          return true;  // Continue
        } else {
          Serial.println("Operation cancelled.");
          return false;  // Cancel
        }
      }
    }
    Serial.println();
    Serial.println("Timeout - operation cancelled.");
    return false;
  }
  return true;  // Already homed, proceed
}

void startSafeLimitTest(AccelStepper& stepper, const char* axisName, 
                        int minPin, int maxPin, LimitState& limit, bool towardMin) {
  Serial.println();
  Serial.print("SAFE LIMIT TEST: ");
  Serial.print(axisName);
  Serial.print(" toward ");
  Serial.println(towardMin ? "MIN" : "MAX");
  Serial.println("Moving in 2mm steps with 1 second pause...");
  Serial.println("Press S to stop");
  Serial.println();
  
  testStepper = &stepper;
  testAxisName = axisName;
  testMinPin = minPin;
  testMaxPin = maxPin;
  testLimit = &limit;
  testExpectMin = towardMin;
  
  // Clear limit flags
  limit.triggered = false;
  
  // Set slow speed for safety
  stepper.setMaxSpeed(HOMING_CREEP_SPEED);  // Slow speed
  stepper.setAcceleration(TEST_MOTOR_ACCEL);
  
  safeLimitTestRunning = true;
  lastStepTime = millis();
  
  // Start first move
  long distance = towardMin ? -SAFE_TEST_STEP_STEPS : SAFE_TEST_STEP_STEPS;
  stepper.move(distance);
}

void testSingleMotor(AccelStepper& stepper, const char* name, bool towardMax, 
                     int minPin, int maxPin, LimitState& limit) {
  Serial.println();
  Serial.print("Testing ");
  Serial.print(name);
  Serial.print(" motor toward ");
  Serial.println(towardMax ? "MAX" : "MIN");
  Serial.println("Running until limit hit, then backing off...");
  Serial.println("Press S to stop");
  
  quickTestStepper = &stepper;
  quickTestLimit = &limit;
  quickTestMinPin = minPin;
  quickTestMaxPin = maxPin;
  quickTestTowardMax = towardMax;
  quickTestBackingOff = false;
  
  // Set reduced speed for testing
  stepper.setMaxSpeed(TEST_MOTOR_SPEED);
  stepper.setAcceleration(TEST_MOTOR_ACCEL);
  
  // Clear limit flag
  limit.triggered = false;
  
  long distance = towardMax ? 999999 : -999999;
  stepper.move(distance);
  
  testMotorRunning = true;
}

void processDiagnosticCommand(char cmd) {
  cmd = toupper(cmd);
  
  // Global commands (work in all menus)
  if (cmd == 'T') {
    currentMenu = MENU_MAIN;
    diagPrintln();
    printMainMenu();
    return;
  }
  else if (cmd == 'X') {
    diagPrintln();
    diagPrintln("Exiting diagnostic mode, entering production mode...");
    diagnosticMode = false;
    diagPrintln("Production Mode - Listening for RPi4 commands");
    diagPrintln();
    return;
  }
  else if (cmd == 'S') {
    emergencyStop();
    testMotorRunning = false;
    safeLimitTestRunning = false;
    diagPrintln("EMERGENCY STOP!");
    diagPrintln();
    return;
  }
  else if (cmd == 'H') {
    diagPrintln();
    switch (currentMenu) {
      case MENU_MAIN: printMainMenu(); break;
      case MENU_MOVE: printMoveMenu(); break;
      case MENU_JOG: printJogMenu(); break;
      case MENU_TEST_MOTORS: printTestMotorsMenu(); break;
      case MENU_SAFE_LIMIT_TEST: printSafeLimitTestMenu(); break;
      case MENU_SPEED_SETTINGS: printSpeedSettingsMenu(); break;
      case MENU_CALIBRATION: printCalibrationMenu(); break;
    }
    return;
  }
  else if (cmd == '?') {
    printStatus();
    Serial.println();
    return;
  }
  else if (cmd == 'U') {
    // Hierarchical up navigation
    switch (currentMenu) {
      case MENU_MAIN:
        // Already at top
        break;
      case MENU_SAFE_LIMIT_TEST:
        // Safe limit test is under motor test menu
        currentMenu = MENU_TEST_MOTORS;
        diagPrintln();
        printTestMotorsMenu();
        break;
      default:
        // All other menus go back to main
        currentMenu = MENU_MAIN;
        diagPrintln();
        printMainMenu();
        break;
    }
    return;
  }
  
  // Menu-specific commands
  switch (currentMenu) {
    case MENU_MAIN:
      if (cmd == '1') {
        // HOME All Axes (sequential)
        diagPrintln();
        diagPrintln("Starting sequential homing (Z->Y->X->Focus)...");
        diagPrintln();
        
        homeAxis(zStepper, "Z", Z_MIN_PIN, Z_MAX_PIN, zLimit, STATE_HOMING_Z);
        if (currentState == STATE_ERROR) return;
        
        homeAxis(yStepper, "Y", Y_MIN_PIN, Y_MAX_PIN, yLimit, STATE_HOMING_Y);
        if (currentState == STATE_ERROR) return;
        
        homeAxis(xStepper, "X", X_MIN_PIN, X_MAX_PIN, xLimit, STATE_HOMING_X);
        if (currentState == STATE_ERROR) return;
        
        homeAxis(fStepper, "Focus", F_MIN_PIN, F_MAX_PIN, fLimit, STATE_HOMING_F);
        if (currentState == STATE_ERROR) return;
        
        homedThisSession = true;
        changeState(diagnosticMode ? STATE_DIAGNOSTIC : STATE_IDLE);
        diagPrintln("================================================================================");
        diagPrintln("HOMING COMPLETE - All axes at 0.00mm");
        diagPrintln("================================================================================");
        diagPrintln();
        printMainMenu();
      }
      else if (cmd == '2') {
        // FAST HOME All Axes (parallel)
        fastHomeAll();
        homedThisSession = true;
        changeState(diagnosticMode ? STATE_DIAGNOSTIC : STATE_IDLE);
      }
      else if (cmd == '3') {
        currentMenu = MENU_MOVE;
        // Initialize pending move to current position
        pendingMove.x = xStepper.currentPosition() / STEPS_PER_MM;
        pendingMove.y = yStepper.currentPosition() / STEPS_PER_MM;
        pendingMove.z = zStepper.currentPosition() / STEPS_PER_MM;
        pendingMove.f = fStepper.currentPosition() / STEPS_PER_MM;
        diagPrintln();
        printMoveMenu();
      }
      else if (cmd == '4') {
        currentMenu = MENU_JOG;
        diagPrintln();
        printJogMenu();
      }
      else if (cmd == '5') {
        Serial.println();
        printStatus();
        Serial.println();
        printMainMenu();
      }
      else if (cmd == '6') {
        // Test limit switches
        diagPrintln();
        diagPrintln("Listening for limit switches... (press T to return to menu)");
        diagPrintln();
        
        xLimit.triggered = false;
        yLimit.triggered = false;
        zLimit.triggered = false;
        fLimit.triggered = false;
        
        bool testing = true;
        while (testing) {
          // CRITICAL: Keep steppers running
          xStepper.run();
          yStepper.run();
          zStepper.run();
          fStepper.run();
          
          // Update pin states for hysteresis (reset to HIGH when released)
          if (digitalRead(X_MIN_PIN) == HIGH) xLimit.lastMinPinState = HIGH;
          if (digitalRead(X_MAX_PIN) == HIGH) xLimit.lastMaxPinState = HIGH;
          if (digitalRead(Y_MIN_PIN) == HIGH) yLimit.lastMinPinState = HIGH;
          if (digitalRead(Y_MAX_PIN) == HIGH) yLimit.lastMaxPinState = HIGH;
          if (digitalRead(Z_MIN_PIN) == HIGH) zLimit.lastMinPinState = HIGH;
          if (digitalRead(Z_MAX_PIN) == HIGH) zLimit.lastMaxPinState = HIGH;
          if (digitalRead(F_MIN_PIN) == HIGH) fLimit.lastMinPinState = HIGH;
          if (digitalRead(F_MAX_PIN) == HIGH) fLimit.lastMaxPinState = HIGH;
          
          // Check for exit command from either Serial (USB) or Serial1 (RPI4)
          if (Serial.available() > 0 || Serial1.available() > 0) {
            char c = Serial.available() ? Serial.read() : Serial1.read();
            if (c == 'T' || c == 't') {
              testing = false;
              break;
            }
          }
          
          // Check for triggers - report what ISR detected regardless of current pin state
          if (xLimit.triggered) {
            bool wasMin = xLimit.isMin;
            xLimit.triggered = false;
            char msg[64];
            snprintf(msg, sizeof(msg), "[OK] %s triggered!", wasMin ? "X_MIN" : "X_MAX");
            diagPrintln(msg);
          }
          
          if (yLimit.triggered) {
            bool wasMin = yLimit.isMin;
            yLimit.triggered = false;
            char msg[64];
            snprintf(msg, sizeof(msg), "[OK] %s triggered!", wasMin ? "Y_MIN" : "Y_MAX");
            diagPrintln(msg);
          }
          
          if (zLimit.triggered) {
            bool wasMin = zLimit.isMin;
            zLimit.triggered = false;
            char msg[64];
            snprintf(msg, sizeof(msg), "[OK] %s triggered!", wasMin ? "Z_MIN" : "Z_MAX");
            diagPrintln(msg);
          }
          
          if (fLimit.triggered) {
            bool wasMin = fLimit.isMin;
            int pinState = digitalRead(F_MIN_PIN);
            fLimit.triggered = false;
            char msg[64];
            snprintf(msg, sizeof(msg), "[OK] %s triggered! Pin 22 reads: %d", 
                     wasMin ? "F_MIN" : "F_MAX", pinState);
            diagPrintln(msg);
          }
          
          // Interrupts are always enabled now - no action needed
          
          delay(10);
        }
        
        diagPrintln();
        diagPrintln("Limit switch test complete.");
        diagPrintln();
        printMainMenu();
      }
      else if (cmd == '7') {
        currentMenu = MENU_TEST_MOTORS;
        diagPrintln();
        printTestMotorsMenu();
      }
      else if (cmd == '8') {
        // Speed settings (placeholder)
        diagPrintln();
        diagPrintln("Speed settings menu not yet implemented.");
        diagPrintln();
        printMainMenu();
      }
      else if (cmd == '9') {
        diagPrintln();
        printSystemInfo();
      }
      break;
      
    case MENU_MOVE:
      if (cmd == '1') {
        // Command '1' with arguments (e.g., "1 50 100 10 5") is handled in DIAG_CMD parser
        // If we get here with just '1', show an error
        diagPrintln();
        diagPrintln("ERROR: Command 1 requires arguments: 1 X Y Z F");
        diagPrintln("Example: 1 50 100 10 5");
        diagPrintln();
        printMoveMenu();
      }
      else if (cmd == '2') {
        // Execute coordinated move to target
        diagPrintln();
        
        // Check soft limits
        bool limitsOk = true;
        char buffer[128];
        
        if (pendingMove.x < 0 || pendingMove.x > X_MAX_TRAVEL_MM) {
          snprintf(buffer, sizeof(buffer), "ERROR: X=%.2fmm exceeds limits (0 to %.1fmm)",
                   pendingMove.x, X_MAX_TRAVEL_MM);
          diagPrintln(buffer);
          limitsOk = false;
        }
        if (pendingMove.y < 0 || pendingMove.y > Y_MAX_TRAVEL_MM) {
          snprintf(buffer, sizeof(buffer), "ERROR: Y=%.2fmm exceeds limits (0 to %.1fmm)",
                   pendingMove.y, Y_MAX_TRAVEL_MM);
          diagPrintln(buffer);
          limitsOk = false;
        }
        if (pendingMove.z < 0 || pendingMove.z > Z_MAX_TRAVEL_MM) {
          snprintf(buffer, sizeof(buffer), "ERROR: Z=%.2fmm exceeds limits (0 to %.1fmm)",
                   pendingMove.z, Z_MAX_TRAVEL_MM);
          diagPrintln(buffer);
          limitsOk = false;
        }
        // Note: F axis has no soft limit enforcement
        
        if (!limitsOk) {
          diagPrintln();
          diagPrintln("Move cancelled - position out of bounds");
          diagPrintln();
          printMoveMenu();
        } else {
          long xTarget = (long)(pendingMove.x * STEPS_PER_MM);
          long yTarget = (long)(pendingMove.y * STEPS_PER_MM);
          long zTarget = (long)(pendingMove.z * STEPS_PER_MM);
          long fTarget = (long)(pendingMove.f * STEPS_PER_MM);
          
          // Check if already at target
          long xDist = xTarget - xStepper.currentPosition();
          long yDist = yTarget - yStepper.currentPosition();
          long zDist = zTarget - zStepper.currentPosition();
          long fDist = fTarget - fStepper.currentPosition();
          
          if (xDist == 0 && yDist == 0 && zDist == 0 && fDist == 0) {
            diagPrintln("Already at target position - no move needed");
            diagPrintln();
            printMoveMenu();
          } else {
            snprintf(buffer, sizeof(buffer), "Moving: X=%.2fmm Y=%.2fmm Z=%.2fmm F=%.2fmm",
                     xDist / STEPS_PER_MM, yDist / STEPS_PER_MM, 
                     zDist / STEPS_PER_MM, fDist / STEPS_PER_MM);
            diagPrintln(buffer);
            
            xStepper.moveTo(xTarget);
            yStepper.moveTo(yTarget);
            zStepper.moveTo(zTarget);
            fStepper.moveTo(fTarget);
            
            while (xStepper.distanceToGo() != 0 || yStepper.distanceToGo() != 0 ||
                   zStepper.distanceToGo() != 0 || fStepper.distanceToGo() != 0) {
              xStepper.run();
              yStepper.run();
              zStepper.run();
              fStepper.run();
            }
            
            diagPrintln("Move complete!");
            diagPrintln();
            printMoveMenu();
          }
        }
      }
      else if (cmd == '3') {
        // Reset targets to current position
        pendingMove.x = xStepper.currentPosition() / STEPS_PER_MM;
        pendingMove.y = yStepper.currentPosition() / STEPS_PER_MM;
        pendingMove.z = zStepper.currentPosition() / STEPS_PER_MM;
        pendingMove.f = fStepper.currentPosition() / STEPS_PER_MM;
        diagPrintln();
        diagPrintln("Targets reset to current position");
        diagPrintln();
        printMoveMenu();
      }
      break;
      
    case MENU_JOG:
      if (cmd >= '1' && cmd <= '4') {
        // Commands 1-4 require distance argument (handled in DIAG_CMD parser)
        diagPrintln();
        diagPrintln("ERROR: Jog commands require distance argument");
        diagPrintln("Example: 1 10.5 (jog X by +10.5mm) or 2 -5 (jog Y by -5mm)");
        diagPrintln();
        printJogMenu();
      }
      break;
      
    case MENU_TEST_MOTORS:
      if (cmd == '1') testSingleMotor(xStepper, "X", false, X_MIN_PIN, X_MAX_PIN, xLimit);
      else if (cmd == '2') testSingleMotor(xStepper, "X", true, X_MIN_PIN, X_MAX_PIN, xLimit);
      else if (cmd == '3') testSingleMotor(yStepper, "Y", false, Y_MIN_PIN, Y_MAX_PIN, yLimit);
      else if (cmd == '4') testSingleMotor(yStepper, "Y", true, Y_MIN_PIN, Y_MAX_PIN, yLimit);
      else if (cmd == '5') testSingleMotor(zStepper, "Z", false, Z_MIN_PIN, Z_MAX_PIN, zLimit);
      else if (cmd == '6') testSingleMotor(zStepper, "Z", true, Z_MIN_PIN, Z_MAX_PIN, zLimit);
      else if (cmd == '7') testSingleMotor(fStepper, "F", false, F_MIN_PIN, F_MAX_PIN, fLimit);
      else if (cmd == '8') testSingleMotor(fStepper, "F", true, F_MIN_PIN, F_MAX_PIN, fLimit);
      else if (cmd == '9') {
        currentMenu = MENU_SAFE_LIMIT_TEST;
        printSafeLimitTestMenu();
      }
      break;
    
    case MENU_SAFE_LIMIT_TEST:
      if (cmd == '1') startSafeLimitTest(xStepper, "X", X_MIN_PIN, X_MAX_PIN, xLimit, true);
      else if (cmd == '2') startSafeLimitTest(xStepper, "X", X_MIN_PIN, X_MAX_PIN, xLimit, false);
      else if (cmd == '3') startSafeLimitTest(yStepper, "Y", Y_MIN_PIN, Y_MAX_PIN, yLimit, true);
      else if (cmd == '4') startSafeLimitTest(yStepper, "Y", Y_MIN_PIN, Y_MAX_PIN, yLimit, false);
      else if (cmd == '5') startSafeLimitTest(zStepper, "Z", Z_MIN_PIN, Z_MAX_PIN, zLimit, true);
      else if (cmd == '6') startSafeLimitTest(zStepper, "Z", Z_MIN_PIN, Z_MAX_PIN, zLimit, false);
      else if (cmd == '7') startSafeLimitTest(fStepper, "F", F_MIN_PIN, F_MAX_PIN, fLimit, true);
      else if (cmd == '8') startSafeLimitTest(fStepper, "F", F_MIN_PIN, F_MAX_PIN, fLimit, false);
      break;
      
    case MENU_SPEED_SETTINGS:
      Serial.println();
      Serial.println("Speed settings menu - implementation pending");
      Serial.println("Note: Requires code modification and recompile");
      Serial.println();
      printSpeedSettingsMenu();
      break;
      
    case MENU_CALIBRATION:
      Serial.println();
      Serial.println("Calibration menu - implementation pending");
      Serial.println();
      printCalibrationMenu();
      break;
  }
}

//==============================================================================
// DIAGNOSTIC MULTI-CHARACTER COMMAND PROCESSING (USB Serial)
// Handles commands like "4 10" for jog menu
//==============================================================================
void processDiagnosticMultiCommand(const char* cmd) {
  Serial.println();  // Echo newline after user input
  
  // Handle JOG menu multi-character commands (e.g., "4 10")
  if (currentMenu == MENU_JOG && cmd[0] >= '1' && cmd[0] <= '4') {
    int axis;
    float distance;
    int parsed = sscanf(cmd, "%d %f", &axis, &distance);
    
    if (parsed == 2 && axis >= 1 && axis <= 4) {
      // Calculate target position
      float currentPos, targetPos, maxTravel;
      AccelStepper* stepper;
      const char* axisName;
      
      switch (axis) {
        case 1: // X axis
          stepper = &xStepper;
          axisName = "X";
          currentPos = xStepper.currentPosition() / STEPS_PER_MM;
          targetPos = currentPos + distance;
          maxTravel = X_MAX_TRAVEL_MM;
          break;
        case 2: // Y axis
          stepper = &yStepper;
          axisName = "Y";
          currentPos = yStepper.currentPosition() / STEPS_PER_MM;
          targetPos = currentPos + distance;
          maxTravel = Y_MAX_TRAVEL_MM;
          break;
        case 3: // Z axis
          stepper = &zStepper;
          axisName = "Z";
          currentPos = zStepper.currentPosition() / STEPS_PER_MM;
          targetPos = currentPos + distance;
          maxTravel = Z_MAX_TRAVEL_MM;
          break;
        case 4: // F axis
          stepper = &fStepper;
          axisName = "F";
          currentPos = fStepper.currentPosition() / STEPS_PER_MM;
          targetPos = currentPos + distance;
          maxTravel = F_MAX_TRAVEL_MM;
          break;
        default:
          diagPrintln("Invalid axis");
          return;
      }
      
      // Check soft limits
      if (targetPos < 0) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "ERROR: Target %.2f below minimum (0.00)", targetPos);
        diagPrintln(buffer);
        printJogMenu();
        return;
      }
      if (targetPos > maxTravel) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "ERROR: Target %.2f exceeds maximum (%.2f)", targetPos, maxTravel);
        diagPrintln(buffer);
        printJogMenu();
        return;
      }
      
      // Execute jog
      long targetSteps = (long)(targetPos * STEPS_PER_MM);
      stepper->moveTo(targetSteps);
      
      char buffer[128];
      snprintf(buffer, sizeof(buffer), "Jogging %s by %.2fmm to %.2fmm", axisName, distance, targetPos);
      diagPrintln(buffer);
      
      // Wait for move to complete (blocking for simplicity in diagnostic mode)
      while (stepper->distanceToGo() != 0) {
        stepper->run();
      }
      
      diagPrintln("Jog complete");
      diagPrintln();
      printJogMenu();
      return;
    }
  }
  
  // Handle MOVE menu multi-character commands (e.g., "1 10 20 5 0")
  if (currentMenu == MENU_MOVE && cmd[0] == '1') {
    float x, y, z, f;
    int parsed = sscanf(cmd, "1 %f %f %f %f", &x, &y, &z, &f);
    if (parsed == 4) {
      pendingMove.x = x;
      pendingMove.y = y;
      pendingMove.z = z;
      pendingMove.f = f;
      
      char buffer[128];
      snprintf(buffer, sizeof(buffer), "Target set to X=%.2f Y=%.2f Z=%.2f F=%.2f", x, y, z, f);
      diagPrintln(buffer);
      diagPrintln();
      printMoveMenu();
      return;
    }
  }
  
  // If not handled as multi-char command, try as single char
  processDiagnosticCommand(cmd[0]);
}

//==============================================================================
// PROTOCOL COMMAND PROCESSING
//==============================================================================
void processProtocolCommand(const char* message, uint8_t seq) {
  // Reset watchdog
  lastMasterMessage = millis();
  
  // Check for duplicate sequence number
  if (isDuplicateSeq(seq) && seq != 0) {
    Serial.printf("Duplicate seq %d detected - resending last ACK\n", seq);
    sendAck(seq, "DUPLICATE");
    return;
  }
  
  // Record this sequence number
  recordSeq(seq);
  
  // Parse command
  char cmd[32];
  sscanf(message, "!%s", cmd);
  
  // Convert to uppercase for comparison
  for (char* p = cmd; *p; p++) *p = toupper(*p);
  
  // ===== CONNECTION MANAGEMENT =====
  
  if (strcmp(cmd, "CONNECT") == 0) {
    // Extract master version
    char masterVer[32] = "unknown";
    const char* verPtr = strstr(message, "MASTER=");
    if (verPtr) {
      sscanf(verPtr, "MASTER=%s", masterVer);
    }
    
    masterConnected = true;
    watchdogEnabled = true;
    currentState = STATE_CONNECTED;
    
    char response[128];
    snprintf(response, sizeof(response), "SLAVE=%s PROTO=%d", FIRMWARE_VERSION, PROTOCOL_VERSION);
    sendAck(seq, response);
    
    Serial.printf("Master connected: %s\n", masterVer);
  }
  
  else if (strcmp(cmd, "DISCONNECT") == 0) {
    emergencyStop();
    disableMotors();
    masterConnected = false;
    watchdogEnabled = false;
    currentState = STATE_DISCONNECTED;
    sendAck(seq);
    Serial.println("Master disconnected");
  }
  
  else if (strcmp(cmd, "PING") == 0) {
    char response[64];
    snprintf(response, sizeof(response), "UPTIME=%lu", millis());
    sendAck(seq, response);
  }
  
  // ===== MOTION COMMANDS =====
  
  else if (strcmp(cmd, "HOME") == 0 || strcmp(cmd, "HOME_FAST") == 0) {
    bool fastHome = (strcmp(cmd, "HOME_FAST") == 0);
    
    if (currentState != STATE_IDLE && currentState != STATE_CONNECTED) {
      sendNack(seq, "ERR_INVALID_STATE", "Must be IDLE or CONNECTED to home");
      return;
    }
    
    sendAck(seq, fastHome ? "HOMING=XYZF_PARALLEL" : "HOMING=ZYXF");
    currentState = STATE_HOMING;
    
    enableMotors();
    
    if (fastHome) {
      // Parallel homing
      fastHomeAll();
      // Check if fastHomeAll encountered an error
      if (currentState == STATE_ERROR) {
        sendNack(seq, "ERR_HOMING_FAILED", "Fast homing failed");
        return;
      }
    } else {
      // Sequential homing
      homeAxis(zStepper, "Z", Z_MIN_PIN, Z_MAX_PIN, zLimit, STATE_HOMING_Z);
      if (currentState == STATE_ERROR) {
        sendNack(seq, "ERR_HOMING_FAILED", "Z axis homing failed");
        return;
      }
      
      homeAxis(yStepper, "Y", Y_MIN_PIN, Y_MAX_PIN, yLimit, STATE_HOMING_Y);
      if (currentState == STATE_ERROR) {
        sendNack(seq, "ERR_HOMING_FAILED", "Y axis homing failed");
        return;
      }
      
      homeAxis(xStepper, "X", X_MIN_PIN, X_MAX_PIN, xLimit, STATE_HOMING_X);
      if (currentState == STATE_ERROR) {
        sendNack(seq, "ERR_HOMING_FAILED", "X axis homing failed");
        return;
      }
      
      homeAxis(fStepper, "Focus", F_MIN_PIN, F_MAX_PIN, fLimit, STATE_HOMING_F);
      if (currentState == STATE_ERROR) {
        sendNack(seq, "ERR_HOMING_FAILED", "Focus axis homing failed");
        return;
      }
    }
    
    // Mark all as homed
    xHomed = yHomed = zHomed = fHomed = true;
    homedThisSession = true;
    
    currentState = STATE_IDLE;
    sendComplete(seq, "HOMED=XYZF");
  }
  
  else if (strcmp(cmd, "MOVE") == 0) {
    if (currentState != STATE_IDLE) {
      sendNack(seq, "ERR_ALREADY_MOVING", "Previous motion not complete");
      return;
    }
    
    if (!xHomed || !yHomed || !zHomed || !fHomed) {
      sendNack(seq, "ERR_NOT_HOMED", "Must home all axes before move");
      return;
    }
    
    // Parse target positions
    float targetX = xStepper.currentPosition() / STEPS_PER_MM;
    float targetY = yStepper.currentPosition() / STEPS_PER_MM;
    float targetZ = zStepper.currentPosition() / STEPS_PER_MM;
    float targetF = fStepper.currentPosition() / STEPS_PER_MM;
    
    bool hasX = false, hasY = false, hasZ = false, hasF = false;
    
    const char* ptr = message;
    while (*ptr) {
      if (*ptr == 'X' && (ptr == message + 6 || *(ptr-1) == ' ')) {
        sscanf(ptr, "X%f", &targetX);
        hasX = true;
      }
      else if (*ptr == 'Y' && (ptr == message || *(ptr-1) == ' ')) {
        sscanf(ptr, "Y%f", &targetY);
        hasY = true;
      }
      else if (*ptr == 'Z' && (ptr == message || *(ptr-1) == ' ')) {
        sscanf(ptr, "Z%f", &targetZ);
        hasZ = true;
      }
      else if (*ptr == 'F' && (ptr == message || *(ptr-1) == ' ')) {
        sscanf(ptr, "F%f", &targetF);
        hasF = true;
      }
      ptr++;
    }
    
    // Validate soft limits
    if (hasX && (targetX < 0 || targetX > X_MAX_TRAVEL_MM)) {
      char errMsg[128];
      snprintf(errMsg, sizeof(errMsg), "X=%.2f exceeds limit %.2fmm", targetX, X_MAX_TRAVEL_MM);
      sendNack(seq, "ERR_OUT_OF_BOUNDS", errMsg);
      return;
    }
    if (hasY && (targetY < 0 || targetY > Y_MAX_TRAVEL_MM)) {
      char errMsg[128];
      snprintf(errMsg, sizeof(errMsg), "Y=%.2f exceeds limit %.2fmm", targetY, Y_MAX_TRAVEL_MM);
      sendNack(seq, "ERR_OUT_OF_BOUNDS", errMsg);
      return;
    }
    if (hasZ && (targetZ < 0 || targetZ > Z_MAX_TRAVEL_MM)) {
      char errMsg[128];
      snprintf(errMsg, sizeof(errMsg), "Z=%.2f exceeds limit %.2fmm", targetZ, Z_MAX_TRAVEL_MM);
      sendNack(seq, "ERR_OUT_OF_BOUNDS", errMsg);
      return;
    }
    
    // Convert to steps and execute move
    long targetSteps[4];
    targetSteps[0] = (long)(targetX * STEPS_PER_MM);
    targetSteps[1] = (long)(targetY * STEPS_PER_MM);
    targetSteps[2] = (long)(targetZ * STEPS_PER_MM);
    targetSteps[3] = (long)(targetF * STEPS_PER_MM);
    
    xStepper.moveTo(targetSteps[0]);
    yStepper.moveTo(targetSteps[1]);
    zStepper.moveTo(targetSteps[2]);
    fStepper.moveTo(targetSteps[3]);
    
    currentState = STATE_MOVING;
    
    char ackData[128];
    snprintf(ackData, sizeof(ackData), "MOVING X%.2fmm Y%.2fmm Z%.2fmm F%.2fmm",
             hasX ? (targetX - xStepper.currentPosition()/STEPS_PER_MM) : 0,
             hasY ? (targetY - yStepper.currentPosition()/STEPS_PER_MM) : 0,
             hasZ ? (targetZ - zStepper.currentPosition()/STEPS_PER_MM) : 0,
             hasF ? (targetF - fStepper.currentPosition()/STEPS_PER_MM) : 0);
    sendAck(seq, ackData);
    
    // Wait for move to complete
    while (xStepper.distanceToGo() != 0 || yStepper.distanceToGo() != 0 ||
           zStepper.distanceToGo() != 0 || fStepper.distanceToGo() != 0) {
      xStepper.run();
      yStepper.run();
      zStepper.run();
      fStepper.run();
      
      // Check for limit hits
      if (checkLimitTriggered(&xLimit, X_MIN_PIN, X_MAX_PIN) ||
          checkLimitTriggered(&yLimit, Y_MIN_PIN, Y_MAX_PIN) ||
          checkLimitTriggered(&zLimit, Z_MIN_PIN, Z_MAX_PIN) ||
          checkLimitTriggered(&fLimit, F_MIN_PIN, F_MAX_PIN)) {
        emergencyStop();
        currentState = STATE_ERROR;
        sendNack(seq, "ERR_LIMIT_HIT", "Limit switch triggered during move");
        sendEvent("LIMIT", "Hit during move");
        return;
      }
    }
    
    currentState = STATE_IDLE;
    char completeData[128];
    snprintf(completeData, sizeof(completeData), "X=%.2f Y=%.2f Z=%.2f F=%.2f",
             xStepper.currentPosition() / STEPS_PER_MM,
             yStepper.currentPosition() / STEPS_PER_MM,
             zStepper.currentPosition() / STEPS_PER_MM,
             fStepper.currentPosition() / STEPS_PER_MM);
    sendComplete(seq, completeData);
  }
  
  else if (strcmp(cmd, "JOG") == 0) {
    if (currentState != STATE_IDLE) {
      sendNack(seq, "ERR_ALREADY_MOVING", "Previous motion not complete");
      return;
    }
    
    // Parse axis and distance: !JOG X 10.5
    char axis;
    float distance;
    if (sscanf(message + 5, "%c %f", &axis, &distance) != 2) {
      sendNack(seq, "ERR_INVALID_ARGS", "Format: !JOG <axis> <distance>");
      return;
    }
    
    axis = toupper(axis);
    long steps = (long)(distance * STEPS_PER_MM);
    
    AccelStepper* stepper = nullptr;
    float currentPos, targetPos, maxTravel;
    
    switch (axis) {
      case 'X': 
        stepper = &xStepper; 
        currentPos = xStepper.currentPosition() / STEPS_PER_MM;
        maxTravel = X_MAX_TRAVEL_MM;
        break;
      case 'Y': 
        stepper = &yStepper; 
        currentPos = yStepper.currentPosition() / STEPS_PER_MM;
        maxTravel = Y_MAX_TRAVEL_MM;
        break;
      case 'Z': 
        stepper = &zStepper; 
        currentPos = zStepper.currentPosition() / STEPS_PER_MM;
        maxTravel = Z_MAX_TRAVEL_MM;
        break;
      case 'F': 
        stepper = &fStepper; 
        currentPos = fStepper.currentPosition() / STEPS_PER_MM;
        maxTravel = F_MAX_TRAVEL_MM;
        break;
      default:
        sendNack(seq, "ERR_INVALID_ARGS", "Axis must be X, Y, Z, or F");
        return;
    }
    
    targetPos = currentPos + distance;
    
    // Check soft limits
    if (targetPos < 0) {
      char errMsg[64];
      snprintf(errMsg, sizeof(errMsg), "Target %.2f below min 0.00", targetPos);
      sendNack(seq, "ERR_OUT_OF_BOUNDS", errMsg);
      return;
    }
    if (targetPos > maxTravel) {
      char errMsg[64];
      snprintf(errMsg, sizeof(errMsg), "Target %.2f exceeds max %.2f", targetPos, maxTravel);
      sendNack(seq, "ERR_OUT_OF_BOUNDS", errMsg);
      return;
    }
    
    stepper->move(steps);
    currentState = STATE_JOGGING;
    
    char ackData[64];
    snprintf(ackData, sizeof(ackData), "JOGGING %c %+.2fmm", axis, distance);
    sendAck(seq, ackData);
    
    // Wait for jog to complete
    while (stepper->distanceToGo() != 0) {
      stepper->run();
      xStepper.run();
      yStepper.run();
      zStepper.run();
      fStepper.run();
    }
    
    currentState = STATE_IDLE;
    char completeData[128];
    snprintf(completeData, sizeof(completeData), "X=%.2f Y=%.2f Z=%.2f F=%.2f",
             xStepper.currentPosition() / STEPS_PER_MM,
             yStepper.currentPosition() / STEPS_PER_MM,
             zStepper.currentPosition() / STEPS_PER_MM,
             fStepper.currentPosition() / STEPS_PER_MM);
    sendComplete(seq, completeData);
  }
  
  else if (strcmp(cmd, "STOP") == 0) {
    emergencyStop();
    currentState = STATE_IDLE;
    sendAck(seq, "STOPPED");
  }
  
  // ===== QUERY COMMANDS =====
  
  else if (strcmp(cmd, "STATUS") == 0) {
    sendStatus(seq);
  }
  
  else if (strcmp(cmd, "VERSION") == 0) {
    char response[128];
    snprintf(response, sizeof(response), "FW=%s HW=Teensy4.1 AXES=4 LIMITS=8", FIRMWARE_VERSION);
    sendAck(seq, response);
  }
  
  else if (strcmp(cmd, "LIMITS") == 0) {
    char response[256];
    snprintf(response, sizeof(response), 
             "X_MIN=%d X_MAX=%d Y_MIN=%d Y_MAX=%d Z_MIN=%d Z_MAX=%d F_MIN=%d F_MAX=%d",
             digitalRead(X_MIN_PIN) == LOW ? 1 : 0,
             digitalRead(X_MAX_PIN) == LOW ? 1 : 0,
             digitalRead(Y_MIN_PIN) == LOW ? 1 : 0,
             digitalRead(Y_MAX_PIN) == LOW ? 1 : 0,
             digitalRead(Z_MIN_PIN) == LOW ? 1 : 0,
             digitalRead(Z_MAX_PIN) == LOW ? 1 : 0,
             digitalRead(F_MIN_PIN) == LOW ? 1 : 0,
             digitalRead(F_MAX_PIN) == LOW ? 1 : 0);
    sendAck(seq, response);
  }
  
  // ===== DIAGNOSTIC MODE COMMANDS =====
  
  else if (strcmp(cmd, "DIAG_ENTER") == 0) {
    diagnosticMode = true;
    currentState = STATE_DIAGNOSTIC;
    watchdogEnabled = false;
    sendAck(seq, "ENTERING_DIAGNOSTIC");
    Serial.println();
    const char* banner = "*** DIAGNOSTIC MODE ***";
    Serial.println(banner);
    sendDiagMessage("");
    sendDiagMessage(banner);
    printMainMenu();
  }
  
  else if (strcmp(cmd, "DIAG_EXIT") == 0) {
    if (currentState != STATE_DIAGNOSTIC) {
      sendNack(seq, "ERR_NOT_IN_DIAG", "Not in diagnostic mode");
      return;
    }
    diagnosticMode = false;
    currentState = STATE_IDLE;
    watchdogEnabled = true;
    lastMasterMessage = millis();
    sendAck(seq, "EXITING_DIAGNOSTIC");
    Serial.println();
    Serial.println("*** PROTOCOL MODE ***");
  }
  
  else if (strcmp(cmd, "DIAG_CMD") == 0) {
    if (currentState != STATE_DIAGNOSTIC) {
      sendNack(seq, "ERR_NOT_IN_DIAG", "Must be in diagnostic mode");
      return;
    }
    
    // Extract the command part after "DIAG_CMD " (10 characters including space)
    const char* cmdArgs = message + 10; // Skip "DIAG_CMD "
    
    // Check if this is a multi-value command (starts with '1' and has spaces)
    if (currentMenu == MENU_MOVE && cmdArgs[0] == '1' && strchr(cmdArgs, ' ') != NULL) {
      // Parse "1 X Y Z F" format
      float x, y, z, f;
      int parsed = sscanf(cmdArgs, "1 %f %f %f %f", &x, &y, &z, &f);
      
      if (parsed == 4) {
        pendingMove.x = x;
        pendingMove.y = y;
        pendingMove.z = z;
        pendingMove.f = f;
        
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Target set to X=%.2f Y=%.2f Z=%.2f F=%.2f", x, y, z, f);
        sendAck(seq, buffer);
        diagPrintln();
        diagPrintln(buffer);
        diagPrintln();
        printMoveMenu();
      } else {
        sendNack(seq, "ERR_INVALID_ARGS", "Expected: 1 X Y Z F (4 numbers)");
      }
    }
    // Check if this is a JOG command with distance argument
    else if (currentMenu == MENU_JOG && cmdArgs[0] >= '1' && cmdArgs[0] <= '4' && strchr(cmdArgs, ' ') != NULL) {
      // Parse "N DIST" format where N is 1-4 and DIST is jog distance (can be negative)
      int axis;
      float distance;
      int parsed = sscanf(cmdArgs, "%d %f", &axis, &distance);
      
      if (parsed == 2 && axis >= 1 && axis <= 4) {
        // Calculate target position
        float currentPos, targetPos, maxTravel;
        AccelStepper* stepper;
        const char* axisName;
        
        switch (axis) {
          case 1: // X axis
            stepper = &xStepper;
            axisName = "X";
            currentPos = xStepper.currentPosition() / STEPS_PER_MM;
            targetPos = currentPos + distance;
            maxTravel = X_MAX_TRAVEL_MM;
            break;
          case 2: // Y axis
            stepper = &yStepper;
            axisName = "Y";
            currentPos = yStepper.currentPosition() / STEPS_PER_MM;
            targetPos = currentPos + distance;
            maxTravel = Y_MAX_TRAVEL_MM;
            break;
          case 3: // Z axis
            stepper = &zStepper;
            axisName = "Z";
            currentPos = zStepper.currentPosition() / STEPS_PER_MM;
            targetPos = currentPos + distance;
            maxTravel = Z_MAX_TRAVEL_MM;
            break;
          case 4: // F axis
            stepper = &fStepper;
            axisName = "F";
            currentPos = fStepper.currentPosition() / STEPS_PER_MM;
            targetPos = currentPos + distance;
            maxTravel = 999999.0; // F axis has no soft limit
            break;
        }
        
        // Check soft limits (except for F axis)
        char buffer[128];
        if (axis != 4 && (targetPos < 0 || targetPos > maxTravel)) {
          snprintf(buffer, sizeof(buffer), "ERROR: %s target %.2fmm exceeds limits (0 to %.1fmm)",
                   axisName, targetPos, maxTravel);
          sendNack(seq, "ERR_SOFT_LIMIT", buffer);
          diagPrintln();
          diagPrintln(buffer);
          diagPrintln();
          printJogMenu();
        } else {
          // Execute jog move
          long targetSteps = (long)(targetPos * STEPS_PER_MM);
          snprintf(buffer, sizeof(buffer), "Jogging %s by %+.2fmm (%.2f -> %.2f)",
                   axisName, distance, currentPos, targetPos);
          sendAck(seq, buffer);
          diagPrintln();
          diagPrintln(buffer);
          
          stepper->moveTo(targetSteps);
          
          while (stepper->distanceToGo() != 0) {
            stepper->run();
            // Keep other steppers alive
            if (axis != 1) xStepper.run();
            if (axis != 2) yStepper.run();
            if (axis != 3) zStepper.run();
            if (axis != 4) fStepper.run();
          }
          
          lastJogDistance = abs(distance);
          diagPrintln("Jog complete!");
          diagPrintln();
          printJogMenu();
        }
      } else {
        sendNack(seq, "ERR_INVALID_ARGS", "Expected: N DIST where N=1-4 and DIST is distance in mm");
      }
    }
    else {
      // Single character command
      char diagCmd = cmdArgs[0];
      sendAck(seq, "CMD_SENT");
      processDiagnosticCommand(diagCmd);
    }
  }
  
  else if (strcmp(cmd, "DIAG_MENU") == 0) {
    if (currentState != STATE_DIAGNOSTIC) {
      sendNack(seq, "ERR_NOT_IN_DIAG", "Must be in diagnostic mode");
      return;
    }
    
    sendAck(seq, "MENU_FOLLOWS");
    printMainMenu();
  }
  
  // ===== ERROR HANDLING =====
  
  else if (strcmp(cmd, "CLEAR_ERROR") == 0) {
    if (currentState == STATE_ERROR) {
      currentState = STATE_IDLE;
      lastErrorCode[0] = '\0';
      lastErrorMessage[0] = '\0';
      sendAck(seq, "ERROR_CLEARED");
    } else {
      sendNack(seq, "ERR_INVALID_STATE", "Not in error state");
    }
  }
  
  else {
    sendNack(seq, "ERR_UNKNOWN_CMD", "Command not recognized");
  }
}

//==============================================================================
// COMMAND PROCESSING (LEGACY MODE)
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
  char buffer[128];
  
  diagPrintln("@STATUS");
  
  snprintf(buffer, sizeof(buffer), "  State: %s", stateNames[currentState]);
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  X: %.2fmm (%ld steps)",
           xStepper.currentPosition() / STEPS_PER_MM, xStepper.currentPosition());
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  Y: %.2fmm (%ld steps)",
           yStepper.currentPosition() / STEPS_PER_MM, yStepper.currentPosition());
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  Z: %.2fmm (%ld steps)",
           zStepper.currentPosition() / STEPS_PER_MM, zStepper.currentPosition());
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  Focus: %.2fmm (%ld steps)",
           fStepper.currentPosition() / STEPS_PER_MM, fStepper.currentPosition());
  diagPrintln(buffer);
  
  snprintf(buffer, sizeof(buffer), "  Limits: X_MIN=%s X_MAX=%s Y_MIN=%s Y_MAX=%s Z_MIN=%s Z_MAX=%s F_MIN=%s F_MAX=%s",
           digitalRead(X_MIN_PIN) == LOW ? "CLOSED" : "OPEN",
           digitalRead(X_MAX_PIN) == LOW ? "CLOSED" : "OPEN",
           digitalRead(Y_MIN_PIN) == LOW ? "CLOSED" : "OPEN",
           digitalRead(Y_MAX_PIN) == LOW ? "CLOSED" : "OPEN",
           digitalRead(Z_MIN_PIN) == LOW ? "CLOSED" : "OPEN",
           digitalRead(Z_MAX_PIN) == LOW ? "CLOSED" : "OPEN",
           digitalRead(F_MIN_PIN) == LOW ? "CLOSED" : "OPEN",
           digitalRead(F_MAX_PIN) == LOW ? "CLOSED" : "OPEN");
  diagPrintln(buffer);
}
