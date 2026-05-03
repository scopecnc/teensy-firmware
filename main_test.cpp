/*******************************************************************************
 * Teensy 4.1 Hardware Validation Test Program
 * 
 * Version: TEST-1.0
 * Date: December 2025
 * 
 * Description:
 *   Simple test program for validating wiring of 4 stepper motors and 8 limit
 *   switches. Provides interactive menu to test each component individually.
 * 
 * Usage:
 *   1. Connect single motor to desired axis connector
 *   2. Connect single limit switch to desired connector
 *   3. Use menu to validate each connection
 *   4. When done, restore main_production.cpp to main.cpp
 * 
 * Hardware:
 *   - Teensy 4.1 (i.MX RT1062, 600MHz)
 *   - 4x TB6600 stepper drivers (8 microsteps, inverted ENA logic)
 *   - Test motor: Single NEMA17 stepper
 *   - Test switch: Single NO limit switch
 ******************************************************************************/

#include <Arduino.h>
#include <AccelStepper.h>

//==============================================================================
// VERSION
//==============================================================================
const char* FIRMWARE_VERSION = "TEST-1.0 - Hardware Validation";

//==============================================================================
// PIN DEFINITIONS
//==============================================================================
const int LED_PIN = 13;
const int ENA_ALL_PIN = 33;  // Active LOW

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
const int Z_MAX_PIN = 14;

// Focus Axis
const int F_STEP_PIN = 15;
const int F_DIR_PIN = 16;
const int F_MIN_PIN = 17;
const int F_MAX_PIN = 18;

//==============================================================================
// TEST CONFIGURATION
//==============================================================================
const float TEST_SPEED = 500.0;          // steps/sec (slow for safety)
const float TEST_ACCEL = 250.0;          // steps/sec²
const unsigned int PULSE_WIDTH_US = 10;
const unsigned long MOTOR_RUN_TIME_MS = 5000;  // 5 seconds

// Debounce settings
const unsigned long DEBOUNCE_MS = 300;
const unsigned long VERIFY_DELAY_MS = 20;

//==============================================================================
// STEPPER OBJECTS
//==============================================================================
AccelStepper xStepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper yStepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper zStepper(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper fStepper(AccelStepper::DRIVER, F_STEP_PIN, F_DIR_PIN);

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
// TEST STATE
//==============================================================================
bool testRunning = false;
bool stopRequested = false;

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
// HELPER FUNCTIONS
//==============================================================================
void printMenu() {
  Serial.println();
  Serial.println("================================================================================");
  Serial.println("Teensy 4.1 Hardware Validation Test");
  Serial.println("================================================================================");
  Serial.println();
  Serial.println("LIMIT SWITCH TEST:");
  Serial.println("  1 - Test All Limit Switches (press any switch to see which one)");
  Serial.println();
  Serial.println("MOTOR TESTS:");
  Serial.println("  2 - X Motor Forward      3 - X Motor Reverse");
  Serial.println("  4 - Y Motor Forward      5 - Y Motor Reverse");
  Serial.println("  6 - Z Motor Forward      7 - Z Motor Reverse");
  Serial.println("  8 - Focus Motor Forward  9 - Focus Motor Reverse");
  Serial.println();
  Serial.println("OTHER:");
  Serial.println("  S - Show Pin Status (all switches/enable state)");
  Serial.println("  0 - Stop Current Test");
  Serial.println();
  Serial.print("Enter command: ");
}

void showStatus() {
  Serial.println();
  Serial.println("PIN STATUS:");
  Serial.print("  X_MIN: "); Serial.println(digitalRead(X_MIN_PIN) == HIGH ? "HIGH (open)" : "LOW (closed!)");
  Serial.print("  X_MAX: "); Serial.println(digitalRead(X_MAX_PIN) == HIGH ? "HIGH (open)" : "LOW (closed!)");
  Serial.print("  Y_MIN: "); Serial.println(digitalRead(Y_MIN_PIN) == HIGH ? "HIGH (open)" : "LOW (closed!)");
  Serial.print("  Y_MAX: "); Serial.println(digitalRead(Y_MAX_PIN) == HIGH ? "HIGH (open)" : "LOW (closed!)");
  Serial.print("  Z_MIN: "); Serial.println(digitalRead(Z_MIN_PIN) == HIGH ? "HIGH (open)" : "LOW (closed!)");
  Serial.print("  Z_MAX: "); Serial.println(digitalRead(Z_MAX_PIN) == HIGH ? "HIGH (open)" : "LOW (closed!)");
  Serial.print("  F_MIN: "); Serial.println(digitalRead(F_MIN_PIN) == HIGH ? "HIGH (open)" : "LOW (closed!)");
  Serial.print("  F_MAX: "); Serial.println(digitalRead(F_MAX_PIN) == HIGH ? "HIGH (open)" : "LOW (closed!)");
  Serial.print("  ENA_ALL: ");
  Serial.println(digitalRead(ENA_ALL_PIN) == LOW ? "LOW (motors enabled)" : "HIGH (motors disabled)");
  Serial.println();
}

void testLimitSwitches() {
  Serial.println();
  Serial.println("Listening for limit switches... (press 0 to stop)");
  Serial.println();
  
  testRunning = true;
  stopRequested = false;
  
  // Clear all flags
  xLimit.triggered = false;
  yLimit.triggered = false;
  zLimit.triggered = false;
  fLimit.triggered = false;
  
  while (testRunning && !stopRequested) {
    // Check for stop command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '0') {
        stopRequested = true;
        break;
      }
    }
    
    // Check X axis limits
    if (xLimit.triggered) {
      xLimit.triggered = false;
      delay(VERIFY_DELAY_MS);
      if (xLimit.isMin && digitalRead(X_MIN_PIN) == LOW) {
        Serial.println("✓ X_MIN triggered!");
      } else if (!xLimit.isMin && digitalRead(X_MAX_PIN) == LOW) {
        Serial.println("✓ X_MAX triggered!");
      }
    }
    
    // Check Y axis limits
    if (yLimit.triggered) {
      yLimit.triggered = false;
      delay(VERIFY_DELAY_MS);
      if (yLimit.isMin && digitalRead(Y_MIN_PIN) == LOW) {
        Serial.println("✓ Y_MIN triggered!");
      } else if (!yLimit.isMin && digitalRead(Y_MAX_PIN) == LOW) {
        Serial.println("✓ Y_MAX triggered!");
      }
    }
    
    // Check Z axis limits
    if (zLimit.triggered) {
      zLimit.triggered = false;
      delay(VERIFY_DELAY_MS);
      if (zLimit.isMin && digitalRead(Z_MIN_PIN) == LOW) {
        Serial.println("✓ Z_MIN triggered!");
      } else if (!zLimit.isMin && digitalRead(Z_MAX_PIN) == LOW) {
        Serial.println("✓ Z_MAX triggered!");
      }
    }
    
    // Check Focus axis limits
    if (fLimit.triggered) {
      fLimit.triggered = false;
      delay(VERIFY_DELAY_MS);
      if (fLimit.isMin && digitalRead(F_MIN_PIN) == LOW) {
        Serial.println("✓ F_MIN triggered!");
      } else if (!fLimit.isMin && digitalRead(F_MAX_PIN) == LOW) {
        Serial.println("✓ F_MAX triggered!");
      }
    }
    
    delay(10);  // Small delay to avoid busy loop
  }
  
  testRunning = false;
  Serial.println();
  Serial.println("Limit switch test stopped.");
}

void testMotor(AccelStepper& stepper, const char* name, bool forward) {
  Serial.println();
  Serial.print(name);
  Serial.print(" Motor spinning ");
  Serial.print(forward ? "FORWARD" : "REVERSE");
  Serial.println(" for 5 seconds...");
  
  testRunning = true;
  stopRequested = false;
  
  // Set direction (large distance)
  long distance = forward ? 999999 : -999999;
  stepper.move(distance);
  
  unsigned long startTime = millis();
  unsigned long lastUpdate = 0;
  
  while (testRunning && !stopRequested) {
    // Check for stop command
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '0') {
        stopRequested = true;
        break;
      }
    }
    
    // Run motor
    stepper.run();
    
    // Progress bar
    unsigned long elapsed = millis() - startTime;
    if (elapsed >= MOTOR_RUN_TIME_MS) {
      break;
    }
    
    // Update progress every 250ms
    if (millis() - lastUpdate >= 250) {
      int percent = (elapsed * 100) / MOTOR_RUN_TIME_MS;
      Serial.print("\r");
      for (int i = 0; i < percent / 5; i++) Serial.print("█");
      for (int i = percent / 5; i < 20; i++) Serial.print("░");
      Serial.print(" ");
      Serial.print(percent);
      Serial.print("%  ");
      lastUpdate = millis();
    }
  }
  
  // Stop motor
  stepper.setCurrentPosition(stepper.currentPosition());
  
  Serial.println();
  if (stopRequested) {
    Serial.println("Motor test stopped by user.");
  } else {
    Serial.println("Complete. Motor should have spun.");
  }
  
  testRunning = false;
  Serial.println();
}

void processCommand(char cmd) {
  cmd = toupper(cmd);
  
  switch (cmd) {
    case '1':
      testLimitSwitches();
      break;
      
    case '2':
      testMotor(xStepper, "X", true);
      break;
      
    case '3':
      testMotor(xStepper, "X", false);
      break;
      
    case '4':
      testMotor(yStepper, "Y", true);
      break;
      
    case '5':
      testMotor(yStepper, "Y", false);
      break;
      
    case '6':
      testMotor(zStepper, "Z", true);
      break;
      
    case '7':
      testMotor(zStepper, "Z", false);
      break;
      
    case '8':
      testMotor(fStepper, "Focus", true);
      break;
      
    case '9':
      testMotor(fStepper, "Focus", false);
      break;
      
    case 'S':
      showStatus();
      break;
      
    case '0':
      if (testRunning) {
        stopRequested = true;
        Serial.println("Stopping test...");
      }
      break;
      
    default:
      Serial.println("Unknown command");
      break;
  }
  
  if (!testRunning) {
    printMenu();
  }
}

//==============================================================================
// SETUP
//==============================================================================
void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(2000);
  
  // Print banner
  Serial.println();
  Serial.println("================================================================================");
  Serial.print("Teensy 4.1 Firmware: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("================================================================================");
  Serial.println();
  Serial.println("This is a HARDWARE VALIDATION test program.");
  Serial.println("Connect your single test motor and switch to validate each connector.");
  Serial.println();
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize enable pin (active LOW)
  pinMode(ENA_ALL_PIN, OUTPUT);
  digitalWrite(ENA_ALL_PIN, LOW);  // Enable motors
  
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
  xStepper.setMaxSpeed(TEST_SPEED);
  xStepper.setAcceleration(TEST_ACCEL);
  xStepper.setMinPulseWidth(PULSE_WIDTH_US);
  
  yStepper.setMaxSpeed(TEST_SPEED);
  yStepper.setAcceleration(TEST_ACCEL);
  yStepper.setMinPulseWidth(PULSE_WIDTH_US);
  
  zStepper.setMaxSpeed(TEST_SPEED);
  zStepper.setAcceleration(TEST_ACCEL);
  zStepper.setMinPulseWidth(PULSE_WIDTH_US);
  
  fStepper.setMaxSpeed(TEST_SPEED);
  fStepper.setAcceleration(TEST_ACCEL);
  fStepper.setMinPulseWidth(PULSE_WIDTH_US);
  
  Serial.println("Test Configuration:");
  Serial.print("  - Motor Speed: ");
  Serial.print(TEST_SPEED);
  Serial.println(" steps/sec");
  Serial.print("  - Motor Run Time: ");
  Serial.print(MOTOR_RUN_TIME_MS / 1000);
  Serial.println(" seconds");
  Serial.print("  - Pulse Width: ");
  Serial.print(PULSE_WIDTH_US);
  Serial.println("µs");
  
  Serial.println();
  Serial.println("READY");
  
  printMenu();
}

//==============================================================================
// MAIN LOOP
//==============================================================================
void loop() {
  // Always run steppers (in case motor test is active)
  xStepper.run();
  yStepper.run();
  zStepper.run();
  fStepper.run();
  
  // Blink LED
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (millis() - lastBlink >= 500) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = millis();
  }
  
  // Process serial commands
  if (Serial.available() > 0 && !testRunning) {
    char cmd = Serial.read();
    
    // Echo command
    Serial.println(cmd);
    
    // Ignore newlines/carriage returns
    if (cmd != '\n' && cmd != '\r') {
      processCommand(cmd);
    }
  }
}
