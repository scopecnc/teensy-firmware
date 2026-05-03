# Teensy 4.1 Hardware Validation Test Program

**Version:** TEST-1.0  
**Purpose:** Systematic validation of stepper motor and limit switch wiring  
**Status:** Production-ready diagnostic tool

---

## Overview

This test program provides a simple interactive menu for validating the electrical connections between the Teensy 4.1 and the CNC hardware (TB6600 drivers, NEMA17 motors, and limit switches). Use this tool whenever you need to:

- Verify new wiring installations
- Troubleshoot connectivity issues
- Confirm motor direction and operation
- Test limit switch functionality
- Debug hardware problems before running production firmware

---

## Quick Start

### 1. Upload Test Program

The test program is located in `src/main.cpp`. To upload:

```bash
platformio run --target upload
```

Or use the PlatformIO interface in VS Code.

### 2. Connect Serial Monitor

Open the serial monitor at **115200 baud**. You should see:

```
================================================================================
Teensy 4.1 Hardware Validation Test
================================================================================

LIMIT SWITCH TEST:
  1 - Test All Limit Switches (press any switch to see which one)

MOTOR TESTS:
  2 - X Motor Forward      3 - X Motor Reverse
  4 - Y Motor Forward      5 - Y Motor Reverse
  6 - Z Motor Forward      7 - Z Motor Reverse
  8 - Focus Motor Forward  9 - Focus Motor Reverse

OTHER:
  S - Show Pin Status (all switches/enable state)
  0 - Stop Current Test
```

### 3. Run Tests

Simply type a menu number or letter and press Enter.

---

## Test Modes

### Option 1: Limit Switch Test

**What it does:** Monitors all 8 limit switches simultaneously using hardware interrupts. When any switch is pressed, it immediately prints which one triggered.

**How to use:**
1. Type `1` and press Enter
2. You'll see: `Listening for limit switches... (press 0 to stop)`
3. Press each limit switch one at a time
4. Verify the correct name appears: `✓ X_MIN triggered!`
5. Type `0` to exit the test

**Expected behavior:**
- Switch pressed → Pin goes from HIGH (3.3V) to LOW (0V)
- ISR triggers on FALLING edge
- Message appears within milliseconds
- Debounce prevents multiple triggers (300ms cooldown)

**Troubleshooting:**
- **No response:** Check wiring, verify switch is Normally Open (NO)
- **Wrong name:** Switch connected to wrong pin, check connector labels
- **Multiple triggers:** Mechanical bounce (should be filtered by 300ms debounce)

---

### Options 2-9: Motor Tests

**What they do:** Spin a single motor in the specified direction for 5 seconds at a safe test speed (500 steps/sec).

**Test options:**
- `2` - X axis motor, forward direction
- `3` - X axis motor, reverse direction
- `4` - Y axis motor, forward direction
- `5` - Y axis motor, reverse direction
- `6` - Z axis motor, forward direction
- `7` - Z axis motor, reverse direction
- `8` - Focus motor, forward direction
- `9` - Focus motor, reverse direction

**How to use:**
1. Type a number (2-9) and press Enter
2. Motor will spin for 5 seconds with progress bar:
   ```
   X Motor spinning FORWARD for 5 seconds...
   ████████████████████ 100%
   Complete. Motor should have spun.
   ```
3. Observe motor rotation and direction
4. Press `0` during test to stop early if needed

**What to verify:**
- ✅ Motor spins smoothly (no stuttering or stalling)
- ✅ Direction matches label (forward vs reverse)
- ✅ Motor is enabled (shaft should resist manual rotation)
- ✅ No unusual noise or vibration
- ✅ TB6600 driver not overheating

**Troubleshooting:**
- **No motion:** Check ENA_ALL (pin 33), verify TB6600 power, check wiring
- **Stuttering:** Check STEP/DIR connections, verify 10µs pulse width
- **Wrong direction:** DIR pin reversed, or need to swap in production code
- **Motor freewheels:** ENA logic may be inverted (should be active LOW)

---

### Option S: Show Pin Status

**What it does:** Displays the current digital state of all pins (one-time snapshot).

**How to use:**
1. Type `S` and press Enter
2. Review output:
   ```
   Current Pin Status:
     X_MIN: HIGH (open)
     X_MAX: HIGH (open)
     Y_MIN: LOW (closed!)
     ...
     ENA_ALL: LOW (motors enabled)
   ```

**Expected values:**
- **Limit switches (unpressed):** HIGH (3.3V) due to INPUT_PULLUP
- **Limit switches (pressed):** LOW (0V) when grounded
- **ENA_ALL:** LOW (motors enabled) — TB6600 uses active-LOW enable

**Use cases:**
- Verify switch wiring before running tests
- Check if switch is stuck closed (reads LOW when not pressed)
- Confirm enable signal is correct (should be LOW for operation)
- Debug intermittent connection issues

---

### Option 0: Stop Current Test

Immediately stops any running test (motor spin or limit switch monitoring).

---

## Hardware Configuration

### Pin Assignments

| Axis | Step | Dir | Min Limit | Max Limit |
|------|------|-----|-----------|-----------|
| X    | 2    | 3   | 4         | 5         |
| Y    | 6    | 7   | 8         | 9         |
| Z    | 10   | 11  | 12        | 14        |
| Focus| 15   | 16  | 17        | 18        |

**Common Signals:**
- ENA_ALL: Pin 33 (active LOW, enables all TB6600 drivers)
- LED: Pin 13 (onboard LED, blinks during operation)

### Test Parameters

- **Motor Speed:** 500 steps/sec (conservative for safety)
- **Acceleration:** 250 steps/sec²
- **Pulse Width:** 10µs (TB6600 minimum is 2.5µs)
- **Debounce:** 300ms between limit switch triggers
- **Verification Delay:** 20ms (confirms switch still pressed)
- **Motor Run Time:** 5 seconds per test

---

## Typical Validation Workflow

### New Installation

1. **Visual inspection:** Verify all connectors seated properly
2. **Power check:** Confirm 24V to TB6600 drivers (motors off)
3. **Upload test program**
4. **Run Pin Status (`S`):** Check all switches read HIGH when unpressed
5. **Test all limit switches (`1`):** Press each switch, verify correct label
6. **Test each motor (2-9):** Verify smooth operation and direction
7. **Document results:** Note any swapped connections or direction reversals
8. **Fix wiring issues:** Re-route or re-label as needed
9. **Re-test to confirm:** Repeat until all tests pass
10. **Upload production firmware:** Switch to `main_production.cpp`

### Troubleshooting Existing Setup

1. **Run Pin Status (`S`):** Look for stuck switches (LOW when unpressed)
2. **Test suspect motor:** Use options 2-9 to isolate problem axis
3. **Test suspect limit:** Use option 1, focus on problematic switch
4. **Check with logic analyzer:** Verify signal integrity at Teensy pins
5. **Swap connectors if needed:** Update wiring documentation

---

## Switching Between Test and Production Firmware

### To Upload Production Firmware

```bash
# Move test program out of src/
mv src/main.cpp src/main_test.cpp

# Move production firmware into src/
mv main_production.cpp src/main.cpp

# Upload
platformio run --target upload
```

### To Upload Test Firmware

```bash
# Move production firmware out of src/
mv src/main.cpp main_production.cpp

# Move test program into src/
mv src/main_test.cpp src/main.cpp

# Upload
platformio run --target upload
```

**Important:** Only ONE `main.cpp` should exist in the `src/` folder at a time.

---

## Future Enhancements

This test program can be expanded with:

- **UART communication test:** Send/receive test messages to/from RPi4
- **Continuous motor test:** Run for extended period to check thermal issues
- **Speed ramping test:** Test motors at various speeds to find optimal rates
- **Simultaneous multi-axis test:** Move all axes at once (stress test)
- **Encoder feedback test:** If encoders added in future
- **Emergency stop test:** Verify limit switches halt motion immediately

To add new tests, follow the existing menu structure in `main.cpp`.

---

## Technical Notes

### Interrupt Architecture

- Uses hardware interrupts on FALLING edge for all 8 limit switches
- ISRs set flags (`xLimit.triggered`, etc.) checked in main loop
- Debounce logic in ISR prevents multiple triggers within 300ms
- Verification delay (20ms) confirms switch still pressed after ISR
- Non-blocking architecture allows `0` key to stop tests immediately

### TB6600 Driver Specifics

- **Enable Logic:** Active LOW (ENA=LOW enables motor, ENA=HIGH disables)
- **Signal Levels:** 3.3V from Teensy works correctly (tested)
- **Pulse Width:** 10µs exceeds 2.5µs minimum requirement
- **Microstepping:** Configured for 8 microsteps (1600 steps/rev)

### AccelStepper Library

- Version 1.64.0 (specified in `platformio.ini`)
- Uses DRIVER mode (separate STEP/DIR pins)
- `setMinPulseWidth(10)` ensures TB6600 compatibility
- `run()` called continuously in main loop for smooth stepping

---

## Troubleshooting Guide

### No Serial Output

- Check USB cable connection
- Verify serial monitor set to **115200 baud**
- Press Teensy reset button
- Try different USB port

### Motors Don't Spin

1. Check `S` status - ENA_ALL should be LOW
2. Verify 24V power to TB6600 drivers
3. Check STEP/DIR connections with logic analyzer
4. Confirm motor coils connected (A+/A-, B+/B-)
5. Try different motor (isolate driver vs motor issue)

### Limit Switches Don't Trigger

1. Use `S` to check pin state (should be HIGH when open)
2. Test with logic analyzer (should see 3.3V → 0V transition)
3. Verify switch is Normally Open (NO) type
4. Check pull-up resistor (internal INPUT_PULLUP enabled)
5. Try different switch to isolate wiring vs switch issue

### Wrong Motor Moves

- X/Y/Z/Focus connectors swapped at Teensy or driver
- Re-label connectors or swap in hardware

### Motor Spins Wrong Direction

- DIR pin may be inverted for that axis
- Note for production firmware configuration
- Or physically swap motor connector wires (swap one coil pair)

---

## File Locations

- **Test Program:** `src/main.cpp` (when active)
- **Production Firmware:** `main_production.cpp` (backed up to root)
- **This Guide:** `TEST_PROGRAM_GUIDE.md`
- **Project Documentation:** `project_context.md`
- **PlatformIO Config:** `platformio.ini`

---

## Contact & Support

For hardware-level debugging, the test program provides:
- Real-time pin state monitoring
- Interrupt-driven switch detection
- Controlled motor movements
- Non-blocking architecture for easy stop

Use this tool whenever you need confidence in the electrical connections before running production firmware.

---

**Last Updated:** December 22, 2025  
**Validated On:** Teensy 4.1 with 4x TB6600 drivers, 4x NEMA17 motors, 8x NO limit switches
