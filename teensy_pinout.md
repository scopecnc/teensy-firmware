# Teensy 4.1 Pinout - Robotic Microscope Controller

Source of truth: `src/main.cpp` (active firmware)

## UART Communication (Serial1)

| Pin | Function | Notes |
|-----|----------|-------|
| 0   | RX       | From RPi |
| 1   | TX       | To RPi |

## Common Enable

| Pin | Function | Notes |
|-----|----------|-------|
| 33  | ENA_ALL  | Active LOW - enables all 4 TB6600 drivers |

## X Axis (Pins 2-5)

| Pin | Function | Notes |
|-----|----------|-------|
| 2   | X_STEP   | |
| 3   | X_DIR    | |
| 4   | X_MIN    | Limit switch, NO, to GND |
| 5   | X_MAX    | Limit switch, NO, to GND |

## Y Axis (Pins 6-9)

| Pin | Function | Notes |
|-----|----------|-------|
| 6   | Y_STEP   | |
| 7   | Y_DIR    | |
| 8   | Y_MIN    | Limit switch, NO, to GND |
| 9   | Y_MAX    | Limit switch, NO, to GND |

## Z Axis (Pins 10-14)

| Pin | Function | Notes |
|-----|----------|-------|
| 10  | Z_STEP   | |
| 11  | Z_DIR    | |
| 12  | Z_MIN    | Limit switch, NO, to GND |
| 13  | SKIPPED  | Onboard LED |
| 14  | Z_MAX    | Limit switch, NO, to GND |

## Focus Axis (Pins 15-18, 22)

| Pin | Function | Notes |
|-----|----------|-------|
| 15  | F_STEP   | |
| 16  | F_DIR    | |
| 22  | F_MIN    | Limit switch, NO, to GND (moved from pin 17 - I2C conflict) |
| 18  | F_MAX    | Limit switch, NO, to GND |

## Other

| Pin | Function | Notes |
|-----|----------|-------|
| 13  | LED_PIN  | Onboard LED (heartbeat) |
