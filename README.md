# Teensy 4.1 Firmware - Robotic Microscope Controller

## Phase 1: Hello World & Serial Communication

This is the initial validation phase to ensure the development environment works.

### Hardware Required
- Teensy 4.1
- USB cable

### What This Does
- Blinks onboard LED at 1Hz
- Sends heartbeat messages every second over USB Serial
- Responds to simple commands: `PING` and `STATUS`

### How to Use

1. **Open Project**: Double-click `teensy_firmware.code-workspace` in the parent folder
2. **Build**: Click the checkmark (✓) in the bottom toolbar OR press `Ctrl+Alt+B`
3. **Upload**: Click the arrow (→) in the bottom toolbar OR press `Ctrl+Alt+U`
4. **Monitor**: Click the plug icon in the bottom toolbar OR press `Ctrl+Alt+S`

### Testing Commands

In the Serial Monitor (115200 baud), type:
- `PING` - Should respond with `@OK PONG`
- `STATUS` - Should respond with uptime and LED state
- Any other text will echo back with an error

### Next Steps (Phase 2)
- Add TB6600 stepper driver
- Single axis motion control
- Add limit switch handling
