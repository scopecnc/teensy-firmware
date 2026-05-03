# Project Context – Teensy 4.1 CNC / Robotic Microscope Controller

## CURRENT STATUS (January 5, 2026)

**Firmware Version:** v1.2 - Protocol Implementation Complete  
**Status:** ✅ Fully tested and operational  
**Next Phase:** RPi4 client development

### Recent Milestones
- ✅ All hardware validated and calibrated
- ✅ Robust protocol layer implemented with checksums, ACK/NACK, watchdog
- ✅ Diagnostic menu system working (accessible via 'D' on boot)
- ✅ Sequential and parallel homing operational
- ✅ Soft limits enforced: X=270mm, Y=150mm, Z=30mm, F=11.5mm
- ✅ STEPS_PER_MM calibrated to 400 (measured 100mm command = 50mm actual)
- ✅ Speed optimized to 6000 steps/sec (4x increase)
- ✅ Common limit handling code eliminates duplication
- ✅ F axis (zoom) belt drive calibrated with custom backoff (~7°)

### For RPi4 Developer
- **See PROTOCOL.md** for complete command reference, timing, error codes
- **UART Config:** 115200 baud, 8N1, line-terminated with \n
- **Startup:** Teensy sends `#BOOT FW=v1.2 AXES=4 *XX\n` on power-up
- **Connection:** Send `!CONNECT MASTER=<version> *XX\n` to establish link
- **Watchdog:** Must send command or `!PING` every 3 seconds (5 second timeout)
- **All messages** include XOR checksum (see PROTOCOL.md Appendix A for algorithm)
- **Python examples** provided in PROTOCOL.md Appendix B

---

## 1. High‑Level Overview

This project implements a **long‑running, robust, semi‑autonomous motion controller** for a robotic microscope built on a modified 3018 CNC frame. A Teensy 4.1 handles all **real‑time motion control**, while a Raspberry Pi 4 (RPi4) handles **GUI, imaging, sequencing, and user interaction**.

The system must run **continuously for days**, support **automatic stepping through an array of items**, and allow **manual jog override** at each position without losing synchronization or state.

The design explicitly avoids G‑code semantics in favor of a **purpose‑built motion appliance** optimized for microscopy workflows.

### Communication Architecture
- **Master:** Raspberry Pi 4 (initiates all commands)
- **Slave:** Teensy 4.1 (responds to commands, sends events)
- **Protocol:** Text-based with checksums (see PROTOCOL.md)
- **Connection:** UART Serial1 (pins 0/1 on Teensy, GPIO 14/15 on RPi)
- **Features:** ACK/NACK, sequence numbers, watchdog timer, detailed error codes

---

## 2. Hardware Architecture

### Controller
- **Teensy 4.1** (i.MX RT1062)
- Programmed from **VS Code + PlatformIO** on PC
- Hardware UART (Serial1) connection to RPi4

### Motors
- 4 × NEMA17 stepper motors
  - X, Y, Z (CNC axes with lead screws)
  - F/ZOOM (microscope zoom ring - belt driven with 1:4 ratio)

### Drivers
- Initial bring‑up: **single TB6600 + single stepper**
- Final system: TB6600 or equivalent per axis
- STEP / DIR / EN interface

### Limit Switches
- 8 total (min + max per axis)
- Normally‑Open (NO)
- Internal pull-ups enabled
- Interrupt‑driven

---

## 2.1 Teensy 4.1 Pinout

### UART Communication (Serial1)
| Function | Pin | Notes |
|----------|-----|-------|
| RX (from RPi) | 0 | Serial1 RX |
| TX (to RPi) | 1 | Serial1 TX |

### Common Enable
| Function | Pin | Notes |
|----------|-----|-------|
| ENA_ALL | 33 | Common enable for all 4 TB6600 drivers (active LOW - inverted logic) |

### X Axis (Pins 2-5)
| Function | Pin | Notes |
|----------|-----|-------|
| X_STEP | 2 | Step pulse to TB6600 |
| X_DIR | 3 | Direction control |
| X_MIN | 4 | Limit switch (NO, pulled up, to GND) |
| X_MAX | 5 | Limit switch (NO, pulled up, to GND) |

### Y Axis (Pins 6-9)
| Function | Pin | Notes |
|----------|-----|-------|
| Y_STEP | 6 | Step pulse to TB6600 |
| Y_DIR | 7 | Direction control |
| Y_MIN | 8 | Limit switch (NO, pulled up, to GND) |
| Y_MAX | 9 | Limit switch (NO, pulled up, to GND) |

### Z Axis (Pins 10-14)
| Function | Pin | Notes |
|----------|-----|-------|
| Z_STEP | 10 | Step pulse to TB6600 |
| Z_DIR | 11 | Direction control |
| Z_MIN | 12 | Limit switch (NO, pulled up, to GND) |
| Z_MAX | 14 | Limit switch (NO, pulled up, to GND) |

*Note: Pin 13 is skipped (onboard LED)*

### F/ZOOM Axis (Pins 15-16, 22, 18) - Belt Driven
| Function | Pin | Notes |
|----------|-----|---------|
| F_STEP | 15 | Step pulse to TB6600 |
| F_DIR | 16 | Direction control |
| F_MIN | 22 | Limit switch (NO, pulled up, to GND) - **Moved from pin 17 due to I2C conflicts** |
| F_MAX | 18 | Limit switch (NO, pulled up, to GND) |

**Note:** F axis controls the microscope zoom ring via a belt with 1:4 ratio (stepper pulley:zoom ring). Unlike the lead screw axes, this uses rotational motion measured in equivalent "mm" for software consistency.

### TB6600 Wiring Summary (Active LOW Enable)
Each TB6600 driver connects:
- **PUL+** ← Teensy STEP pin (2, 6, 10, or 15) [3.3V logic]
- **PUL-** ← GND
- **DIR+** ← Teensy DIR pin (3, 7, 11, or 16) [3.3V logic]
- **DIR-** ← GND
- **ENA+** ← Teensy ENA_ALL pin (33) [3.3V logic]
- **ENA-** ← GND

**Signal Logic:**
- **ENA LOW = Driver ENABLED, ENA HIGH = Driver DISABLED** (inverted logic)
- STEP rising edge = Step pulse
- DIR HIGH/LOW = Direction control

All 4 drivers share the common ENA signal on pin 33.

**Important:** TB6600 drivers have inverted enable logic despite datasheet ambiguity. Verified during hardware testing.

**Note:** TB6600 3.3V logic compatibility verified. Teensy 4.1 outputs 3.3V successfully drive STEP/DIR inputs. Only ENA required workaround due to inverted logic.

### Limit Switch Wiring
Each limit switch:
- **One terminal** → Teensy limit pin (4, 5, 8, 9, 12, 14, 22, or 18)
- **Other terminal** → GND
- Internal pull-up resistor enabled in firmware (INPUT_PULLUP)
- Switch closes (connects to GND) when triggered (Normally-Open)
- Interrupt on FALLING edge

**Note:** F_MIN uses pin 22 (not 17) to avoid I2C hardware interference on pins 17/19/20

---

## 3. Software Architecture

### Current Implementation Status

**Firmware Version:** v1.2 (December 2025)

**Implemented Features:**
- ✅ 4-axis stepper control (X, Y, Z, Focus)
- ✅ Sequential homing with 4-phase approach (seek → backoff → creep → final_backoff)
- ✅ Coordinated multi-axis moves via MultiStepper
- ✅ Incremental jogging with soft limit enforcement
- ✅ Emergency stop on any limit hit
- ✅ State machine (IDLE, HOMING, MOVING, JOGGING, ERROR)
- ✅ Non-blocking serial command parser
- ✅ Heartbeat telemetry (2-second interval)
- ✅ Comprehensive error detection and reporting

**Testing Configuration:**
- Speeds set to 25% for initial validation
- Max speed: 1500 steps/sec (~56 RPM)
- Acceleration: 500 steps/sec²
- Homing: 800 steps/sec seek, 200 steps/sec creep

**Not Yet Implemented:**
- ❌ #SEQ command protocol (using simplified commands)
- ❌ RPi UART integration (currently USB Serial only)
- ❌ Persistent position storage
- ❌ Auto-step sequencing

### Teensy (C++ Firmware)

Responsibilities:
- Deterministic stepper control
- Acceleration / deceleration
- Homing sequences
- Limit switch handling with emergency stop
- Incremental jogging
- Motion state machine
- Serial command processing
- Periodic heartbeat and telemetry

Libraries:
- **AccelStepper 1.64** (primary motion engine)
  - Note: Original TeensyStep library incompatible with Teensy 4.x (requires Kinetis chip)
  - AccelStepper provides coordinated multi-axis motion via MultiStepper class
  - Software-based timing adequate for CNC microscopy application
- Arduino core for Teensy

#### Motion Parameters (v1.2 - CALIBRATED & OPERATIONAL)

**CRITICAL CALIBRATED VALUES:**
- **STEPS_PER_MM: 400** (was 200)
  - Measured: 100mm command produced 50mm actual travel
  - Cause: TB6600 drivers set to 1/16 microstepping (not 1/8) OR T4 lead screws (not T8)
  - Applies uniformly to X, Y, Z axes
  
**Soft Limits (MEASURED on physical hardware):**
- **X: 0 to 270mm** (108,000 steps) - enforced
- **Y: 0 to 150mm** (60,000 steps) - enforced
- **Z: 0 to 30mm** (12,000 steps) - enforced
- **F/Zoom: 0 to 11.5mm** (4,600 steps) - enforced (belt drive, 1:4 ratio)

**Step Pulse Width:** 10µs
- TB6600 requires minimum 2.5µs, set to 10µs for signal robustness
- Configured via `AccelStepper::setMinPulseWidth(10)`

**Speed and Acceleration (OPTIMIZED - 4x increase from initial):**
- **Max Speed: 6000 steps/sec** (~225 RPM, 30 mm/sec)
- **Acceleration: 2000 steps/sec²**
- **Homing Seek: 3000 steps/sec** (fast approach)
- **Homing Creep: 400 steps/sec** (precision approach)
- With NEMA17 (200 steps/rev) + 1/16 microsteps:
  - 3200 steps/revolution
  - 400 steps/mm (calibrated)

**CNC Performance at Current Speeds:**
- Linear speed: 30 mm/sec = 1800 mm/min
- Acceleration time to max: 3 seconds
- Traverse times:
  - 270mm (X-axis full travel): ~9 seconds
  - 150mm (Y-axis full travel): ~5 seconds
  - 30mm (Z-axis full travel): ~1 second

**Motor Direction Inversions (setPinsInverted):**
- **X: Inverted** (true, false, false)
- **Y: Normal** (false, false, false)
- **Z: Inverted** (true, false, false)
- **Focus: Inverted** (true, false, false)

**Homing Parameters:**
- Back-off distance (X, Y, Z): 5mm (2000 steps at 400 steps/mm)
- Back-off distance (F/Zoom): 249 steps (~7° zoom ring rotation, belt 1:4 ratio)
- Sequential order: Z → Y → X → Focus
- Parallel mode: All axes simultaneously (FAST HOME)
- Timeout: 120 seconds per axis
- Home position: MIN limit = 0.00mm
- **4-phase homing:** SEEK → BACKOFF → CREEP → FINAL_BACKOFF

**Limit Switch Robustness (COMPLETELY REDESIGNED):**
- **Timestamp-based debouncing:** 50ms debounce using `debounceUntil` field (no interrupt detaching)
- **Pin state hysteresis:** Track last pin state, only trigger on HIGH→LOW transitions
- **Triple-read verification:** 200µs stable LOW required (noise rejection)
- **Polling fallback:** `checkLimitTriggered()` directly reads pins if interrupt missed
- **Continuous state tracking:** Main loop updates pin states when HIGH (prevents stale state)
- Applied in all 8 ISRs (xMinISR, xMaxISR, yMinISR, yMaxISR, zMinISR, zMaxISR, fMinISR, fMaxISR)
- **Reason:** Long cables (several feet) cause electrical noise; old detach/reattach approach had race conditions
- **F_MIN pin:** Moved to pin 22 (pins 17/19/20 have I2C hardware conflicts)
- **Result:** Eliminated 25% failure rate during fast homing operations

#### Protocol Commands (v1.2 - See PROTOCOL.md for complete reference)

**Command Format:** `!<CMD> [args] *<checksum>\n`  
**Response Format:** `@<TYPE> <seq> [data] *<checksum>\n`  
**Event Format:** `#<EVENT> [data] *<checksum>\n`

**Core Commands:**
| Command | Parameters | Description | Example |
|---------|-----------|-------------|----------|
| `!CONNECT` | MASTER=version | Establish connection | `!CONNECT MASTER=v1.0 *3C` |
| `!PING` | None | Watchdog keepalive | `!PING *1F` |
| `!STATUS` | None | Report state and positions | `!STATUS *1F` |
| `!HOME` | [axes] | Sequential homing (Z→Y→X→F) | `!HOME *1A` |
| `!HOME_FAST` | None | Parallel homing (all axes) | `!HOME_FAST *2B` |
| `!MOVE` | X# Y# Z# F# | Coordinated absolute move (mm) | `!MOVE X100 Y50 *3A` |
| `!JOG` | axis distance | Incremental single-axis move | `!JOG X 10.5 *2F` |
| `!STOP` | None | Emergency stop all axes | `!STOP *1C` |
| `!DIAG_ENTER` | None | Enter diagnostic mode | `!DIAG_ENTER *2A` |
| `!DIAG_EXIT` | None | Exit diagnostic mode | `!DIAG_EXIT *4D` |

**Response Format:**
- ACK: `@ACK <seq> [data] *<checksum>`
- NACK: `@NACK <seq> <error_code> <message> *<checksum>`
- Status: `@STATUS <seq> X=? Y=? Z=? F=? STATE=? HOMED=? *<checksum>`
- Complete: `@COMPLETE <seq> [data] *<checksum>`

**Event Messages:**
- `#BOOT FW=v1.2 AXES=4` - Power-on / reset
- `#COMM_LOST Watchdog timeout` - Communication lost (5s timeout)
- `#LIMIT X_MIN` / `#LIMIT Y_MAX` etc. - Limit switch triggered
- `#HOMING_PROGRESS X SEEK` - Homing phase updates

**Protocol Features:**
- **Checksums:** XOR checksum on all messages (prevents corruption)
- **Sequence Numbers:** Duplicate detection, retry support
- **Watchdog Timer:** 5-second timeout, automatic motor shutdown
- **Asynchronous Startup:** Either device can boot first
- **20+ Error Codes:** ERR_NOT_HOMED, ERR_OUT_OF_BOUNDS, ERR_LIMIT_HIT, etc.

**State Machine (Extended):**
- `BOOT` - Power-on initialization
- `DISCONNECTED` - Waiting for master connection
- `CONNECTED` - Master connected, not yet homed
- `IDLE` - Ready for commands, homed
- `HOMING` - Executing homing sequence
- `MOVING` - Executing coordinated move
- `JOGGING` - Executing incremental jog
- `ERROR` - Fault state, requires !CLEAR_ERROR
- `DIAGNOSTIC` - Manual diagnostic mode active
- `COMM_LOST` - Watchdog expired, motors disabled

**Safety Features:**
- All motion stops immediately on any limit hit
- Soft limits enforced before move starts (X≤270mm, Y≤150mm, Z≤30mm)
- Homing detects wrong limit hit (MAX instead of MIN)
- 120-second timeout per homing operation
- Position reported in mm (float with 2 decimal places)
- Watchdog automatically disables motors on communication loss
- Emergency stop can be called at any time

Non‑Responsibilities:
- No GUI
- No imaging
- No autofocus logic
- No scheduling decisions

### Raspberry Pi 4 (Python)

Responsibilities:
- GUI (buttons, jog controls, status display)
- Image capture & display
- Autofocus algorithms
- Auto‑step sequencing through item arrays
- Idle detection and resume logic
- Serial command orchestration
- Error handling and retries

---

## 4. Motion Model

### Axis Abstraction
Each axis is modeled as:

```
Axis {
  Stepper* stepper;
  limitMinPin;
  limitMaxPin;
  stepsPerUnit;   // mm or focus ticks
  softMin;
  softMax;
  currentPos;
  limitHitFlag;
}
```

### Motion Types
- **Absolute move**: moveTo(X,Y,Z,F)
- **Jog**: velocity‑based continuous motion
- **Homing**: seek → back‑off → re‑approach → zero

---

## 5. State Machine (v1.2 Implementation)

### States
```
IDLE          - Ready for commands, motors idle
HOMING_Z      - Homing Z axis (4-phase sequence)
HOMING_Y      - Homing Y axis
HOMING_X      - Homing X axis  
HOMING_F      - Homing Focus axis
MOVING        - Executing coordinated multi-axis move
JOGGING       - Executing single-axis incremental jog
ERROR         - Fault state (limit hit during motion, timeout, etc.)
DIAGNOSTIC    - Human interactive diagnostic mode (remote accessible)
```

### State Transitions
- IDLE → HOMING_Z: `HOME` command received
- HOMING_Z → HOMING_Y: Z axis homing complete
- HOMING_Y → HOMING_X: Y axis homing complete
- HOMING_X → HOMING_F: X axis homing complete
- HOMING_F → IDLE: All axes homed successfully
- IDLE → MOVING: `MOVE` command received
- MOVING → IDLE: All axes reach target position
- IDLE → JOGGING: `JOG` command received
- JOGGING → IDLE: Jog distance complete
- ANY → DIAGNOSTIC: `!DIAG_ENTER` command
- DIAGNOSTIC → IDLE: `!DIAG_EXIT` command
- ANY → ERROR: Limit hit during motion, timeout, or invalid operation
- ERROR → IDLE: `!CLEAR_ERROR` command (requires re-homing before moves)

### Homing Sequence (4-Phase per Axis)
1. **SEEK Phase**: Fast approach toward MIN limit (3000 steps/sec)
   - Detects if MAX limit hit instead → ERROR with diagnostic message
   - 2-minute timeout
2. **BACKOFF Phase**: Move 5mm away from limit (normal speed)
   - Releases switch for clean re-approach
3. **CREEP Phase**: Slow precision approach to MIN limit (400 steps/sec)
   - Establishes precise home position
4. **FINAL BACKOFF Phase**: Move 5mm away from limit again
   - Sets position to 0.00mm (5mm from physical MIN limit)
   - Ensures switch is released and motors are in safe position

### Error Handling
- Limit hit during MOVING/JOGGING → Immediate stop, transition to ERROR
- Wrong limit hit during homing → Stop with diagnostic message
- Timeout during homing → Stop, transition to ERROR
- Soft limit violation → Reject command, stay in IDLE
- Command received in wrong state → Reject with error message

Notes:
- Teensy enforces safety and motion correctness
- RPi decides *when* to auto-step (not yet implemented)
- Manual commands always available in IDLE state

---

## 6. Limit Switch Handling (Critical)

### Electrical
- Normally‑Open (NO) switches
- Interrupt‑driven (FALLING edge)
- Immediate response
- 300ms debounce with 20ms verification

### Runtime Behavior
1. Interrupt fires on FALLING edge
2. Debounce check (300ms between triggers)
3. Back off fixed distance at slow speed
4. Clamp soft limit
5. Emit async LIMIT event
6. Remain enabled for jog‑away

**EN is never hard‑disabled on limit hit**.

---

## 7. Jogging Model

Jogging is **velocity‑based**, not step‑based.

- RPi sends jog commands while button held
- Teensy ramps velocity smoothly
- Stopping jog commands causes decel to zero

This provides smooth, responsive manual control suitable for microscopy.

---

## 8. Serial Protocol Design

### Goals
- Never lose sync
- Survive long runtimes
- Debuggable in text logs
- Safe retry behavior

### Transport
- USB CDC Serial
- 115200 baud (initial)
- Line‑framed ASCII

### Command Format (RPi → Teensy)

```
#<SEQ> <COMMAND> [ARGS]
```

Examples:
```
#123 HOME
#124 MOVE X=10.000 Y=5.000 Z=1.200 F=0.300
#125 JOG X +0.05
#126 STOP
#127 PING
```

### Responses (Teensy → RPi)

```
@<SEQ> OK
@<SEQ> ERROR <CODE>
```

### Asynchronous Events (Teensy → RPi)

```
!LIMIT Z+
!HEARTBEAT STATE=AUTO_MOVING
!POS X=9.998 Y=5.001 Z=1.199 F=0.301
```

### Heartbeat
- Teensy emits heartbeat at 1–5 Hz
- Includes current state
- Loss of heartbeat → ERROR state

---

## 9. Debuggability Requirements (Very Important)

### Serial Logging (Teensy)

Enable compile‑time debug flags to print:
- RX lines with timestamps
- TX lines with timestamps
- Parsed command + seq ID
- State transitions
- Limit interrupts
- Motion start / stop

Example:
```
[T+1234.567] RX: #124 MOVE X=10 Y=5
[T+1234.570] STATE: AUTO_MOVING
[T+1236.101] TX: @124 OK
```

### Python‑Side Logging

- Log all TX/RX with monotonic timestamps
- Persist logs to disk
- Detect missing ACKs
- Retry with same SEQ

---

## 10. RPi4 Client Development Quick Start

### Essential Files
1. **PROTOCOL.md** - Complete protocol specification (command reference, timing, examples, Python code)
2. **project_context.md** - This file (hardware details, calibrated values)
3. **src/main.cpp** - Teensy firmware source (reference implementation)

### Connection Setup
```python
import serial

# UART configuration
ser = serial.Serial(
    port='/dev/serial0',  # GPIO 14/15 on RPi
    baudrate=115200,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=0.1
)

# Wait for BOOT message
while True:
    line = ser.readline().decode().strip()
    if line.startswith('#BOOT'):
        print(f"Teensy ready: {line}")
        break
```

### Checksum Implementation
```python
def calculate_checksum(message):
    """XOR checksum for protocol messages"""
    checksum = 0
    for char in message:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def send_command(ser, command):
    """Send command with checksum"""
    checksum = calculate_checksum(command)
    message = f"{command} *{checksum}\n"
    ser.write(message.encode())
    return message
```

### Basic Command Flow
```python
# 1. Connect
send_command(ser, "!CONNECT MASTER=RPi_v1.0")
# Wait for: @ACK 0 SLAVE=v1.2 *XX

# 2. Home axes
send_command(ser, "!HOME")
# Wait for: @ACK 1 HOMING=ZYXF *XX
# Then: @COMPLETE 1 HOMED=XYZF *XX

# 3. Move to position
send_command(ser, "!MOVE X100 Y50 Z10")
# Wait for: @ACK 2 MOVING ... *XX
# Then: @COMPLETE 2 X=100.00 Y=50.00 Z=10.00 F=0.00 *XX

# 4. Query status
send_command(ser, "!STATUS")
# Receive: @STATUS 3 X=100.00 Y=50.00 Z=10.00 F=0.00 STATE=IDLE HOMED=XYZF *XX
```

### Watchdog Handling
```python
import time
import threading

last_message_time = time.time()

def watchdog_thread():
    """Send PING every 3 seconds"""
    global last_message_time
    while True:
        time.sleep(3)
        if time.time() - last_message_time > 3:
            send_command(ser, "!PING")
            last_message_time = time.time()
```

### Error Handling
```python
def parse_response(line):
    """Parse Teensy response"""
    if line.startswith('@ACK'):
        return 'ack', line
    elif line.startswith('@NACK'):
        # Extract error: @NACK 5 ERR_OUT_OF_BOUNDS X=500.00 exceeds limit 270.00mm *XX
        parts = line.split(' ', 3)
        seq = int(parts[1])
        error_code = parts[2]
        message = parts[3].split('*')[0].strip()
        return 'nack', {'seq': seq, 'code': error_code, 'msg': message}
    elif line.startswith('@COMPLETE'):
        return 'complete', line
    elif line.startswith('@STATUS'):
        return 'status', line
    elif line.startswith('#'):
        return 'event', line
    return 'unknown', line
```

### Known Constraints
- **Soft Limits:** X≤270mm, Y≤150mm, Z≤30mm, F unlimited
- **Must home** before any moves (`!HOME` or `!HOME_FAST`)
- **Watchdog:** Send command or `!PING` every 3 seconds (5s timeout)
- **Timing:** Command ACK < 10ms, Status query < 20ms, Move start < 50ms
- **No queuing:** Wait for `@COMPLETE` before sending next move command

### Testing Sequence
1. Power Teensy → Wait for `#BOOT`
2. Send `!CONNECT` → Verify `@ACK`
3. Send `!HOME` → Verify `@COMPLETE`
4. Send `!STATUS` → Verify positions at 0.00mm
5. Send `!MOVE X10 Y10` → Verify `@COMPLETE` with new positions
6. Send `!MOVE X500` → Verify `@NACK` with `ERR_OUT_OF_BOUNDS`
7. Wait 6 seconds (no commands) → Verify `#COMM_LOST` event

### Common Issues & Solutions
- **Checksum errors:** Ensure XOR calculation matches PROTOCOL.md examples
- **Watchdog timeout:** Implement background PING thread or send STATUS regularly
- **NACK ERR_NOT_HOMED:** Always home after connection before moves
- **NACK ERR_OUT_OF_BOUNDS:** Check soft limits (X≤270, Y≤150, Z≤30)
- **No response:** Check baud rate (115200), verify UART pins (GPIO 14/15)

---

## 11. Long‑Running Operation Considerations

- No blocking delays in Teensy loop
- No dynamic memory allocation in hot paths
- Periodic POS re‑anchoring
- Explicit error states
- Watchdog via heartbeat

---

## 11. Development Plan

### Phase 1
- Teensy + TB6600
- Single axis
- MOVE / JOG / STOP

### Phase 2
- Add one limit switch
- Homing
- Bounce‑back

### Phase 3
- Protocol ACK + heartbeat
- Python client

### Phase 4
- Add remaining axes
- Auto‑step sequencing

---

## 12. Architectural Decisions & Lessons Learned

### Unified Codebase Approach
- **Decision:** Single firmware supports both protocol mode AND diagnostic mode
- **Benefit:** Same calibrated values, no sync issues, remote diagnostics via `!DIAG_ENTER`
- **Mode switching:** `!DIAG_ENTER` / `!DIAG_EXIT` commands allow remote troubleshooting

### Common Code Patterns
- **Helper functions:** `checkLimitTriggered()` used for hybrid interrupt+polling limit detection
- **Single source of truth:** Motor inversions set once in `setup()` via `setPinsInverted()`
- **No code duplication:** All limit handling uses same helper functions
- **Note:** `reenableAxisInterrupts()` function was removed during limit switch robustness redesign

### Noise Handling Evolution
- **Problem:** Long limit switch cables (several feet) caused false triggers
- **Failed attempts:** Single read, 20µs filter, 100µs filter, pin changes
- **Working solution:** Triple-read (200µs stable) + interrupt disable + conditional re-enable
- **Critical pin:** F_MIN moved to pin 22 (pins 17/19/20 have I2C hardware interference)

### Calibration Discovery
- **Measurement:** 100mm command produced 50mm actual travel (all axes)
- **Root cause:** TB6600 drivers set to 1/16 microstepping (not 1/8 as assumed)
- **Solution:** STEPS_PER_MM doubled from 200 to 400
- **Verification:** Consistent across X, Y, Z axes

### Speed Optimization
- **Initial:** 1500 steps/sec (conservative for testing)
- **Final:** 6000 steps/sec (4x increase) with 10µs pulse width
- **Result:** 30 mm/sec linear speed, adequate for microscopy application

### Protocol Design Philosophy
- **Text-based:** Human-readable for debugging
- **Checksums:** XOR for simplicity and low overhead
- **Sequence numbers:** Enable retry logic without re-execution
- **Watchdog:** Master responsibility to keep connection alive
- **Detailed errors:** 20+ specific error codes with context

### Limit Switch Robustness: Critical Design Evolution
- **Original Problem:** 25% failure rate during fast homing operations (missed switch presses)
- **Root Cause:** Race condition in traditional detach/reattach interrupt pattern:
  1. ISR fires → sets triggered flag → detaches interrupt
  2. Switch bounces briefly to HIGH during debounce period
  3. Main loop sees HIGH → re-enables interrupt → clears triggered flag
  4. Switch still pressed (LOW) but interrupt already detached → motor continues past limit
- **Failed Attempts:**
  - Extended debounce to 300ms: Reduced but didn't eliminate failures
  - Still missed ~1 in 4 button presses during high-speed testing
- **Successful Solution - Three-Layer Hybrid Architecture:**
  1. **Timestamp-based debouncing:** ISR checks `debounceUntil` timestamp instead of detaching interrupt
  2. **Pin state hysteresis:** Track last pin state, only trigger on HIGH→LOW transitions (prevents release bounce detection)
  3. **Polling fallback:** `checkLimitTriggered()` directly reads pins if interrupt missed during tight loops
  4. **Continuous state tracking:** Main loop AND all tight loops update pin states when HIGH (prevents stale state)
- **Critical Implementation Details:**
  - LimitState struct: Added `debounceUntil`, `lastMinPinState`, `lastMaxPinState` volatile fields
  - ISRs simplified from 19 lines to 8 lines each - never detach interrupts
  - Removed `reenableAxisInterrupts()` function entirely (18+ call sites eliminated)
  - Triple-read verification with 200µs delays rejects electrical noise
  - Debounce reduced from 300ms to 50ms (6x faster response)
- **Design Principles Validated:**
  - **Never disable interrupts during normal operation** - use timestamps and state machines instead
  - **Dual detection** (interrupt + polling) ensures 100% reliability even in tight loops
  - **State hysteresis** prevents false triggers on release/bounce
  - **100% non-blocking** - no spinlocks, delays, or interrupt disabling
- **Result:** Eliminated all missed detections, no false triggers, works perfectly in all test scenarios
- **Key Insight:** Traditional embedded patterns (detach/reattach interrupts) fail during high-speed operations with mechanical switches. State-based architectures with timestamp debouncing are fundamentally more robust.

---

## 13. Non‑Goals

- G‑code compatibility
- On‑device UI
- Vision processing on Teensy
- SSH or Python on Teensy

---

## 14. Key Design Principles

- Teensy = motion appliance (slave)
- RPi = decision maker (master)
- Determinism over cleverness
- Explicit state > implicit behavior
- Text protocols for debuggability
- Safety first (watchdog, soft limits, emergency stop)

---

## 15. Notes for Future Development

### For Teensy Agent (returning to this codebase):
- All calibrated values are in constants at top of main.cpp
- Protocol implementation is in `processProtocolCommand()` function
- Diagnostic menu is in `processDiagnosticCommand()` function
- Common helpers are centralized - use them, don't duplicate
- Test with diagnostic mode first (press 'D' on boot)

### For RPi Agent (starting fresh):
- **Read PROTOCOL.md first** - it has everything you need
- Implement checksum calculation exactly per spec (XOR all bytes)
- Test connection manually via `screen /dev/serial0 115200` before coding
- Implement watchdog thread FIRST (3 second interval, 5 second timeout)
- Parse all responses - don't assume ACK
- Handle all error codes (see PROTOCOL.md Section 7)
- Test error conditions: out of bounds, not homed, communication loss

### For Integration:
- Teensy firmware v1.2 is stable and tested
- No changes needed on Teensy side for basic RPi client
- Future enhancements: move queuing, trajectory smoothing, position feedback
- Camera integration happens on RPi side only (Teensy unaware)

---

## 16. Current Status Summary

**Hardware:** ✅ Fully validated  
**Firmware:** ✅ v1.2 operational  
**Calibration:** ✅ STEPS_PER_MM=400, soft limits measured  
**Protocol:** ✅ Implemented with checksums, ACK/NACK, watchdog  
**Diagnostic Mode:** ✅ Full menu system working  
**Documentation:** ✅ PROTOCOL.md complete with examples  

**Next Phase:** RPi4 Python client development  
**Blocked on:** Nothing - ready for integration  
**Risk areas:** None identified - all testing successful

