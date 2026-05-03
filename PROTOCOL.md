# Teensy 4.1 Motion Controller Protocol Specification
**Version:** 1.1  
**Date:** January 5, 2026  
**Author:** AI Assistant for Teensy Motion Control System

---

## 1. Overview

This document defines the serial communication protocol between a Raspberry Pi 4 (Master) and Teensy 4.1 (Slave) for a 4-axis CNC motion controller. The protocol is designed for robustness, async startup, communication loss detection, and detailed error reporting.

### 1.1 Architecture
- **Master:** Raspberry Pi 4 (initiates all commands)
- **Slave:** Teensy 4.1 (responds to commands, sends status/events)
- **Topology:** Point-to-point UART connection
- **Startup:** Asynchronous - either device may boot first
- **Axes:** X, Y, Z (lead screw driven), F/Zoom (belt driven, 1:4 ratio)

### 1.2 Design Goals
- **Robust:** Checksums, sequence numbers, ACK/NACK, retry mechanism
- **Safe:** Communication watchdog, automatic motor shutdown on loss
- **Debuggable:** Human-readable text protocol, all messages visible on console
- **Extensible:** Detailed error codes, version negotiation, future commands
- **Deterministic:** Clear state machine, timeout handling

---

## 2. Physical Layer

### 2.1 UART Configuration
| Parameter | Value |
|-----------|-------|
| Baud Rate | 115200 |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |
| Line Ending | `\n` (LF, 0x0A) |

### 2.2 Connection
- **Teensy Serial1:** Hardware UART on pins 0 (RX) and 1 (TX) at 115200 baud
- **Teensy Serial (USB):** Used for diagnostic console output only
- **RPi GPIO UART:** `/dev/serial0` (GPIO 14/15, pins 8/10)
- **Cable:** 3-wire minimum (TX, RX, GND)

**Hardware Configuration:**
- Protocol commands from RPi4 → Teensy Serial1 (pins 0/1)
- Diagnostic output → both Serial (USB console) and Serial1 (RPi4)
- `#DIAG` messages mirrored to Serial1 for remote diagnostic visibility

---

## 3. Message Format

### 3.1 Frame Structure
All messages are line-oriented (newline terminated). Three message types:

#### **Commands (Master → Slave)**
```
!<CMD> [args] *<checksum>\n
```
- `!` - Command prefix
- `<CMD>` - Command name (uppercase, letters/underscores)
- `[args]` - Space-separated arguments (optional)
- `*` - Checksum delimiter
- `<checksum>` - 2-digit hex XOR checksum of everything before `*`
- `\n` - Line terminator

**Example:**
```
!MOVE X100 Y50 *3A\n
```

#### **Responses (Slave → Master)**
```
@<TYPE> <seq> [data] *<checksum>\n
```
- `@` - Response prefix
- `<TYPE>` - `ACK`, `NACK`, `STATUS`, `COMPLETE`, etc.
- `<seq>` - Sequence number echoed from command
- `[data]` - Response data (optional)
- `*<checksum>` - 2-digit hex XOR checksum

**Examples:**
```
@ACK 42 *1F\n
@NACK 43 ERR_LIMIT X exceeds 270mm *5C\n
@STATUS 44 X=100.00 Y=50.25 Z=0.00 F=10.50 STATE=IDLE *7A\n
```

#### **Events (Slave → Master, unsolicited)**
```
#<EVENT> [data] *<checksum>\n
```
- `#` - Event prefix
- `<EVENT>` - Event name
- `[data]` - Event data (optional)
- `*<checksum>` - 2-digit hex XOR checksum

**Examples:**
```
#LIMIT X_MIN *2B\n
#COMM_LOST Watchdog timeout *4D\n
#BOOT FW=v1.2 AXES=4 *1E\n
```

### 3.2 Checksum Calculation
XOR all bytes from start to checksum delimiter `*` (exclusive):
```c
uint8_t checksum = 0;
for (char c : message_before_asterisk) {
    checksum ^= c;
}
sprintf(checksum_str, "%02X", checksum);
```

### 3.3 Sequence Numbers
- Master assigns increasing sequence number to each command (0-255, wraps)
- Uses uint8 type - sufficient for duplicate detection with 10-command history window
- Slave echoes sequence number in ACK/NACK
- Used for retry logic and duplicate detection

---

## 4. Protocol State Machine

### 4.1 Slave States
| State | Description | Motor Enable |
|-------|-------------|--------------|
| `BOOT` | Power-on, initializing | Disabled |
| `DISCONNECTED` | No master connection established | Disabled |
| `CONNECTED` | Master handshake complete | Disabled |
| `IDLE` | Ready for commands, not moving | Enabled |
| `HOMING` | Executing homing sequence | Enabled |
| `MOVING` | Executing coordinated move | Enabled |
| `JOGGING` | Executing incremental move | Enabled |
| `ERROR` | Limit hit, timeout, or fault | Disabled |
| `DIAGNOSTIC` | Human interactive mode | Enabled |
| `COMM_LOST` | Watchdog expired, no master | Disabled |

### 4.2 State Transitions
```
BOOT → DISCONNECTED (on startup)
DISCONNECTED → CONNECTED (on !CONNECT)
CONNECTED → IDLE (on !HOME)
IDLE → MOVING (on !MOVE)
MOVING → IDLE (on completion)
IDLE → HOMING (on !HOME)
HOMING → IDLE (on completion)
ANY → ERROR (on limit hit, hardware fault)
ANY → COMM_LOST (on watchdog timeout)
ANY → DIAGNOSTIC (on !DIAG_ENTER)
DIAGNOSTIC → IDLE (on !DIAG_EXIT)
ERROR → IDLE (on !CLEAR_ERROR)
COMM_LOST → DISCONNECTED (auto-recovery attempt)
```

### 4.3 Startup Sequence

**Case 1: Teensy boots first**
```
Teensy: #BOOT FW=v1.2 AXES=4 *1E
Teensy: (enters DISCONNECTED state)
Teensy: (waits for master...)

RPi: !CONNECT MASTER=v1.0 *3C
Teensy: @ACK 0 SLAVE=v1.2 *2F
Teensy: (enters CONNECTED state)
```

**Case 2: RPi boots first**
```
RPi: (waits for #BOOT or sends !PING periodically)
Teensy: #BOOT FW=v1.2 AXES=4 *1E

RPi: !CONNECT MASTER=v1.0 *3C
Teensy: @ACK 0 SLAVE=v1.2 *2F
```

**Case 3: Both booting simultaneously**
```
Teensy: #BOOT FW=v1.2 AXES=4 *1E
RPi: (sees #BOOT, waits 500ms for bootup)
RPi: !CONNECT MASTER=v1.0 *3C
Teensy: @ACK 0 SLAVE=v1.2 *2F
```

---

## 5. Communication Watchdog

### 5.1 Slave Watchdog
- **Timeout:** 5 seconds of no messages from master
- **Action:** 
  1. Immediately stop all motors (emergency stop)
  2. Transition to `COMM_LOST` state
  3. Send `#COMM_LOST Watchdog timeout *XX`
  4. Disable motor drivers
  5. Attempt auto-recovery (send `#BOOT` every 10 seconds)

### 5.2 Master Heartbeat
- **Requirement:** Master must send command or `!PING` at least every 3 seconds
- **Recommendation:** Send `!STATUS` every 1 second during idle
- **During moves:** Move completion messages reset watchdog

### 5.3 Recovery
```
Teensy: #COMM_LOST Watchdog timeout *4D
Teensy: (10 seconds later) #BOOT FW=v1.2 AXES=4 *1E
RPi: !CONNECT MASTER=v1.0 *3C
Teensy: @ACK 0 SLAVE=v1.2 *2F
RPi: !HOME *1A (re-home after recovery)
```

---

## 6. Command Set

### 6.1 Connection Management

#### `!CONNECT MASTER=<version>`
Establish connection with master.
- **State:** `DISCONNECTED` → `CONNECTED`
- **Response:** `@ACK <seq> SLAVE=<version>`
- **Errors:** None

#### `!DISCONNECT`
Gracefully disconnect (for shutdown/reboot).
- **State:** Any → `DISCONNECTED`
- **Response:** `@ACK <seq>`
- **Action:** Stop motors, disable drivers

#### `!PING`
Keep-alive / watchdog reset.
- **State:** No change
- **Response:** `@ACK <seq> UPTIME=<ms>`
- **Errors:** None

### 6.2 Motion Commands

#### `!HOME [axes]`
Sequential homing of specified axes (default: all).
- **Args:** `XYXF` (any combination, e.g., `XZ`)
- **State:** `IDLE` → `HOMING` → `IDLE`
- **Response:** 
  - `@ACK <seq>` (started)
  - `#HOMING_PROGRESS X_MIN_FOUND` (events during)
  - `@COMPLETE <seq> HOMED=XYZF` (finished)
- **Errors:**
  - `ERR_ALREADY_MOVING` - Cannot home while moving
  - `ERR_LIMIT_HIT` - Wrong limit triggered
  - `ERR_TIMEOUT` - Homing timeout (120s)

#### `!HOME_FAST`
Parallel homing of all axes (faster).
- **Args:** None
- **State:** `IDLE` → `HOMING` → `IDLE`
- **Response:** Same as `!HOME`
- **Errors:** Same as `!HOME`

#### `!MOVE X<val> Y<val> Z<val> F<val>`
Coordinated absolute move.
- **Args:** Position in mm (omitted axes don't move)
- **State:** `IDLE` → `MOVING` → `IDLE`
- **Response:**
  - `@ACK <seq>` (motion started)
  - `@COMPLETE <seq> X=<x> Y=<y> Z=<z> F=<f>` (motion finished, includes final position)
- **Position Synchronization:** Master MUST use position from `@COMPLETE` as authoritative. Do not update local position until `@COMPLETE` is received.
- **Completion Detection:** If @COMPLETE is lost, master should poll !STATUS to check STATE=IDLE
- **Timeout Guidance:** Calculate expected duration from distance/speed + 2s margin, then poll !STATUS
- **Errors:**
  - `ERR_NOT_HOMED` - Must home first
  - `ERR_OUT_OF_BOUNDS` - X exceeds 270mm (etc.)
  - `ERR_ALREADY_MOVING` - Previous move not complete
  - `ERR_LIMIT_HIT` - Hit limit during move

**Example:**
```
!MOVE X100 Y50.5 Z10 *3A
@ACK 12 *4B
... (motion occurs) ...
@COMPLETE 12 X=100.00 Y=50.50 Z=10.00 F=0.00 *6C
```

#### `!JOG <axis> <distance>`
Incremental move relative to current position.
- **Args:** Axis (X/Y/Z/F), distance in mm (+ or -)
- **State:** `IDLE` → `JOGGING` → `IDLE`
- **Response:**
  - `@ACK <seq>` (jog started)
  - `@COMPLETE <seq> X=<x> Y=<y> Z=<z> F=<f>` (jog finished, includes final position)
- **Position Synchronization:** Master MUST use position from `@COMPLETE` as authoritative. Do not update local position until `@COMPLETE` is received. This prevents desync when commands are rejected due to checksum errors.
- **Errors:** Same as `!MOVE`

**Example:**
```
!JOG F 0.20 *09
@ACK 13 *1A
... (motion occurs) ...
@COMPLETE 13 X=0.00 Y=0.00 Z=1.80 F=10.60 *3D
```

#### `!STOP`
Emergency stop - immediately halt all motion.
- **State:** Any → `IDLE`
- **Response:** `@ACK <seq> STOPPED`
- **Errors:** None

#### `!CLEAR_ERROR`
Clear error state and return to IDLE.
- **State:** `ERROR` → `IDLE`
- **Response:** `@ACK <seq> ERROR_CLEARED`
- **Errors:**
  - `ERR_INVALID_STATE` - Not in ERROR state
- **Note:** After clearing error, must re-home before moving

### 6.3 Query Commands

#### `!STATUS`
Get current position, state, and system status.
- **Response:** `@STATUS <seq> X=... Y=... Z=... F=... STATE=... HOMED=...`
- **Errors:** None

**Example:**
```
!STATUS *1F
@STATUS 14 X=100.00 Y=50.50 Z=10.00 F=0.00 STATE=IDLE HOMED=XYZF *7A
```

#### `!VERSION`
Get firmware version and capabilities.
- **Response:** `@ACK <seq> FW=v1.2 HW=Teensy4.1 AXES=4 LIMITS=8`
- **Errors:** None

#### `!LIMITS`
Get current limit switch states.
- **Response:** `@ACK <seq> X_MIN=0 X_MAX=0 Y_MIN=0 ... (1=pressed)`
- **Errors:** None

### 6.4 Configuration Commands

#### `!SET_SPEED <speed>`
Set maximum speed (steps/sec).
- **Args:** Speed value (1000-20000)
- **Response:** `@ACK <seq> SPEED=<speed>`
- **Errors:** `ERR_OUT_OF_RANGE` - Speed invalid

#### `!SET_ACCEL <accel>`
Set acceleration (steps/sec²).
- **Args:** Acceleration value (500-10000)
- **Response:** `@ACK <seq> ACCEL=<accel>`
- **Errors:** `ERR_OUT_OF_RANGE` - Acceleration invalid

### 6.5 Diagnostic Mode

#### `!DIAG_ENTER`
Enter diagnostic mode (human interactive).
- **State:** Any → `DIAGNOSTIC`
- **Response:** `@ACK <seq> ENTERING_DIAGNOSTIC`
- **Action:** Print diagnostic main menu
- **Note:** Protocol commands paused (except !PING, !DIAG_EXIT, !STATUS, !DIAG_CMD, !DIAG_MENU)
- **Watchdog:** DISABLED during diagnostic mode for safety (re-enabled on !DIAG_EXIT)

#### `!DIAG_EXIT`
Exit diagnostic mode, return to protocol control.
- **State:** `DIAGNOSTIC` → `IDLE`
- **Response:** `@ACK <seq> EXITING_DIAGNOSTIC`

#### `!DIAG_CMD <cmd>`
Send menu command character in diagnostic mode.
- **Args:** Single character (e.g., `1`, `T`, `S`)
- **State:** Must be in `DIAGNOSTIC`
- **Response:** `@ACK <seq>` followed by menu output
- **Errors:** `ERR_NOT_IN_DIAG` - Not in diagnostic mode

#### `!DIAG_MENU`
Get current diagnostic menu text.
- **State:** Must be in `DIAGNOSTIC`
- **Response:** `@ACK <seq>` followed by menu lines prefixed with `#DIAG`

---

## 7. Error Codes

### 7.1 Protocol Errors (Communication)
| Code | Description | Recovery |
|------|-------------|----------|
| `ERR_CHECKSUM` | Checksum mismatch | NACK, master retries |
| `ERR_UNKNOWN_CMD` | Command not recognized | NACK, check spelling |
| `ERR_INVALID_ARGS` | Missing or malformed arguments | NACK, check syntax |
| `ERR_TIMEOUT` | Command execution timeout | NACK, may need restart |

### 7.2 State Errors (Invalid Operations)
| Code | Description | Recovery |
|------|-------------|----------|
| `ERR_NOT_CONNECTED` | Must `!CONNECT` first | Send `!CONNECT` |
| `ERR_NOT_HOMED` | Must home before move | Send `!HOME` |
| `ERR_ALREADY_MOVING` | Previous motion not complete | Wait or `!STOP` |
| `ERR_NOT_IN_DIAG` | Diagnostic command in wrong mode | Send `!DIAG_ENTER` |
| `ERR_INVALID_STATE` | Operation not allowed in current state | Check `!STATUS` |

### 7.3 Motion Errors (Mechanical/Safety)
| Code | Description | Recovery |
|------|-------------|----------|
| `ERR_LIMIT_HIT` | Limit switch triggered | `!CLEAR_ERROR`, re-home |
| `ERR_OUT_OF_BOUNDS` | Position exceeds soft limits | Adjust target position |
| `ERR_HOMING_FAILED` | Homing did not complete | Check wiring, retry |
| `ERR_MOTOR_FAULT` | Driver fault detected | Check power, wiring |

### 7.4 System Errors (Critical)
| Code | Description | Recovery |
|------|-------------|----------|
| `ERR_COMM_LOST` | Watchdog timeout | Re-establish connection |
| `ERR_HARDWARE` | Internal hardware error | Power cycle |
| `ERR_EMERGENCY_STOP` | User-initiated E-stop | `!CLEAR_ERROR`, re-home |

---

## 8. ACK/NACK and Retry

### 8.1 ACK Response
Indicates command was received, validated, and started:
```
@ACK <seq> [optional_data] *<checksum>\n
```

### 8.2 NACK Response
Indicates command was rejected:
```
@NACK <seq> <error_code> <human_message> *<checksum>\n
```

**Example:**
```
!MOVE X500 Y50 *2A
@NACK 15 ERR_OUT_OF_BOUNDS X=500.00 exceeds limit 270.00mm *5F
```

### 8.3 Retry Logic (Master)
1. Send command with sequence number
2. Wait for ACK/NACK (timeout: 1 second)
3. If no response or `ERR_CHECKSUM`:
   - Retry up to 3 times
   - If still failing, abort and report error
4. If NACK with other error:
   - Do not retry (error is deterministic)
   - Report error to application layer

### 8.4 Duplicate Detection (Slave)
- If same sequence number received twice:
  - Assume retransmit due to lost ACK
  - Re-send ACK, do not re-execute command
- Sequence number window: last 10 commands

---

## 9. Asynchronous Events

Events are sent by slave without prompting:

### 9.1 Boot Event
```
#BOOT FW=<version> AXES=<count> *<checksum>\n
```
Sent on power-up or reset.

### 9.2 Limit Events
```
#LIMIT <axis>_<min/max> *<checksum>\n
```
Sent when limit switch triggers (even if not moving).

**Example:**
```
#LIMIT X_MAX *3B
```

### 9.3 Communication Lost Event
```
#COMM_LOST <reason> *<checksum>\n
```
Sent when watchdog expires or connection drops.

### 9.4 Homing Progress Events
```
#HOMING_PROGRESS <axis> <phase> *<checksum>\n
```
Optional detailed homing status.

**Example:**
```
#HOMING_PROGRESS X SEEK *2A
#HOMING_PROGRESS X CREEP *3F
#HOMING_PROGRESS X COMPLETE *1D
```

### 9.5 Diagnostic Output
When in diagnostic mode, menu output is prefixed:
```
#DIAG <menu_line> *<checksum>\n
```

---

## 10. Timing and Performance

### 10.1 Command Latency
- **ACK response:** < 10ms typical
- **Status query:** < 20ms
- **Move start:** < 50ms (after ACK)

### 10.2 Status Update Rate
- Master can query `!STATUS` up to 20 Hz (every 50ms)
- Recommended: 1 Hz during idle, 10 Hz during motion

### 10.3 Move Completion
- `@COMPLETE` sent immediately when motion stops
- Master polls `!STATUS` if completion not received within expected time

### 10.4 Throughput
- UART at 115200 baud ≈ 11.5 KB/sec
- Typical command: ~50 bytes
- Max command rate: ~200 commands/sec (far exceeds needs)

---

## 11. Example Communication Sessions

### 11.1 Startup and Homing
```
Teensy: #BOOT FW=v1.2 AXES=4 *1E
RPi:    !CONNECT MASTER=v1.0 *3C
Teensy: @ACK 0 SLAVE=v1.2 *2F

RPi:    !HOME *1A
Teensy: @ACK 1 HOMING=ZYXF *4B
Teensy: #HOMING_PROGRESS Z SEEK *2A
Teensy: #HOMING_PROGRESS Z COMPLETE *3D
... (Y, X, F homing) ...
Teensy: @COMPLETE 1 HOMED=XYZF *5C

RPi:    !STATUS *1F
Teensy: @STATUS 2 X=0.00 Y=0.00 Z=0.00 F=0.00 STATE=IDLE HOMED=XYZF *7A
```

### 11.2 Coordinated Move
```
RPi:    !MOVE X100 Y50 Z10 *3A
Teensy: @ACK 3 MOVING X=100.0mm Y=50.0mm Z=10.0mm *5F
... (3 seconds of motion) ...
Teensy: @COMPLETE 3 X=100.00 Y=50.00 Z=10.00 F=0.00 *6C

RPi:    !STATUS *1F
Teensy: @STATUS 4 X=100.00 Y=50.00 Z=10.00 F=0.00 STATE=IDLE HOMED=XYZF *8B
```

### 11.3 Error Handling
```
RPi:    !MOVE X500 *2A
Teensy: @NACK 5 ERR_OUT_OF_BOUNDS X=500.00 exceeds limit 270.00mm *5F

RPi:    !MOVE X100 Y50 Z10 *3A
Teensy: @ACK 6 MOVING X=0.0mm Y=0.0mm Z=0.0mm *4D
... (hits limit during move) ...
Teensy: #LIMIT Y_MAX *3B
Teensy: @NACK 6 ERR_LIMIT_HIT Limit switch triggered during move *7E

RPi:    !STATUS *1F
Teensy: @STATUS 7 X=100.00 Y=145.23 Z=8.45 F=0.00 STATE=ERROR HOMED=XYZF *9A

RPi:    !CLEAR_ERROR *4B
Teensy: @ACK 8 ERROR_CLEARED *3D

RPi:    !HOME Y *2F
Teensy: @ACK 9 HOMING=Y *1A
... (re-home Y axis) ...
Teensy: @COMPLETE 9 HOMED=XYZF *4C
```

### 11.4 Communication Loss and Recovery
```
RPi:    !STATUS *1F
Teensy: @STATUS 10 X=50.00 Y=25.00 Z=5.00 F=0.00 STATE=IDLE HOMED=XYZF *8C

... (5 seconds of no communication) ...

Teensy: #COMM_LOST Watchdog timeout *4D
Teensy: #STATE_CHANGE COMM_LOST *3E

... (10 seconds later) ...

Teensy: #BOOT FW=v1.2 AXES=4 *1E
RPi:    !CONNECT MASTER=v1.0 *3C
Teensy: @ACK 0 SLAVE=v1.2 *2F
Teensy: #STATE_CHANGE CONNECTED *5F

RPi:    !STATUS *1F
Teensy: @STATUS 1 X=50.00 Y=25.00 Z=5.00 F=0.00 STATE=CONNECTED HOMED=XYZF *9D
         (position preserved, homed flags preserved, but master should re-home for safety)

RPi:    !HOME *1A
... (re-establish coordinate system) ...
```

### 11.5 Diagnostic Mode
```
RPi:    !DIAG_ENTER *2A
Teensy: @ACK 20 ENTERING_DIAGNOSTIC *4F
Teensy: #DIAG ======================================== *3A
Teensy: #DIAG DIAGNOSTIC MAIN MENU *1F
Teensy: #DIAG ======================================== *3A
Teensy: #DIAG 1 - HOME All Axes *2B
... (menu lines) ...

RPi:    !DIAG_CMD 1 *3C
Teensy: @ACK 21 CMD_SENT *2D
Teensy: #DIAG Starting sequential homing... *4E
... (homing output) ...
Teensy: #DIAG HOMING COMPLETE *1F

RPi:    !DIAG_EXIT *4D
Teensy: @ACK 22 EXITING_DIAGNOSTIC *5E
Teensy: #STATE_CHANGE IDLE *2F
```

---

## 12. Implementation Notes

### 12.1 Teensy Side (Slave)
- Use `Serial.available()` and `Serial.readStringUntil('\n')` for line-oriented parsing
- Maintain watchdog timer that resets on any received message
- Echo all protocol messages to console for debugging (prefix with timestamp)
- Implement command parser that validates checksum before processing
- Use state machine for connection management and safety

### 12.2 RPi Side (Master)
- Use `pyserial` for UART communication
- Implement retry logic with exponential backoff
- Monitor for `#BOOT` events to detect Teensy resets
- Send periodic `!PING` or `!STATUS` to keep watchdog alive
- Queue commands and wait for ACK/COMPLETE before sending next
- Log all communication for debugging

### 12.3 Testing Strategy
1. **Unit test:** Checksum calculation, command parsing
2. **Integration test:** Send commands via terminal emulator
3. **Stress test:** Rapid command sequences, communication loss simulation
4. **Safety test:** Verify watchdog, verify soft limits, verify E-stop

---

## 13. Future Extensions

### 13.1 Possible Additions
- `!SET_HOME X<val> Y<val>` - Set current position as home offset
- `!MOVE_RELATIVE` - Alias for `!JOG` with multiple axes
- `!SPEED_OVERRIDE <percent>` - Temporary speed adjustment
- `!PAUSE` / `!RESUME` - Pause motion mid-move
- `!QUEUE_MOVE` - Buffer multiple moves for smoother paths
- Binary protocol mode for higher throughput
- CRC16 checksums for even more robustness

### 13.2 Version Negotiation
Future protocol versions can add features while maintaining backward compatibility:
- `!CONNECT` includes version number
- Slave responds with supported features
- Both sides use lowest common denominator

---

## 14. Quick Reference

### 14.1 Command Summary
| Command | Description | Response |
|---------|-------------|----------|
| `!CONNECT` | Establish connection | `@ACK` |
| `!PING` | Watchdog keepalive | `@ACK` |
| `!HOME` | Sequential homing | `@ACK` → `@COMPLETE` |
| `!HOME_FAST` | Parallel homing | `@ACK` → `@COMPLETE` |
| `!MOVE` | Absolute move | `@ACK` → `@COMPLETE` |
| `!JOG` | Incremental move | `@ACK` → `@COMPLETE` |
| `!STOP` | Emergency stop | `@ACK` |
| `!CLEAR_ERROR` | Clear error state | `@ACK` |
| `!STATUS` | Get position/state | `@STATUS` |
| `!DIAG_ENTER` | Enter diagnostic mode | `@ACK` |
| `!DIAG_EXIT` | Exit diagnostic mode | `@ACK` |

### 14.2 State Summary
- `DISCONNECTED` → `CONNECTED` → `IDLE` → `MOVING`/`HOMING` → `IDLE`
- `ERROR` requires `!CLEAR_ERROR`
- `COMM_LOST` auto-recovers to `DISCONNECTED`

### 14.3 Soft Limits (Enforced)
- X: 0 to 270mm
- Y: 0 to 150mm
- Z: 0 to 30mm
- F: 0 to 11.5mm (zoom axis, belt drive)

### 14.4 Timing
- Watchdog: 5 seconds
- Recommended heartbeat: 1-3 seconds
- Homing timeout: 120 seconds
- Command retry: 3 attempts, 1 second timeout each

---

## Appendix A: Checksum Examples

### Example 1: Simple Command
```
Message: !PING
Calculation: '!' ^ 'P' ^ 'I' ^ 'N' ^ 'G' = 0x21 ^ 0x50 ^ 0x49 ^ 0x4E ^ 0x47 = 0x1F
Result: !PING *1F
```

### Example 2: Command with Arguments
```
Message: !MOVE X100 Y50
Calculation: 0x21 ^ 0x4D ^ 0x4F ^ 0x56 ^ 0x45 ^ 0x20 ^ 0x58 ^ 0x31 ^ 0x30 ^ 0x30 ^ 0x20 ^ 0x59 ^ 0x35 ^ 0x30 = 0x3A
Result: !MOVE X100 Y50 *3A
```

### Example 3: Response
```
Message: @ACK 42 SLAVE=v1.2
Calculation: XOR of all characters = 0x2F
Result: @ACK 42 SLAVE=v1.2 *2F
```

---

## Appendix B: Python Code Snippets

### B.1 Checksum Calculation
```python
def calculate_checksum(message):
    """Calculate XOR checksum for protocol message"""
    checksum = 0
    for char in message:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def add_checksum(message):
    """Add checksum to message"""
    checksum = calculate_checksum(message)
    return f"{message} *{checksum}\n"

# Example usage
cmd = "!MOVE X100 Y50"
cmd_with_checksum = add_checksum(cmd)
print(cmd_with_checksum)  # !MOVE X100 Y50 *3A\n
```

### B.2 Message Validation
```python
def validate_message(line):
    """Validate received message checksum"""
    if '*' not in line:
        return False, "No checksum"
    
    message, checksum_part = line.rsplit('*', 1)
    received_checksum = checksum_part.strip().upper()
    calculated_checksum = calculate_checksum(message)
    
    if received_checksum == calculated_checksum:
        return True, message
    else:
        return False, f"Checksum mismatch: {received_checksum} != {calculated_checksum}"
```

### B.3 Send Command with Retry
```python
import serial
import time

def send_command(ser, command, seq, timeout=1.0, retries=3):
    """Send command with retry logic"""
    cmd_line = f"{command} *{calculate_checksum(command)}\n"
    
    for attempt in range(retries):
        ser.write(cmd_line.encode())
        ser.flush()
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                valid, msg = validate_message(response)
                
                if valid and f"@ACK {seq}" in msg:
                    return True, msg
                elif valid and f"@NACK {seq}" in msg:
                    return False, msg
        
        print(f"Retry {attempt + 1}/{retries}")
    
    return False, "Timeout - no response"

# Usage
ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)
success, response = send_command(ser, "!MOVE X100 Y50", seq=5)
```

---

## 15. Q&A - RPI4 Agent Feedback

### Q1: Sequence Number Scope - The spec says "0-255, wraps" but responses show larger numbers (e.g., seq 42). Is this a typo or do you need uint16 instead of uint8?

**A:** The implementation uses uint8 (0-255, wraps). Example sequence numbers like 42 are well within this range. The original spec was correct. Reviewed implementation at line 219: `uint8_t lastSeqNum = 0;` confirms uint8 type. The 10-command duplicate detection window (line 220) is sufficient for protocol needs with uint8 range.

### Q2: Move Completion Detection - !MOVE sends @ACK then later @COMPLETE. What happens if @COMPLETE is lost? Should master poll !STATUS to detect completion?

**A:** Yes, exactly. Added guidance in Section 6.2 (!MOVE command): If @COMPLETE is lost, master should poll !STATUS to check STATE=IDLE. Recommended approach: calculate expected move duration from (distance/speed) + 2s safety margin, then poll !STATUS if @COMPLETE not received within that timeframe. The STATE field will show IDLE when move completes.

### Q3: Expected Duration or Timeout Guidance for Moves?

**A:** Added to Section 6.2. Master should calculate expected duration as: `duration = max_axis_distance / speed + 2s_margin`. If @COMPLETE not received within this time, poll !STATUS to check for completion or error. Example: 100mm move at 50mm/s = 2s + 2s margin = 4s timeout.

### Q4: Diagnostic Mode vs Protocol Mode - Does "Protocol commands paused" mean !PING stops working? Watchdog still active during diagnostic mode?

**A:** Clarified in Section 6.5 (!DIAG_ENTER command): 
- **Protocol commands:** Paused EXCEPT for !PING, !DIAG_EXIT, !STATUS, !DIAG_CMD, and !DIAG_MENU. These essential commands remain functional.
- **Watchdog:** DISABLED during diagnostic mode (see Q7). Master does NOT need to send !PING during diagnostic mode. Watchdog automatically re-enabled when !DIAG_EXIT is called.

### Q5: Error State Recovery - !CLEAR_ERROR command is shown in examples but not documented in Section 6. Should this be added?

**A:** Fixed. Added !CLEAR_ERROR command documentation in Section 6.2, including:
- State transition: ERROR → IDLE
- Response: @ACK <seq> ERROR_CLEARED
- Error handling: ERR_INVALID_STATE if not in ERROR state
- Note: Must re-home after clearing error before moving
- Also added to command summary table in Section 14.1

### Q6: Python Implementation Hint - Threading approach?

**A:** Excellent suggestion. Recommended implementation: Use threading with queue-based architecture:
- **Thread 1 (TX):** Command queue → serial transmit, handles retries and ACK/NACK
- **Thread 2 (RX):** Serial receive → parse messages → route to appropriate queues (responses vs events)
- **Main thread:** Application logic, queues commands, consumes async events (#LIMIT, #COMM_LOST, etc.)

This cleanly separates concerns and handles async events without blocking command flow.

### Q7: Diagnostic Mode & Watchdog - Should watchdog be suspended during DIAGNOSTIC mode? Or should the diagnostic menu auto-send pings?

**A:** Watchdog is DISABLED during diagnostic mode (confirmed in implementation at line 2620: `watchdogEnabled = false`). This is the correct design choice:
- Diagnostic mode is for human interaction with unpredictable timing
- Forcing master to ping during manual testing adds complexity
- Watchdog re-enabled on !DIAG_EXIT (line 2635: `watchdogEnabled = true`)
- Safety maintained: emergency stop still functional, limits still enforced

Master does NOT need to ping during diagnostic mode.

### Q8: Position Persistence - After COMM_LOST, positions are preserved but HOMED=None. Is this the intended behavior?

**A:** Partially correct. After COMM_LOST:
- **Position counters preserved:** Step counts remain valid (motors stopped, counters not reset)
- **HOMED flags preserved:** Implementation does NOT clear xHomed/yHomed/zHomed/fHomed flags
- **However:** Master should treat positions as untrustworthy after communication loss
- **Best practice:** Master should re-home after recovery to establish trusted coordinate system

The HOMED flags technically remain set in firmware, but master should consider them invalid for safety. The example in Section 11.4 showing HOMED=None is the recommended master behavior, not actual firmware state.

### Q9: Partial Move Completion - If a move hits a limit partway through, what's the final position reported? Where it stopped or where it was commanded?

**A:** Position reported is WHERE IT STOPPED (actual position). When ERR_LIMIT_HIT occurs:
1. Motors immediately stop at current position (emergencyStop() called)
2. Slave enters ERROR state
3. @NACK sent with ERR_LIMIT_HIT (does NOT include position in NACK message)
4. **Master must query !STATUS to get actual stopped position**
5. Position reflects where motion stopped (not commanded position)

Example workflow:
```
RPi: !MOVE X200
Teensy: @ACK 6 MOVING...
... hits X_MAX limit at 145.23mm ...
Teensy: @NACK 6 ERR_LIMIT_HIT Limit switch triggered during move
RPi: !STATUS
Teensy: @STATUS 7 X=145.23 Y=50.00 STATE=ERROR
```

Critical: Unlike the example in Section 11.3, the actual implementation does NOT include position in the NACK. Master must query !STATUS separately.

### Q10: Command Queuing - Protocol says "wait for @COMPLETE before next move". Any plan to support command queuing in the future?

**A:** Not in v1.0, but listed in Section 13.1 (Future Extensions) as `!QUEUE_MOVE` command. Current API design doesn't preclude this:
- Sequence numbers already support multiple in-flight commands
- Could add QUEUE_SIZE parameter to !CONNECT negotiation
- Would need additional states: MOVING_QUEUED, queue status queries
- Buffer management and look-ahead for smooth path planning

Recommendation: Design Python API with queuing in mind (e.g., async move commands that return futures), but implement synchronously for v1.0. This allows future queuing support without API breaking changes.

---

## Changelog

### Version 1.1 (January 5, 2026)

**Position in @COMPLETE responses:**
- `!JOG` now includes full position (X, Y, Z, F) in `@COMPLETE` response
- Previously `!JOG` returned only `@COMPLETE <seq>` with no position data
- `!MOVE` already included position; documentation clarified

**Position Synchronization guidance added:**
- Master MUST wait for `@COMPLETE` before updating local position
- Using position from `@COMPLETE` prevents desync when commands are rejected (checksum errors, etc.)
- Added explicit "Position Synchronization" notes to `!MOVE` and `!JOG` sections

**Rationale:** Rapid jog commands (e.g., holding button) could cause position desync between Master and Slave when some commands failed checksum validation. By including authoritative position in `@COMPLETE` and requiring Master to wait for it, both sides stay synchronized.

**End of Protocol Specification**

*For questions or clarifications, refer to the implementation in `/teensy_firmware/src/main.cpp`*
