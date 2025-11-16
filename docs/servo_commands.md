# LS-231SE Servo Drive Commands

This document describes commands for the LS-231SE servo drives (and likely other Logosol servo drives).

For generic LDCN network commands, see [ldcn_protocol.md](ldcn_protocol.md).

**Reference**: LS-231SE Datasheet (Doc # 712231004 / Rev. A, 05/05/2011)

---

## Command Summary

| Cmd | Name | Data Bytes | Description |
|-----|------|------------|-------------|
| 0x0 | Reset Position | 0 | Reset 32-bit encoder counter to zero |
| 0x1 | Set Address | 2 | See [ldcn_protocol.md](ldcn_protocol.md) |
| 0x2 | Define Status | 1-2 | Configure persistent status reporting |
| 0x3 | Read Status | 1-2 | Request status data (one-time) |
| 0x4 | Load Trajectory | 1-15 | Load motion parameters (position, velocity, accel, PWM) |
| 0x5 | Start Motion | 0 | Execute previously loaded trajectory |
| 0x6 | Set Gain | 14 | Set PID gains and servo parameters |
| 0x7 | Stop Motor | 1 or 5 | Stop motor with various modes (abrupt, smooth, etc.) |
| 0x8 | I/O Control | 1 or 3 | Control brake output and set path point timing |
| 0x9 | Set Homing Mode | 1 | Configure homing capture conditions |
| 0xA | Set Baud Rate | 1 | See [ldcn_protocol.md](ldcn_protocol.md) |
| 0xB | Clear Sticky Bits | 0 | Clear latched fault status bits |
| 0xC | Save Current Position as Home | 0 | Store current position as home position |
| 0xD | Add Path Points | 0-14 | Add points to path buffer (up to 7 points per command) |
| 0xE | No Operation (NOP) | 0 | See [ldcn_protocol.md](ldcn_protocol.md) |
| 0xE | Extended Commands | 1-n | Sub-commands for advanced features (see table below) |
| 0xF | Hard Reset | 0 | See [ldcn_protocol.md](ldcn_protocol.md) |

### Extended Commands (0xE Sub-commands)

| Sub-Cmd | Name | Data Bytes | Description |
|---------|------|------------|-------------|
| 0x00 | Stop on Limit Switches | 3 | Configure automatic stop behavior when limits triggered |
| 0x01 | Read Hall Sensors | 1 | Initialize brushless motor angle from hall sensors |
| 0x02 | Repeat Last Answer | 1 | Resend last status packet (error recovery) |
| 0x04 | Hardware Sync Mode | 2 | Enable/disable multi-drive servo tick synchronization |
| 0x05 | Set Watchdog Mode | 3 | Configure communication watchdog timer (safety) |
| 0x10 | Set Motor Error Limit | 3 | Set motor position error limit (dual-loop systems) |

---

## Command Descriptions

### Reset Position

**Command:** `0x00` (CMD_RESET_POS)<br>
**Data bytes:** 0 bytes<br>
**Returns:** Yes - Standard status packet

Resets the 32-bit encoder counter to zero.

**Example**:

```text
AA 01 00 01  # Reset position on device 1
```text

**Notes**:

- The position encoder is different from the home position
- Do not issue this command during a trapezoidal move

---

### Set Address

**Command:** `0x01` (CMD_SET_ADDR)<br>
**Data bytes:** 2 bytes<br>
**Returns:** Yes - Standard status packet

See [ldcn_protocol.md](ldcn_protocol.md) for complete documentation.

**Summary**: Sets individual address (1-127) and group address (128-255).

---

### Define Status

**Command:** `0x02` (CMD_DEFINE_STATUS)<br>
**Data bytes:** 1 or 2 bytes<br>
**Returns:** Yes - Status packet with defined status items

Defines what additional data will be sent in status packets along with the status byte.

**Default**: `0x0000` (no additional status data)

**Status Configuration Bits** (Servo Devices):

| Bit | Data Item | Size | Description |
|-----|-----------|------|-------------|
| 0 | position | 4 bytes | Current encoder position (int32, little-endian) |
| 1 | ad_value | 1 byte | Analog-to-digital converter value (0-255) |
| 2 | velocity | 2 bytes | Actual velocity (int16, no fractional component) |
| 3 | auxiliary | 1 byte | Auxiliary status byte (see below) |
| 4 | home | 4 bytes | Captured home position (int32) |
| 5 | device_id | 2 bytes | Device ID (0) + Version (20-29 decimal) |
| 6 | pos_error | 2 bytes | Current position following error (int16) |
| 7 | path_count | 1 byte | Number of points in path buffer (0-255) |
| 8 | digital_in | 2 bytes | Digital input states |
| 9 | analog_in | 2 bytes | Analog input values |
| 10-11 | (reserved) | - | Clear to 0 |
| 12 | watchdog | 2 bytes | Watchdog status (0xFFFF=disabled, 0x0000=expired) |
| 13 | motor_pos | 6 bytes | Motor position and position error |
| 14-15 | (reserved) | - | Clear to 0 |

**Notes**:

- Status data is always sent in bit order (0, 1, 2, 3, ...)
- Setting bits causes corresponding data to be appended after status byte
- Power-up or `hard reset` resets to return only status byte + checksum

---

### Read Status

**Command:** `0x03` (CMD_READ_STATUS)<br>
**Data bytes:** 1 or 2 bytes<br>
**Returns:** Yes - Status packet with requested status items (one time only)

Non-permanent version of Define Status. The status packet returned includes the specified data, but subsequent packets use the previously defined configuration.

---

### Load Trajectory

**Command:** `0x04` (CMD_LOAD_TRAJ)<br>
**Data bytes:** 1 to 15 bytes<br>
**Returns:** Yes - Standard status packet

Loads a motion trajectory into the servo drive's path planner.

**Data**:

- Byte 0: Trajectory control flags
- Optional bytes based on control flags set:
  - If bit 0 set: Bytes 1-4: Position (int32, little-endian, encoder counts)
  - If bit 1 set: Next 4 bytes: Velocity (int32, counts per servo tick)
  - If bit 2 set: Next 4 bytes: Acceleration (int32, counts per tick²)
  - If bit 3 set: Next 1-2 bytes: PWM value (uint8 or uint16)

**Trajectory Control Flags**:

| Bit | Function |
|-----|----------|
| 0   | Load position data (adds 4 bytes) |
| 1   | Load velocity data (adds 4 bytes) |
| 2   | Load acceleration data (adds 4 bytes) |
| 3   | Load PWM value (adds 1-2 bytes) |
| 4   | Servo mode: 0 = PWM mode, 1 = position servo |
| 5   | Profile mode: 0 = trapezoidal, 1 = velocity |
| 6   | Direction flag: 0 = FWD, 1 = REV (velocity/PWM mode) |
| 7   | Start motion now |

**Common Control Values**:

- `0x9F` (bits 0,1,2,3,4,7): Load all params + servo mode + start now
- `0x97` (bits 0,1,2,4,7): Load pos/vel/acc + servo mode + start now
- `0x90` (bits 4,7): Servo mode + start now (no new data)

**Notes**:

- Position is absolute in encoder counts
- Velocity and acceleration values depend on scale and servo rate
- Use SCALE (e.g., 2000 counts/mm) to convert units

---

### Start Motion

**Command:** `0x05` (CMD_START_MOVE)<br>
**Data bytes:** 0 bytes<br>
**Returns:** Yes - Standard status packet

Starts previously loaded motion trajectory.

**Example**:

```text
AA 01 05 06  # Start motion on device 1
```text

**Use Case**: Execute a trajectory loaded with the Load Trajectory command (without bit 7 set). Useful for synchronized multi-axis motion - load all axes, then start them simultaneously with a group command.

---

### Load Gains

**Command:** `0x06` (CMD_LOAD_GAIN)<br>
**Data bytes:** 14 bytes<br>
**Returns:** Yes - Standard status packet

Sets PID control loop gains for the servo drive.

**Data**:

- Bytes 0-1: kp - Proportional gain (uint16)
- Bytes 2-3: kd - Derivative gain (uint16)
- Bytes 4-5: ki - Integral gain (uint16)
- Byte 6: il - Integration limit (uint8)
- Byte 7: ol - Output limit (uint8)
- Byte 8: cl - Current limit (uint8)
- Bytes 9-10: el - Position error limit (uint16)
- Byte 11: sr - Servo rate divisor (uint8)
- Byte 12: db - Deadband (uint8)

**Notes**:

- Gains must be tuned for specific motor and load
- Incorrect gains can cause instability or poor performance
- Start with conservative values and tune incrementally

---

### Stop Motor

**Command:** `0x07` (CMD_STOP_MOTOR)<br>
**Data bytes:** 1 or 5 bytes<br>
**Returns:** Yes - Standard status packet

Stops servo motor and controls amplifier enable state.

**Data**:

- Byte 0: Stop control flags
- Bytes 1-4: Stop position (optional, only if bit 4 set)

**Stop Control Flags**:

| Bit | Function |
|-----|----------|
| 0   | Pic_ae (Power Driver enable) |
| 1   | Turn motor off (disable position servo, set PWM to 0) |
| 2   | Stop abruptly (set command & goal velocity to 0, enable servo) |
| 3   | Stop smoothly (set goal velocity to 0, decelerate) |
| 4   | Stop here (move to specified position abruptly, requires 4 more data bytes) |
| 5-7 | Not used (clear to 0) |

**Common Combinations**:

- `0x05` (bits 0,2): Enable amplifier + stop abruptly → Enable drive for motion
- `0x09` (bits 0,3): Enable amplifier + stop smoothly → Graceful stop
- `0x01` (bit 0 only): Enable amplifier only → Hold current position
- `0x02` (bit 1 only): Turn motor off → Disable position servo
- `0x00`: Disable amplifier → Disable everything

**Examples**:

```text
AA 01 17 05 1D  # Enable amplifier, stop abruptly (standard init)
AA 01 17 09 21  # Enable amplifier, stop smoothly (graceful stop)
AA 01 17 01 19  # Enable amplifier only (hold position)
AA 01 17 02 1A  # Turn motor off (disable servo)
```text

**IMPORTANT**: Bit 0 (Pic_ae) must be set to enable the power driver. Drive initialization requires sending 0x05 (bits 0,2) to enable amplifier and close servo loop.

**Notes**:

- Only one of bits 1, 2, 3, or 4 should be set at the same time
- Bit 4 requires 4 additional data bytes specifying the stopping position

---

### I/O Control

**Command:** `0x08` (CMD_IO_CTRL)<br>
**Data bytes:** 1 or 3 bytes<br>
**Returns:** Yes - Standard status packet

Controls brake output and configures path point buffer timing.

**Data**:

- Byte 0: Control byte
- Bytes 1-2: Path point buffer counter (optional, only if bit 6 set)

**Control Byte**:

| Bit | Function |
|-----|----------|
| 0 | Brake output mode: 0 = automatic (status-controlled), 1 = manual (bit 1 control) |
| 1 | Brake output control (only if bit 0 = 1): 0 = brake off, 1 = brake on |
| 2-5 | Not used (must be 0) |
| 6 | Set path point buffer counter: 0 = no change, 1 = set to bytes 1-2 value |
| 7 | Not used (must be 0) |

**Path Point Buffer Counter**:

- Range: 0x0000 to 0x7FFF
- Purpose: Sets time interval between path points
- Calculation: `time_between_points = counter × 51.2 µs`
- Example: counter = 100 → 5.12 ms between points

**Notes**:

- Brake output is typically controlled automatically based on drive status
- Manual brake control useful for testing or special applications
- Path point timing must be set before executing path mode
- See "Status bits and LEDs" section in datasheet for automatic brake behavior

---

### Set Homing Mode

**Command:** `0x09` (CMD_SET_HOMING)<br>
**Data bytes:** 1 byte<br>
**Returns:** Yes - Standard status packet

Configures homing mode to capture home position on specified conditions.

**Data**:

- Byte 0: Homing control byte

**Homing Control Byte**:

| Bit | Function |
|-----|----------|
| 0   | Capture on Limit 1 (Reverse/Negative direction) |
| 1   | Capture on Limit 2 (Forward/Positive direction) |
| 2   | Turn motor off on home |
| 3   | Capture on Index pulse |
| 4   | Stop abruptly on home |
| 5   | Stop smoothly on home |
| 6   | Capture when position error exceeds limit |
| 7   | Capture when current limiting occurs |

**Important**: Set one (and only one) of bits 2, 4, or 5 for stop behavior.

**Common Control Bytes**:

- `0x11` (0b00010001): Home to Limit 1, stop abruptly
- `0x12` (0b00010010): Home to Limit 2, stop abruptly
- `0x18` (0b00011000): Home to Index, stop abruptly
- `0x21` (0b00100001): Home to Limit 1, stop smoothly
- `0x22` (0b00100010): Home to Limit 2, stop smoothly

**Homing Sequence**:

1. Set home mode with desired capture condition
2. Load velocity trajectory (use velocity mode, not position mode!)
3. Start motion (command 0x05)
4. Wait while `home_in_progress` status bit = 1
5. Home position captured when condition is met

**Example - Home to Limit 2**:

```text
AA 01 19 12 2C     # Set home mode: Limit 2 + stop abruptly
AA 01 94 36 ...    # Load velocity trajectory (forward direction)
AA 01 05 06        # Start motion
# Wait for home_in_progress bit to clear
```text

**Example - Two-Stage Homing** (Limit switch then Index pulse):

```text
AA 01 19 12 2C     # Home to Limit 2, stop abruptly
AA 01 94 36 ...    # Load velocity trajectory (forward)
AA 01 05 06        # Start motion
# Wait for home_in_progress = 0

AA 01 19 18 32     # Home to Index, stop abruptly
AA 01 94 77 ...    # Load velocity trajectory (reverse, slower)
AA 01 05 06        # Start motion
# Wait for home_in_progress = 0
```text

**Status Monitoring**:

- `home_in_progress` (status bit 7) is set when command is issued
- Bit remains 1 while searching for home condition
- Bit clears to 0 when home position is captured

**Notes**:

- Homing uses **velocity mode**, not position mode
- The motor moves continuously until the home condition is met
- Two-stage homing (limit + index) provides both speed and precision
- Always ensure soft limits are configured before homing

---

### Set Baud Rate

**Command:** `0x0A` (CMD_SET_BAUD)<br>
**Data bytes:** 1 byte<br>
**Returns:** Yes - Standard status packet

See [ldcn_protocol.md](ldcn_protocol.md) for complete documentation.

**Summary**: Configures the serial communication baud rate for the device.

---

### Clear Bits

**Command:** `0x0B` (CMD_CLEAR_BITS)<br>
**Data bytes:** 0 bytes<br>
**Returns:** Yes - Standard status packet

Clears "sticky" status bits that latch on fault conditions.

**Example**:

```text
AA 01 0B 0C  # Clear sticky bits on device 1
```text

**Sticky Bits Cleared**:

- Checksum error (bit 1)
- Current limit (bit 2)
- Position error (bit 4)
- Position wrap (auxiliary status bit 1)
- Servo overrun (auxiliary status bit 5)

**Use Case**: After recovering from a fault, clear the fault flags before resuming operation.

---

### Save Current Position as Home

**Command:** `0x0C` (CMD_SAVE_AS_HOME)<br>
**Data bytes:** 0 bytes<br>
**Returns:** Yes - Standard status packet

Saves the current encoder position as the home position.

**Example**:

```text
AA 01 0C 0D  # Save current position as home on device 1
```text

**Use Case**: Synchronous home position capture across multiple axes. This command can be issued to a group of controllers to set their current positions as home synchronously.

**Notes**:

- Stored home position can be read via Define Status/Read Status (bit 4)
- Does not move the motor - only stores the current position value
- Home position is a 32-bit signed integer

---

### Add Path Points

**Command:** `0x0D` (CMD_ADD_PATHPOINT)<br>
**Data bytes:** 0, 2, 4, 6, 8, 10, 12, or 14 bytes<br>
**Returns:** Yes - Standard status packet

Adds incremental path points to the 256-entry path buffer for continuous motion trajectories.

**Data**:

- 0 bytes: Start path execution
- 2 bytes: Add 1 path point
- 4 bytes: Add 2 path points
- 6 bytes: Add 3 path points
- 8 bytes: Add 4 path points
- 10 bytes: Add 5 path points
- 12 bytes: Add 6 path points
- 14 bytes: Add 7 path points (maximum per command)

**Data Format** (per path point, 2 bytes):

- **Format**: 16-bit signed integer (int8.frac8)
- **Byte 0**: Fractional part (1/256 of encoder count)
- **Byte 1**: Integer part (encoder counts)
- **Interpretation**: Incremental velocity applied for each path segment
- **Little-endian**: LSB first, MSB second

**Path Point Mechanics**:

1. Each 2-byte value is added to the desired position every servo tick
2. The value is applied **Path Point Buffer Counter** times (set via 0x8 command)
3. This creates a linear segment from current position to next path point
4. Multiple points create a continuous trajectory

**Buffer Capacity**: 256 path points maximum

**Timing**:

- Time per point = Path Point Buffer Counter × 51.2 µs
- Example: Counter = 100 → 5.12 ms per point
- Must be set via I/O Control (0x8) command before path execution

**Status Monitoring**:

- Use Status bit 7 (path_count) to monitor buffer usage
- Use Auxiliary Status bit 6 (path_mode) to check if path is executing
- Buffer refill when path_count drops below threshold

**Notes**:

- Path buffer holds 256 points total
- Points are consumed at the rate set by Path Point Buffer Counter
- Servo must be enabled before starting path execution
- Path mode stops when buffer empties or Stop Motor/Load Trajectory command sent
- Each point defines an incremental velocity, not absolute position
- Fractional component (1/256 count) allows smooth motion at slow speeds
- For multi-axis coordination, use group commands to start paths simultaneously

---

### No Operation (NOP)

**Command:** `0x0E` (CMD_NOP)<br>
**Data bytes:** 0 bytes<br>
**Returns:** Yes - Status packet according to Define Status configuration

See [ldcn_protocol.md](ldcn_protocol.md) for complete documentation.

**Summary**: Used to request status without executing a command.

---

### Extended Commands (0xE with sub-commands)

Advanced features accessed via command 0xE with sub-command codes.

**Command Structure**:

- **Command**: 0xE
- **Data**: 1 to n bytes
  - Byte 0: Sub-command code (0x00, 0x01, 0x02, 0x04, 0x05, 0x10)
  - Bytes 1-n: Sub-command specific data

#### Sub-command 0x00: Stop on Limit Switches

**Command:** `0x0E` (CMD_EXTENDED)<br>
**Sub-command:** `0x00`<br>
**Data bytes:** 3 bytes<br>
**Returns:** Yes - Standard status packet

Configures automatic stop behavior when limit switches are triggered.

**Data**:

- Byte 0: Sub-command code (0x00)
- Byte 1: Control byte for Limit 1 (Reverse)
- Byte 2: Control byte for Limit 2 (Forward)

**Limit Control Byte Bits**:

| Bit | Function |
|-----|----------|
| 0 | Servo in one direction only (allow motion away from limit) |
| 1 | Turn motor off (disable servo, PWM = 0) |
| 2 | Stop abruptly (servo to current position) |
| 3 | Stop smoothly (decelerate to zero velocity) |
| 4-7 | Not used (must be 0) |

**Behavior**:

- **Bit 0 set**: Position servo enabled only in direction away from limit
- **Bit 1 set**: Position servo disabled, PWM = 0 (bits 2-3 ignored)
- **Bit 2 set**: Motor servos to current position (abrupt stop)
- **Bit 3 set**: Motor decelerates smoothly to zero velocity
- **All bits 0-3 clear**: Stop on limits function disabled (default)

---

#### Sub-command 0x01: Read Hall Sensors and Initialize Angle

**Command:** `0x0E` (CMD_EXTENDED)<br>
**Sub-command:** `0x01`<br>
**Data bytes:** 1 byte<br>
**Returns:** Yes - Standard status packet

Reads hall sensor state and calculates initial motor angle for brushless motors.

**Data**:

- Byte 0: Sub-command code (0x01)

**Description**:

- Reads current hall sensor inputs
- Calculates initial rotor angle
- Angle will be overwritten when first index pulse arrives

**Use Case**: Brushless motor commutation initialization

---

#### Sub-command 0x02: Repeat Last Answer

**Command:** `0x0E` (CMD_EXTENDED)<br>
**Sub-command:** `0x02`<br>
**Data bytes:** 1 byte<br>
**Returns:** Yes - Repeats last status packet sent

Requests the drive to resend its last status response.

**Data**:

- Byte 0: Sub-command code (0x02)

**Description**:

- Drive resends the most recent status packet
- Useful for recovering from communication errors without re-executing command

**Use Case**: Communication error recovery, status verification

---

#### Sub-command 0x04: Enable/Disable Hardware Synchronization Mode

**Command:** `0x0E` (CMD_EXTENDED)<br>
**Sub-command:** `0x04`<br>
**Data bytes:** 2 bytes<br>
**Returns:** Yes - Standard status packet

Synchronizes servo ticks across multiple drives via hardware sync lines.

**Data**:

- Byte 0: Sub-command code (0x04)
- Byte 1: Mode (0 = disable, 1 = enable)

**Description**:

- Eliminates velocity differences caused by oscillator drift
- Multiple LS-231SE drives connected via hardware sync lines
- All drives run servo ticks in perfect synchronization
- Reduces timing errors in coordinated multi-axis motion

**Timing Error Reduction**:

- Without hardware sync: ~10 ppm oscillator drift accumulates over time
- With hardware sync: Only ±25 µs start time variation remains

**Hardware Requirements**:

- Physical sync connections between drives
- See datasheet for sync line wiring

**Use Case**: Precision multi-axis coordinated motion (CNC, robotics)

---

#### Sub-command 0x05: Set Watchdog Mode

**Command:** `0x0E` (CMD_EXTENDED)<br>
**Sub-command:** `0x05`<br>
**Data bytes:** 3 bytes<br>
**Returns:** Yes - Standard status packet

Configures watchdog timer for communication fault detection.

**Data**:

- Byte 0: Sub-command code (0x05)
- Byte 1: Mode
  - 0 = Watchdog off
  - 1 = Disable amplifier on timeout
  - 2 = Stop smoothly and disable amplifier
  - 3 = Stop smoothly (keep amplifier enabled)
- Byte 2: Timeout (in multiples of 8192 µs = 8.192 ms)

**Description**:

- Watchdog refreshed by any command sent to drive
- Upon timeout, executes configured action
- Drive stops executing motion commands after timeout
- Send this command again to reset watchdog

**Watchdog Status** (via Define Status bit 12):

- `0xFFFF` (65535): Watchdog not activated
- `0x0000` (0): Watchdog expired
- Other value: Remaining time in multiples of 8192 µs

**Timeout Calculation**:

- Timeout = byte_2 × 8.192 ms
- Example: byte_2 = 122 → ~1000 ms (1 second)

**Use Case**: Safety interlock, detect lost communication

---

#### Sub-command 0x10: Set Motor Error Limit

**Command:** `0x0E` (CMD_EXTENDED)<br>
**Sub-command:** `0x10`<br>
**Data bytes:** 3 bytes<br>
**Returns:** Yes - Standard status packet

Sets the motor position error limit for dual-loop control systems.

**Data**:

- Byte 0: Sub-command code (0x10)
- Bytes 1-2: Motor error limit (16-bit, little-endian)

**Description**:

- Used in dual-loop mode (encoder on load + encoder on motor)
- After power-up: motor error limit = master error limit
- Set Gain command also resets motor error limit to master error limit
- This command independently sets motor error limit

**Use Case**: Dual-loop servo systems with separate motor and load encoders

---

### Hard Reset

**Command:** `0x0F` (CMD_HARD_RESET)<br>
**Data bytes:** 1 byte<br>
**Returns:** No - Device resets immediately

See [ldcn_protocol.md](ldcn_protocol.md) for complete documentation.

**Summary**: Performs a complete reset of the device.

---

## Servo Status Byte

The status byte returned by servo drives has the following bit definitions:

| Bit | Name | Description |
|-----|------|-------------|
| 0   | move_done | Clear during trapezoidal move or velocity acceleration, set otherwise (including when servo disabled) |
| 1   | cksum_error | Checksum error in received command packet |
| 2   | current_limit | Current limiting exceeded (sticky - clear with Clear Bits command) |
| **3** | **power_on/diag** | **Amplifier power enabled or diagnostic bit** |
| 4   | pos_error | Position error exceeded limit (sticky - clear with Clear Bits command). Also set when servo disabled (power_on=0) |
| 5   | home_source/diag | Home switch input state or diagnostic bit |
| 6   | limit2/diag | Forward limit switch or diagnostic bit |
| 7   | home_in_progress | Set while searching for home position, cleared when home captured |

**Fault Conditions** (sticky bits - must be cleared with Clear Bits command):

- Bit 1: Checksum error - resend command
- Bit 2: Current limit - reduce load or check motor, then clear
- Bit 4: Position error - motor stalled or load too high, resolve issue then clear

**Power Detection**:

- Bit 3 = 1: Amplifier power is ON
- Bit 3 = 0: Amplifier power is OFF

**Notes**:

- Sticky bits remain set until explicitly cleared with Clear Bits (0x0B) command
- Bits 3, 5, 6 may function as diagnostic bits (see LS-231SE Diagnostic and I/O section)

---

## Auxiliary Status Byte

When configured via Define Status (bit 3), an auxiliary status byte is returned:

| Bit | Name | Description |
|-----|------|-------------|
| 0   | index/diag | Complement of index input or diagnostic bit |
| 1   | pos_wrap | 32-bit position counter wrapped (sticky - clear with Clear Bits command) |
| 2   | servo_on | Position servo loop enabled |
| 3   | accel_done | Acceleration phase of trapezoidal move complete, cleared on next move |
| 4   | slew_done | Constant velocity phase of trapezoidal move complete, cleared on next move |
| 5   | servo_overrun | Servo calculation exceeded 51.2µs (sticky - clear with Clear Bits command) |
| 6   | path_mode | Currently executing a path (cleared when buffer empty or Load Trajectory/Stop Motor sent) |
| 7   | (unused) | Not defined in datasheet |

**Notes**:

- Bit 0 may function as diagnostic bit (see LS-231SE Diagnostic and I/O section)
- Sticky bits (1, 5) remain set until cleared with Clear Bits (0x0B) command
- On power-up/reset: pos_wrap, servo_on, accel_done, slew_done, servo_overrun all clear to 0

---

## Servo Initialization Sequence

Complete 7-step initialization sequence for servo drives:

1. Define status reporting
2. Set PID gains (KP, KD, KI, IL, OL, CL, EL, SR, DB)
3. Load initial trajectory (position 0, minimal acceleration)
4. Enable amplifier and close servo loop
5. Reset position counter
6. Clear sticky status bits
7. Read and verify status

---

## Position Scaling

Convert between physical units and encoder counts using a scale factor.

**Common Scales**:

- Direct-drive: 2000-10000 counts/mm
- Ballscrew (5mm pitch): 4000 counts/rev → 800 counts/mm
- Ballscrew (10mm pitch): 4000 counts/rev → 400 counts/mm

---

## References

- Logosol LS-231SE Datasheet (Doc # 712231004)
- `utils/test_servo_init.py` - Working initialization code
- `utils/test_position_command.py` - Motion command examples
