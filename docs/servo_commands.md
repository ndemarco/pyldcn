# LS-231SE Servo Drive Commands

This document describes commands specific to the **LS-231SE servo drives** (and likely other Logosol servo drives).

For generic LDCN network commands (Set Address, Define Status, NOP, etc.), see `LDCN_PROTOCOL.md`.

---

## Servo-Specific Commands

### 0x0 - Reset Position

Resets the 32-bit encoder counter to zero.

**Data**: None

**Example**:
```
AA 01 00 01  # Reset position on device 1
```

**Use Case**: Zeroing the machine coordinate system at a known position.

---

### 0x4 - Load Trajectory

Loads a motion trajectory into the servo drive's path planner.

**Data** (variable length, 1-15 bytes):
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

**Examples**:
```python
# Example 1: Load all trajectory parameters and start immediately
traj_ctrl = 0x97  # bits 0,1,2,4,7: Load pos+vel+acc, servo mode, start now
position = 2000
velocity = 100000
accel = 50000
data = struct.pack('<Biii', traj_ctrl, position, velocity, accel)
send_command(addr, 0x04, list(data))

# Example 2: Load position only and start immediately
traj_ctrl = 0x91  # bits 0,4,7: Load position, servo mode, start now
position = 5000
data = struct.pack('<Bi', traj_ctrl, position)
send_command(addr, 0x04, list(data))

# Example 3: Start motion with previously loaded parameters
traj_ctrl = 0x90  # bits 4,7: Servo mode, start now (no new data)
send_command(addr, 0x04, [traj_ctrl])
```

**Notes**:
- Position is absolute in encoder counts
- Velocity and acceleration values depend on scale and servo rate
- Use SCALE (e.g., 2000 counts/mm) to convert units

---

### 0x5 - Start Motion

Starts previously loaded motion trajectory.

**Data**: None

**Example**:
```
AA 01 05 06  # Start motion on device 1
```

**Use Case**: Execute a trajectory loaded with the Load Trajectory command (without bit 7 set). Useful for synchronized multi-axis motion - load all axes, then start them simultaneously with a group command.

---

### 0x6 - Load Gains

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

**Typical Values** (from test_servo_init.py):
```python
kp = 2       # Position gain
kd = 50      # Velocity gain
ki = 0       # Integral gain
il = 40      # Integration limit
ol = 255     # Output limit
cl = 0       # Current limit (0 = disabled)
el = 2000    # Position error limit (counts)
sr = 20      # Servo rate divisor (51.2µs * 20 = 1.024ms)
db = 0       # Deadband
```

**Example**:
```python
gain_data = struct.pack('<HHHBBBHBB', kp, kd, ki, il, ol, cl, el, sr, db)
send_command(addr, 0x06, list(gain_data))
```

**Notes**:
- Gains must be tuned for specific motor and load
- Incorrect gains can cause instability or poor performance
- Start with conservative values and tune incrementally

---

### 0x7 - Stop Motor

Stops servo motor and controls amplifier enable state.

**Data**:
- Byte 0: Stop control flags

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
```
AA 01 17 05 1D  # Enable amplifier, stop abruptly (standard init)
AA 01 17 09 21  # Enable amplifier, stop smoothly (graceful stop)
AA 01 17 01 19  # Enable amplifier only (hold position)
AA 01 17 02 1A  # Turn motor off (disable servo)
```

**IMPORTANT**: Bit 0 (Pic_ae) must be set to enable the power driver. Drive initialization requires sending 0x05 (bits 0,2) to enable amplifier and close servo loop.

**Notes**:
- Only one of bits 1, 2, 3, or 4 should be set at the same time
- Bit 4 requires 4 additional data bytes specifying the stopping position

---

### 0x9 - Set Home Mode

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
```
AA 01 19 12 2C     # Set home mode: Limit 2 + stop abruptly
AA 01 94 36 ...    # Load velocity trajectory (forward direction)
AA 01 05 06        # Start motion
# Wait for home_in_progress bit to clear
```

**Example - Two-Stage Homing** (Limit switch then Index pulse):
```
AA 01 19 12 2C     # Home to Limit 2, stop abruptly
AA 01 94 36 ...    # Load velocity trajectory (forward)
AA 01 05 06        # Start motion
# Wait for home_in_progress = 0

AA 01 19 18 32     # Home to Index, stop abruptly
AA 01 94 77 ...    # Load velocity trajectory (reverse, slower)
AA 01 05 06        # Start motion
# Wait for home_in_progress = 0
```

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

### 0xB - Clear Bits

Clears "sticky" status bits that latch on fault conditions.

**Data**: None

**Example**:
```
AA 01 0B 0C  # Clear sticky bits on device 1
```

**Sticky Bits Cleared**:
- Checksum error (bit 1)
- Current limit (bit 2)
- Position error (bit 4)

**Use Case**: After recovering from a fault, clear the fault flags before resuming operation.

---

## Servo Status Byte

The status byte returned by servo drives has the following bit definitions:

| Bit | Name | Description |
|-----|------|-------------|
| 0   | move_done | Clear during motion, set when motion complete |
| 1   | cksum_error | Checksum error in received command packet |
| 2   | current_limit | Current limiting active (motor overload) |
| **3** | **power_on** | **Amplifier power enabled** |
| 4   | pos_error | Position error exceeded limit (following error) |
| 5   | home_source | Home switch input state or diagnostic bit |
| 6   | limit2 | Forward limit switch or diagnostic bit |
| 7   | home_in_progress | Currently searching for home position |

**Fault Conditions** (bits that indicate problems):
- Bit 1: Checksum error - resend command
- Bit 2: Current limit - reduce load or check motor
- Bit 4: Position error - motor stalled or load too high

**Power Detection**:
- Bit 3 = 1: Amplifier power is ON
- Bit 3 = 0: Amplifier power is OFF

---

## Auxiliary Status Byte

When configured via Define Status (bit 3), an auxiliary status byte is returned:

| Bit | Name | Description |
|-----|------|-------------|
| 0   | index | Complement of index input |
| 1   | pos_wrap | 32-bit position counter wrapped |
| 2   | servo_on | Position servo loop enabled |
| 3   | accel_done | Acceleration phase complete |
| 4   | slew_done | Constant velocity phase complete |
| 5   | servo_overrun | Servo calculation exceeded time budget |
| 6   | path_mode | Executing path trajectory |

---

## Servo Initialization Sequence

Complete 7-step initialization sequence for servo drives:

```python
def initialize_servo(addr):
    # Step 1: Define status reporting
    status_bits = 0x01 | 0x04 | 0x08 | 0x40  # pos, vel, aux, pos_err
    send_command(addr, 0x02, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

    # Step 2: Set PID gains
    kp, kd, ki = 2, 50, 0
    il, ol, cl = 40, 255, 0
    el, sr, db = 2000, 20, 0
    gain_data = struct.pack('<HHHBBBHBB', kp, kd, ki, il, ol, cl, el, sr, db)
    send_command(addr, 0x06, list(gain_data))

    # Step 3: Load initial trajectory (position 0)
    # Use 0x9F to load all params: pos, vel, acc, PWM, servo mode, start now
    traj_ctrl = 0x9F  # bits 0,1,2,3,4,7
    traj_data = struct.pack('<Biiii', traj_ctrl, 0, 0, 1, 0)  # pos=0, vel=0, acc=1, pwm=0
    send_command(addr, 0x04, list(traj_data))

    # Step 4: Enable amplifier and close servo loop
    send_command(addr, 0x07, [0x05])  # Pic_ae (bit 0) + Stop abruptly (bit 2)

    # Step 5: Reset position counter
    send_command(addr, 0x00, [])

    # Step 6: Clear sticky status bits
    send_command(addr, 0x0B, [])

    # Step 7: Read and verify status
    response = send_command(addr, 0x0E, [])  # NOP to read status
    return parse_status(response)
```

---

## Position Scaling

Convert between physical units and encoder counts:

```python
SCALE = 2000.0  # counts per mm (example)

# Physical to counts
position_counts = position_mm * SCALE

# Counts to physical
position_mm = position_counts / SCALE
```

**Common Scales**:
- Direct-drive: 2000-10000 counts/mm
- Ballscrew (5mm pitch): 4000 counts/rev → 800 counts/mm
- Ballscrew (10mm pitch): 4000 counts/rev → 400 counts/mm

---

## References

- Logosol LS-231SE Datasheet (Doc # 712231004)
- `utils/test_servo_init.py` - Working initialization code
- `utils/test_position_command.py` - Motion command examples
