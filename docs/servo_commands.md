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

**Data**:
- Byte 0: Trajectory control flags
- Bytes 1-4: Position (int32, little-endian, encoder counts)
- Bytes 5-8: Velocity (int32, counts per servo tick)
- Bytes 9-12: Acceleration (int32, counts per tick²)

**Trajectory Control Flags**:
| Bit | Function |
|-----|----------|
| 4   | Start immediately (servo_mode = 1) |
| 7   | Start now (start_now = 1) |

**Common Control Values**:
- `0x90` (0x80 | 0x10): Start now + servo mode

**Example**:
```python
# Move to position 2000 counts
traj_ctrl = 0x90  # start_now=1, servo_mode=1
position = 2000
velocity = 100000
accel = 50000
data = struct.pack('<Biii', traj_ctrl, position, velocity, accel)
```

**Notes**:
- Position is absolute in encoder counts
- Velocity and acceleration values depend on scale and servo rate
- Use SCALE (e.g., 2000 counts/mm) to convert units

---

### 0x5 - Start Motion

Starts previously loaded motion trajectory.

**Data**: TBD (not fully documented in current code)

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
- Byte 0: Stop mode flags

**Stop Mode Flags**:
| Bit | Function |
|-----|----------|
| 0   | Stop abruptly |
| 1   | Stop smoothly |
| 2   | Motor off (disable position servo) |
| 4   | Amplifier enable |

**Common Combinations**:
- `0x05` (STOP_ABRUPT | AMP_ENABLE): Enable amplifier, ready to move
- `0x04` (AMP_ENABLE only): Amplifier enabled, holding position
- `0x01` (STOP_ABRUPT): Stop motion immediately
- `0x00`: Disable everything

**Examples**:
```
AA 01 17 05 1D  # Enable amplifier, stop abruptly (enable drive)
AA 01 17 04 1C  # Motor off (disable drive)
```

**Use Case**: Drive initialization requires sending 0x05 to enable amplifier.

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
    traj_ctrl = 0x90  # start_now + servo_mode
    traj_data = struct.pack('<Biii', traj_ctrl, 0, 0, 0)
    send_command(addr, 0x04, list(traj_data))

    # Step 4: Enable amplifier
    send_command(addr, 0x07, [0x05])  # STOP_ABRUPT | AMP_ENABLE

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
