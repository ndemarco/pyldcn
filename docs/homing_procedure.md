# LS-231SE Homing Procedure

## Overview

The LS-231SE servo drive supports automated homing sequences using the SET_HOME_MODE command (0x09) combined with velocity-mode motion.

## SET_HOME_MODE Command (0x09)

**Command:** `0x09`
**Data bytes:** 1 byte (homing control byte)
**Command packet:** `AA <addr> 19 <control_byte> <checksum>`

### Homing Control Byte

| Bit | Function |
|-----|----------|
| 0   | Capture home position on change of Limit 1 (Reverse/Negative) |
| 1   | Capture home position on change of Limit 2 (Forward/Positive) |
| 2   | Turn motor off on home |
| 3   | Capture home on change of Index pulse |
| 4   | Stop abruptly on home |
| 5   | Stop smoothly on home |
| 6   | Capture home when position error exceeds limit |
| 7   | Capture home when current limiting occurs |

**Important:** Set one (and only one) of bits 2, 4, or 5 to specify stop behavior.

### Common Control Byte Values

| Value | Bits | Meaning |
|-------|------|---------|
| 0x11  | 0,4  | Home to Limit 1, stop abruptly |
| 0x12  | 1,4  | Home to Limit 2, stop abruptly |
| 0x19  | 0,3,4 | Home to Limit 1, then Index, stop abruptly |
| 0x1A  | 1,3,4 | Home to Limit 2, then Index, stop abruptly |
| 0x21  | 0,5  | Home to Limit 1, stop smoothly |
| 0x22  | 1,5  | Home to Limit 2, stop smoothly |

## Complete Homing Sequence

### Example: Home to Limit 2 (Forward direction)

```python
# 1. Set gains (if not already configured)
send_command(addr, 0x06, gain_data)

# 2. Close servo loop
send_command(addr, 0x07, [0x09])  # Stop smoothly + amp enable

# 3. Set home mode - capture on Limit 2, stop abruptly
send_command(addr, 0x09, [0x12])  # Bits 1,4

# 4. Load trajectory in VELOCITY mode (forward direction)
# Control byte: 0x36 = 0b00110110
#   Bit 1: Load velocity
#   Bit 2: Load acceleration
#   Bit 4: Servo mode
#   Bit 5: Velocity profile mode
# Position field is used as distance/direction in velocity mode
traj_ctrl = 0x36
velocity = 40000    # counts per servo tick (adjust for your system)
accel = 10000       # counts per tick²
position = 0        # Not used in velocity mode
traj_data = struct.pack('<Biii', traj_ctrl, position, velocity, accel)
send_command(addr, 0x04, list(traj_data))

# 5. Start motion
send_command(addr, 0x05, [])

# 6. Wait for homing to complete
while True:
    status = read_status(addr)
    if not status['home_in_progress']:
        break
    time.sleep(0.05)

print("Homed to Limit 2")

# 7. Optional: Fine-tune with index pulse
send_command(addr, 0x09, [0x18])  # Bits 3,4: Index + stop abruptly

# Load trajectory in reverse direction
traj_ctrl = 0x77  # Reverse direction
traj_data = struct.pack('<Biii', traj_ctrl, 0, velocity_slow, accel)
send_command(addr, 0x04, list(traj_data))

send_command(addr, 0x05, [])  # Start motion

# Wait for index capture
while True:
    status = read_status(addr)
    if not status['home_in_progress']:
        break
    time.sleep(0.05)

print("Homed to Index pulse")
```

## Velocity Mode for Homing

**Important:** Homing uses **velocity mode**, not position mode!

In velocity mode:
- The motor moves continuously at the specified velocity
- The `position` field in LOAD_TRAJECTORY is ignored (set to 0)
- Direction is set by bit 6 of the trajectory control byte:
  - Bit 6 = 0: Forward direction
  - Bit 6 = 1: Reverse direction
- Motion continues until:
  - Home condition is triggered (limit switch, index, etc.)
  - STOP_MOTOR command is sent
  - Position error or current limit (if configured in SET_HOME_MODE)

### Trajectory Control Byte for Velocity Mode

| Bit | Function |
|-----|----------|
| 0   | Load position (ignored in velocity mode) |
| 1   | Load velocity |
| 2   | Load acceleration |
| 3   | Load PWM |
| 4   | Servo mode (1 = closed loop) |
| 5   | Profile mode (1 = velocity mode, 0 = position mode) |
| 6   | Direction (0 = forward, 1 = reverse) |
| 7   | Start now |

**Common values:**
- `0x36` (0b00110110): Velocity mode, forward, load vel+acc
- `0x76` (0b01110110): Velocity mode, reverse, load vel+acc
- `0x37` (0b00110111): Velocity mode, forward, load all
- `0x77` (0b01110111): Velocity mode, reverse, load all

## Status Monitoring

Monitor the `home_in_progress` bit (bit 7) in the status byte:
- Set to 1 when SET_HOME_MODE command is issued
- Remains 1 while searching for home condition
- Clears to 0 when home position is captured

## Safety Considerations

1. **Always set soft limits** before homing to prevent runaway
2. **Monitor position error** to detect mechanical problems
3. **Use appropriate velocities** - slower is safer
4. **Test with covers open** (test mode) first
5. **Ensure limit switches are properly wired** and functioning

## Two-Stage Homing (Recommended)

For precise homing:

1. **Coarse home:** Move to limit switch at moderate speed
2. **Fine home:** Move slowly back to index pulse for repeatable position

This provides:
- Speed: Fast approach to limit
- Precision: Index pulse gives sub-micron repeatability
- Reliability: Limit switch provides absolute reference

## References

- LS-231SE Manual, Section "Set Homing Mode" (Command 0x09)
- LS-231SE Manual, Section "Procedure FindHomePosition" (page 51)
- LS-231SE Manual, Section "Load Trajectory" for velocity mode details

## Example from Manual

The manual's FindHomePosition procedure:

```
AA 01 19 12 2C     # Set home mode: Limit 2 + stop abruptly (0x12)
AA 01 94 36...     # Load velocity trajectory (forward)
AA 01 05 06        # Start motion
                   # Wait for home_in_progress = 0
AA 01 19 18 32     # Set home mode: Index + stop abruptly (0x18)
AA 01 94 77...     # Load velocity trajectory (reverse)
AA 01 05 06        # Start motion
                   # Wait for home_in_progress = 0
```

This homes to Limit 2, then backs off to the nearest index pulse.
