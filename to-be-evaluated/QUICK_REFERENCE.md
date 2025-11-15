# Quick Reference

## Basic Usage

```python
from pyldcn import LDCNNetwork

# Connect and initialize
network = LDCNNetwork('/dev/ttyUSB0')
network.open()
network.initialize()
network.set_baud_rate(125000)

# Get servo device
servo = network.get_device(1, 'LS-231SE')

# Load PID gains
servo.load_gains(kp=10, kd=1000, ki=0)

# Enable and move
servo.enable()
servo.move_to_counts(position=1000, velocity=500, accel=100)

# Monitor status
servo.read_status()
print(f"Position: {servo.position}")
print(f"Velocity: {servo.velocity}")
print(f"Move done: {servo.move_done}")

# Check diagnostics
condition = servo.get_condition()
print(f"Condition: {condition.condition}")
print(f"Faulted: {servo.is_faulted()}")

# Stop and cleanup
servo.stop()
servo.disable()
network.close()
```

## Three-Layer Status Access

```python
# Layer 1: Raw bits
status_byte = servo.state.status_byte  # 0x70
power = servo.power                     # False
servo_on = servo.servo_on              # False

# Layer 2: Diagnostic condition
condition = servo.get_condition()
print(condition.condition)              # "Home Source"
print(condition.mode)                   # "LDCN"
print(condition.brake_state)            # "Released"

# Layer 3: Metadata
is_faulted = servo.is_faulted()        # False
is_ready = servo.is_ready()            # False
```

## DEFINE_STATUS Configuration

```python
# Position only
servo.status.define_status(0x0001)
status = servo.read_status()  # Returns: status, position

# Position + Velocity
servo.status.define_status(0x0005)
status = servo.read_status()  # Returns: status, position, velocity

# Full status
servo.status.define_status(0x000F)
status = servo.read_status()  # Returns: all items

# One-shot custom read (doesn't change config)
status = servo.status.read_status_custom(0x0001)
```

## Direct Subsystem Access

```python
# Status subsystem
servo.status.define_status(0x0001)
servo.status.read_full_status()
condition = servo.status.get_condition()

# Motion subsystem
servo.motion.reset_position(0)
servo.motion.enable()
servo.motion.stop()
servo.motion.save_home()

# I/O subsystem
servo.io.set_brake(released=True)
servo.io.set_output(0, True)
limits = servo.io.read_limit_switches()
```

## Diagnostic Conditions (20 total)

**LDCN Mode (most common):**
- "No Motor Power after LDCN Init" - Normal power-up state
- "Home Source" - Homing in progress or at home
- "ServoON" - Servo loop active, ready for motion
- "ServoOFF" - Servo disabled but powered
- "Fault" - Error condition, requires reset

**See servo_diagnostics.py for complete list**

## Common Properties

```python
servo.position          # Current position (counts)
servo.velocity          # Current velocity (counts/tick)
servo.pos_error         # Following error (counts)
servo.move_done         # Move complete flag
servo.power             # Amplifier powered
servo.servo_on          # Servo loop active
servo.current_limit     # Current limit exceeded (sticky)
servo.home_in_progress  # Homing active
servo.kp, servo.kd, servo.ki  # PID gains
```

## Test Execution

```bash
# Run all tests
pytest tests/ -v

# Run specific level
pytest tests/test_servo_basic.py -v    # Level 1: No motion
pytest tests/test_servo_status.py -v   # Level 2: Status config
pytest tests/test_servo_motion.py -v   # Level 3: Small moves

# Run with output
pytest tests/test_servo_basic.py -v -s
```

## File Locations

```
New implementations:
  pyldcn/devices/servo_diagnostics.py  # 865 lines
  pyldcn/devices/servo_safety.py       # 184 lines
  tests/test_servo_basic.py            # 408 lines
  tests/test_servo_status.py           # 300 lines
  tests/test_servo_motion.py           # 403 lines
  docs/servo_api_reference.md          # 535 lines

Enhanced files:
  pyldcn/devices/servo.py              # +60 lines
  pyldcn/devices/servo_status.py       # +175 lines
  pyldcn/devices/servo_motion.py       # +114 lines
  pyldcn/devices/servo_io.py           # +77 lines
  pyldcn/network.py                    # +45 lines
```
