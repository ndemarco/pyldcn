# LS-231SE Servo Drive API Reference

**Author:** NickyDoes
**Date:** 2025-11-14
**License:** GPL v2 or later

---

## Quick Start

```python
from pyldcn import LDCNNetwork

network = LDCNNetwork('/dev/ttyUSB0')
network.open()
network.initialize()

servo = network.get_device(1, 'LS-231SE')
servo.load_gains(kp=10, kd=1000, ki=0)
servo.move_to_counts(position=5000, velocity=2000, accel=200)
servo.print_status()
```text

---

## Architecture Overview

| Layer | Component | Purpose |
|-------|-----------|---------|
| **Application** | `LS231SE` main class | High-level servo control API |
| **Subsystems** | Motion, Status, IO, Safety | Functional subsystem separation |
| **State** | `ServoState` | Single source of truth (shared state) |
| **Diagnostics** | `servo_diagnostics` | Three-layer status determination |
| **Protocol** | `protocol.py` | Low-level LDCN communication |

---

## Status Determination (Three-Layer Architecture)

| Layer | Data Type | Source | Purpose |
|-------|-----------|--------|---------|
| **Layer 1 (Primary)** | Raw bits | Status byte + Aux byte | Hardware state flags |
| **Layer 2 (Derived)** | Condition name | Pattern matching | Matched diagnostic condition |
| **Layer 3 (Metadata)** | Classification | Condition attributes | Error class, brake state, fault type |

---

## Command Reference

### Motion Commands

| Command | Method | Parameters | Description |
|---------|--------|------------|-------------|
| 0x00 | `reset_position(position=0)` | position: int | Reset position counter |
| 0x04 | `move_to_counts(pos, vel, acc)` | pos, vel, acc: int | Absolute move (counts) |
| 0x04 | `move_to(pos, vel, acc, scale)` | pos, vel, acc: float; scale: float | Absolute move (physical units) |
| 0x05 | `start_motion()` | - | Start loaded trajectory |
| 0x07 | `stop()` | - | Stop motor |
| 0x09 | `set_home_mode(limit, index, stop)` | limit: int; index: bool; stop: str | Configure homing |
| 0x0C | `save_home()` | - | Save home to EEPROM |
| 0x0D | `add_path_point(pos, vel, acc)` | pos, vel, acc: int | Add path buffer point |

### Status Commands

| Command | Method | Parameters | Returns |
|---------|--------|------------|---------|
| 0x02 | `define_status(mask)` | mask: int (16-bit) | None |
| 0x03 | `read_status()` | - | Dict (all configured items) |
| 0x03 | `read_status_custom(mask)` | mask: int | Dict (custom items) |
| 0x03 | `read_full_status()` | - | Dict (all available items) |
| 0x03 | `read_position()` | - | Dict (position only, fast) |

### I/O Commands

| Command | Method | Parameters | Description |
|---------|--------|------------|-------------|
| 0x08 | `set_brake(released)` | released: bool | Control brake (OUTbit0) |
| 0x08 | `set_output(num, state)` | num: int (0-7); state: bool | Set single output |
| 0x08 | `set_outputs(mask)` | mask: int (8-bit) | Set multiple outputs |
| - | `read_inputs()` | - | Read all input states |

### Configuration Commands

| Command | Method | Parameters | Description |
|---------|--------|------------|-------------|
| 0x06 | `load_gains(kp, kd, ki, ...)` | kp, kd, ki: int; ... | Set PID gains |
| 0x0B | `clear_faults()` | - | Clear sticky fault bits |

### Safety Commands (Extended 0x0E)

| Subcommand | Method | Parameters | Description |
|------------|--------|------------|-------------|
| 0x00 | `configure_limit_stop(action, recover)` | action: str; recover: bool | Limit stop behavior |
| 0x04 | `configure_hw_sync(enable, mode)` | enable: bool; mode: str | Hardware synchronization |
| 0x05 | `set_watchdog(timeout_ms)` | timeout_ms: int (0-65535) | Watchdog timer |
| - | `reset_watchdog()` | - | Reset watchdog (NOP) |
| - | `emergency_stop()` | - | Emergency stop |

---

## Status Byte Flags (Layer 1)

### Primary Status Byte (8 bits)

| Bit | Flag | Property | Type | Description |
|-----|------|----------|------|-------------|
| 0 | `STATUS_MOVE_DONE` | `move_done` | bool | Trapezoidal move complete |
| 1 | `STATUS_CKSUM_ERROR` | `cksum_error` | bool | Checksum error (sticky) |
| 2 | `STATUS_CURRENT_LIMIT` | `current_limit` | bool | Current limit exceeded (sticky) |
| 3 | `STATUS_POWER` | `power` | bool | Amplifier power enabled |
| 4 | `STATUS_POS_ERROR` | `pos_error_flag` | bool | Position error exceeded (sticky) |
| 5 | `STATUS_HOME_SOURCE` | `home_source` | bool | Home switch state |
| 6 | `STATUS_LIMIT2` | `limit2` | bool | Forward limit switch |
| 7 | `STATUS_HOME_IN_PROG` | `home_in_progress` | bool | Homing in progress |

### Auxiliary Status Byte (7 bits)

| Bit | Flag | Property | Type | Description |
|-----|------|----------|------|-------------|
| 0 | `AUX_INDEX` | `index` | bool | Encoder index input |
| 1 | `AUX_POS_WRAP` | `pos_wrap` | bool | Position counter wrapped (sticky) |
| 2 | `AUX_SERVO_ON` | `servo_on` | bool | Servo loop enabled |
| 3 | `AUX_ACCEL_DONE` | `accel_done` | bool | Acceleration phase complete |
| 4 | `AUX_SLEW_DONE` | `slew_done` | bool | Slew phase complete |
| 5 | `AUX_SERVO_OVERRUN` | `servo_overrun` | bool | Servo tick overrun (sticky) |
| 6 | `AUX_PATH_MODE` | `path_mode` | bool | Executing path buffer |

---

## LDCN Mode Diagnostic Conditions (Layer 2)

### Normal/Operational States

| Code | Condition | Faulted | Brake | Fault Relay | Power | Description |
|------|-----------|---------|-------|-------------|-------|-------------|
| - | No Motor Power after LDCN Init | No | Released | Open | OFF | Initial power-up state |
| - | AxisOFF | No | CN8pin9 High | Open | ON | Axis disabled, amplifier on |
| - | ServoON | No | CN8pin9 High | Open | ON | Normal operation, servo active |
| - | ServoOFF | No | CN8pin9 Low | Open | ON | Servo disabled, holding |
| - | Stopped | No | Engaged | Closed | ON | Motor stopped by command |
| - | Home Source | No | Released | Closed | OFF | Homing detected |
| - | Encoder | No | Released | Closed | ON | Encoder status |

### Fault States

| Code | Condition | Error Class | Reset Req | Brake | Description |
|------|-----------|-------------|-----------|-------|-------------|
| - | ErrHALL | FAULT | No | Engaged | Hall sensor error |
| - | ErrEEPROM | FAULT | Yes | Engaged | EEPROM checksum error |
| - | EncoderERR | FAULT | Yes | Engaged | Encoder error (hard reset required) |
| - | No Motor Power | POWER | No | Engaged | Motor power missing |
| - | Overheat | FAULT | No | Engaged | Temperature limit exceeded |
| - | Disabled | FAULT | No | Engaged | Drive disabled |
| - | Master EncoderERROR | FAULT | No | Engaged | Master encoder signal lost |
| - | Brake or Output Short | FAULT | No | Engaged | Output shorted |
| - | MotorShort | FAULT | No | Engaged | Motor windings shorted |
| - | Motor PowerDROP | FAULT | No | Engaged | Motor voltage dropped |
| - | OverLOAD | FAULT | No | Engaged | Current limit exceeded |
| - | PositionERROR | FAULT | No | Engaged | Position following error |
| - | Limit2 | FAULT | No | Engaged | Forward limit triggered |

---

## Status Item Masks (DEFINE_STATUS)

| Mask | Name | Bytes | Description |
|------|------|-------|-------------|
| 0x0001 | position | 4 | Current position (encoder counts) |
| 0x0002 | ad_value | 1 | A/D converter value (0-255) |
| 0x0004 | velocity | 2 | Current velocity (counts/tick) |
| 0x0008 | aux_status | 1 | Auxiliary status byte |
| 0x0010 | home | 4 | Captured home position |
| 0x0020 | device_id | 2 | Device ID and firmware version |
| 0x0040 | pos_error | 2 | Position following error |
| 0x0080 | path_count | 1 | Path buffer count (0-256) |
| 0x1000 | watchdog | 2 | Watchdog timer status |
| 0x2000 | motor_pos | 6 | Motor position and error |

---

## Diagnostic Query Functions

### Layer 1 - Direct Bit Access

| Function | Parameters | Returns | Description |
|----------|------------|---------|-------------|
| `is_powered(status_byte)` | status_byte: int | bool | Amplifier power enabled (bit 3) |
| `is_moving(status_byte)` | status_byte: int | bool | Motion in progress (!bit 0) |
| `is_homing(status_byte)` | status_byte: int | bool | Homing in progress (bit 7) |
| `is_servo_on(aux_byte)` | aux_byte: int | bool | Servo loop enabled (aux bit 2) |
| `is_at_limit(status_byte)` | status_byte: int | bool | At limit switch (bit 6) |
| `in_path_mode(aux_byte)` | aux_byte: int | bool | Executing path (aux bit 6) |

### Layer 2 - Condition Based

| Function | Parameters | Returns | Description |
|----------|------------|---------|-------------|
| `match_diagnostic_condition()` | status, aux, stop, pic_ae, mode | DiagnosticCondition\|None | Match condition from bits |
| `is_faulted(condition)` | condition | bool | Check if faulted |
| `needs_reset(condition)` | condition | bool | Check if hard reset required |
| `is_brake_released(condition)` | condition | bool | Check if brake released |
| `is_operational(condition)` | condition | bool | Check if ready for motion |
| `get_error_class(condition)` | condition | str\|None | Get error classification |

### Layer 3 - Comprehensive State

| Function | Parameters | Returns | Description |
|----------|------------|---------|-------------|
| `get_servo_state()` | status, aux, stop, pic_ae, mode | Dict | All 3 layers combined |

---

## Main Class API

### LS231SE Class Methods

#### Initialization

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `__init__(network, address)` | network, address: int | - | Initialize servo |
| `load_gains(kp, kd, ki, ...)` | kp, kd, ki: int; ... | None | Set PID gains |

#### Motion Control

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `move_to(pos, vel, acc, scale)` | pos, vel, acc: float; scale: float | None | Move to position (physical units) |
| `move_to_counts(pos, vel, acc)` | pos, vel, acc: int | None | Move to position (counts) |
| `stop()` | - | None | Stop motor |
| `enable()` | - | None | Enable amplifier |
| `disable()` | - | None | Disable amplifier and servo |

#### Homing

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `set_home_mode(limit, index, stop)` | limit: int\|None; index: bool; stop: str | None | Configure homing |
| `home_to_limit(limit, vel, acc, ...)` | limit, vel, acc: int; index: bool; ... | None | Complete homing sequence |

#### Status & Diagnostics

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `read_status()` | - | Dict | Read complete status |
| `read_position()` | - | Dict | Read position only (fast) |
| `is_ready(mode)` | mode: str = "LDCN" | bool | Ready for motion commands |
| `is_faulted(mode)` | mode: str = "LDCN" | bool | In faulted state |
| `get_condition(mode)` | mode: str = "LDCN" | DiagnosticCondition\|None | Get diagnostic condition |
| `print_status(include_leds, mode)` | include_leds: bool; mode: str | None | Print formatted status |

#### Fault Management

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `clear_faults()` | - | None | Clear sticky fault bits |
| `check_faults(status_byte)` | status_byte: int | List[str] | List active faults |

---

## Subsystem APIs

### Motion Subsystem (`servo._motion`)

| Method | Parameters | Description |
|--------|------------|-------------|
| `move_to(pos, vel, acc, scale)` | pos, vel, acc: float; scale: float | Move to position (physical) |
| `move_to_counts(pos, vel, acc)` | pos, vel, acc: int | Move to position (counts) |
| `reset_position(position)` | position: int = 0 | Reset position counter |
| `save_home()` | - | Save home to EEPROM |
| `add_path_point(pos, vel, acc)` | pos, vel, acc: int | Add path point |
| `start_path_mode()` | - | Start path execution |
| `clear_path_buffer()` | - | Clear path buffer |
| `get_path_count()` | - | Get remaining points |
| `set_home_mode(limit, index, stop)` | limit: int; index: bool; stop: str | Configure homing |
| `home_to_limit(limit, vel, acc, ...)` | limit, vel, acc: int; ... | Two-stage homing |
| `enable()` | - | Enable amplifier |
| `disable()` | - | Disable amplifier |

### Status Subsystem (`servo._status`)

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `read_status()` | - | Dict | Read complete status |
| `read_position()` | - | Dict | Read position only |
| `define_status(mask)` | mask: int | None | Configure status items |
| `read_status_custom(mask)` | mask: int | Dict | Read custom items |
| `read_full_status()` | - | Dict | Read all items |
| `get_condition(mode)` | mode: str | DiagnosticCondition\|None | Get diagnostic condition |
| `get_comprehensive_state(mode)` | mode: str | Dict | Get all 3 layers |
| `is_faulted(mode)` | mode: str | bool | Check if faulted |
| `is_operational(mode)` | mode: str | bool | Check if operational |
| `needs_reset(mode)` | mode: str | bool | Check if reset required |
| `format_status(include_leds, mode)` | include_leds: bool; mode: str | str | Format status report |

### I/O Subsystem (`servo._io`)

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `set_brake(released)` | released: bool | None | Control brake output |
| `set_output(num, state)` | num: int; state: bool | None | Set single output |
| `set_outputs(mask)` | mask: int | None | Set multiple outputs |
| `read_inputs()` | - | Dict | Read all inputs |
| `get_limit2_state()` | - | bool | Get limit2 state |
| `get_home_switch_state()` | - | bool | Get home switch state |

### Safety Subsystem (`servo._safety`)

| Method | Parameters | Returns | Description |
|--------|------------|---------|-------------|
| `set_watchdog(timeout_ms)` | timeout_ms: int | None | Configure watchdog |
| `reset_watchdog()` | - | None | Reset watchdog timer |
| `configure_hw_sync(enable, mode)` | enable: bool; mode: str | None | Hardware sync |
| `configure_limit_stop(action, recover)` | action: str; recover: bool | None | Limit stop behavior |
| `emergency_stop()` | - | None | Emergency stop |
| `reset_faults()` | - | None | Reset all faults |

---

## State Properties

### ServoState Attributes

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `move_done` | bool | True | Move complete |
| `cksum_error` | bool | False | Checksum error |
| `current_limit` | bool | False | Current limit (sticky) |
| `power` | bool | False | Amplifier power |
| `pos_error_flag` | bool | True | Position error (sticky) |
| `home_source` | bool | True | Home switch |
| `limit2` | bool | False | Forward limit |
| `home_in_progress` | bool | False | Homing active |
| `index` | bool | True | Index input |
| `pos_wrap` | bool | False | Position wrapped (sticky) |
| `servo_on` | bool | False | Servo loop enabled |
| `accel_done` | bool | False | Accel phase complete |
| `slew_done` | bool | False | Slew phase complete |
| `servo_overrun` | bool | False | Servo overrun (sticky) |
| `path_mode` | bool | False | Path mode active |
| `position` | int\|None | None | Current position |
| `velocity` | int\|None | None | Current velocity |
| `pos_error` | int\|None | None | Position error |
| `kp`, `kd`, `ki` | int\|None | None | PID gains |

---

## Examples Quick Reference

| Example | File | Features Demonstrated |
|---------|------|----------------------|
| **Status Monitoring** | `servo_status_example.py` | Layer 1/2/3 status, diagnostics, fault detection, monitoring |
| **Homing** | `servo_homing_example.py` | Limit homing, index homing, two-stage, save/reset home |
| **Comprehensive** | `servo_comprehensive_example.py` | All features: motion, homing, I/O, path, faults, profiling |
| **Coordinated Motion** | `servo_coordinated_motion.py` | Multi-axis, linear/circular interpolation, path mode |

---

## Common Patterns

### Initialize and Move

```python
servo = network.get_device(1, 'LS-231SE')
servo.load_gains(kp=10, kd=1000, ki=0)
servo.move_to_counts(position=5000, velocity=2000, accel=200)

# Wait for completion
while not servo.move_done:
    servo.read_status()
    time.sleep(0.05)
```text

### Two-Stage Homing

```python
servo.home_to_limit(
    limit_switch=2,
    velocity=2000,
    accel=200,
    use_index=True,
    index_velocity=500
)
servo._motion.reset_position(0)
servo._motion.save_home()
```text

### Status Monitoring

```python
servo.read_status()

# Layer 1 - Raw bits
if servo.move_done and servo.servo_on:
    print("Ready")

# Layer 2 - Condition
condition = servo.get_condition()
print(f"Condition: {condition.condition}")

# Layer 3 - Quick checks
if servo.is_faulted():
    print("FAULT DETECTED")
    servo.clear_faults()
```text

### Path Mode

```python
servo._motion.clear_path_buffer()

for i in range(10):
    delta = (100 << 8)  # 100 counts (int8.frac8)
    servo._motion.add_path_point(delta, velocity=500, accel=50)

servo._motion.start_path_mode()

# Monitor execution
while servo._motion.get_path_count() > 0:
    servo.read_status()
    time.sleep(0.1)
```text

### Custom Status Configuration

```python
# Configure status to return specific items
mask = 0x0001 | 0x0004 | 0x0008  # position, velocity, aux
servo.status.define_status(mask)

# Read configured items
status = servo.read_status()
print(f"Pos: {status['position']}, Vel: {status['velocity']}")
```text

---

## Error Handling

### Fault Types

| Error Class | Meaning | Action |
|-------------|---------|--------|
| `FAULT` | Hardware/electrical fault | Investigate cause, clear if safe |
| `ERROR` | Software/configuration error | Check configuration, reset |
| `POWER` | Power supply issue | Check motor power supply |
| `None` | Normal operation | No action needed |

### Sticky Bits (Require Clearing)

| Bit | Flag | Clear Method |
|-----|------|--------------|
| 1 | Checksum Error | `servo.clear_faults()` |
| 2 | Current Limit | `servo.clear_faults()` |
| 4 | Position Error | `servo.clear_faults()` |
| Aux 1 | Position Wrap | `servo.clear_faults()` |
| Aux 5 | Servo Overrun | `servo.clear_faults()` |

### Reset Required Conditions

| Condition | Cause | Action |
|-----------|-------|--------|
| EncoderERR | Encoder signal lost/invalid | Hard reset (power cycle or CMD 0x0F) |
| ErrEEPROM | EEPROM checksum failure | Hard reset + EEPROM reinit |

---

## Performance Considerations

| Operation | Typical Time | Notes |
|-----------|--------------|-------|
| `read_status()` | ~1-2 ms | Full status (4 items) |
| `read_position()` | ~0.5 ms | Position only (fast) |
| `move_to_counts()` | ~1 ms | Command send only |
| `load_gains()` | ~1 ms | Command send only |
| `define_status()` | ~1 ms | One-time configuration |

### Recommended Update Rates

| Operation | Rate | Reason |
|-----------|------|--------|
| Status monitoring | 10-50 Hz | Balance responsiveness vs overhead |
| Position tracking | 50-100 Hz | Smooth motion monitoring |
| Fault checking | 1-10 Hz | Periodic safety check |
| Path execution | Monitor path_count at 10 Hz | Adequate for queue management |

---

## Troubleshooting

| Symptom | Possible Cause | Check/Fix |
|---------|---------------|-----------|
| `is_ready()` returns False | Faulted state | `print_status()`, `clear_faults()` |
| Position not updating | Not reading status | Call `read_status()` regularly |
| Move doesn't start | Servo not enabled | Check `servo_on` flag, call `enable()` |
| Sticky fault persists | Fault not cleared | Call `clear_faults()` |
| Watchdog timeout | No commands sent | Increase timeout or call `reset_watchdog()` |
| Encoder error | Signal lost | Check encoder connections, hard reset |

---

## File Organization

```text
pyldcn/
├── devices/
│   ├── servo.py                    # Main LS231SE class
│   ├── servo_state.py              # Shared state container
│   ├── servo_diagnostics.py        # ⭐ NEW: Status determination (3 layers)
│   ├── servo_status.py             # Status subsystem (enhanced)
│   ├── servo_motion.py             # Motion subsystem (enhanced)
│   ├── servo_io.py                 # I/O subsystem (enhanced)
│   └── servo_safety.py             # ⭐ NEW: Safety/extended commands
│
├── protocol.py                      # LDCN protocol (command constants added)
│
examples/
├── servo_status_example.py          # ⭐ NEW: Status monitoring
├── servo_homing_example.py          # ⭐ NEW: Homing sequences
├── servo_comprehensive_example.py   # ⭐ NEW: Complete demo
└── servo_coordinated_motion.py      # ⭐ NEW: Multi-axis coordination
```text

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-11-14 | Initial implementation: diagnostics, commands, safety, examples |

---

## See Also

- [servo_diagnostics.md](servo_diagnostics.md) - Diagnostic states and LED patterns
- [servo_commands.md](servo_commands.md) - Complete command reference
- [servo_status_reporting.md](servo_status_reporting.md) - Status byte definitions
- [LS-231SE_homing.md](LS-231SE_homing.md) - Homing procedures
- LS-231SE-datasheet.txt - Complete hardware specification
