# Implementation Details

## Architecture

### Three-Layer Diagnostic System

Modeled after SK-2310g2 implementation:

**Layer 1: Primary Bits**
- Direct access to status byte and aux byte flags
- Properties: `power`, `servo_on`, `move_done`, `current_limit`, etc.

**Layer 2: Derived Condition**
- Pattern matching against diagnostic table (20 conditions)
- Supports 'X' (don't-care) bits in patterns
- Returns `DiagnosticCondition` NamedTuple

**Layer 3: Metadata**
- High-level checks: `is_faulted()`, `is_operational()`
- Error classification, brake state, LED states

### Subsystem Architecture

Four subsystems share single `ServoState` object:

```python
LS231SE
├── _status (Status)    # Read status, diagnostics, DEFINE_STATUS
├── _motion (Motion)    # Moves, homing, position reset, enable/disable
├── _io (IO)            # Brake, digital I/O, limit switches
└── _safety (Safety)    # Watchdog, hw sync, limit stop (future)
```

Subsystems exposed as properties for advanced access:
```python
servo.status.define_status(0x0001)  # Direct subsystem access
servo.read_status()                  # Convenience method
```

## Key Implementations

### 1. Diagnostic Pattern Matching (servo_diagnostics.py)

**20 LDCN diagnostic conditions** defined with bit patterns:
```python
DiagnosticCondition(
    condition="No Motor Power after LDCN Init",
    mode="LDCN",
    bit_pattern={'limit2': 0, 'home_source': 1, 'power': 0, ...},
    is_faulted=False,
    brake_state="Released",
    ...
)
```

**Pattern matching with wildcards:**
```python
def _matches_pattern(actual_bits, pattern):
    for key, expected in pattern.items():
        if expected == 'X':  # Don't care
            continue
        if actual_bits[key] != bool(expected):
            return False
    return True
```

### 2. DEFINE_STATUS Implementation (servo_status.py)

**Variable-length status configuration:**
```python
def define_status(self, status_mask: int):
    """Configure which items READ_STATUS returns."""
    self._device.send_command(CMD_DEFINE_STATUS,
                             [mask & 0xFF, (mask >> 8) & 0xFF])

def read_status_custom(self, status_mask: int):
    """One-shot custom read without changing config."""
    response = self._device.send_command(CMD_READ_STATUS,
                                        [mask & 0xFF, (mask >> 8) & 0xFF])
    return self._parse_status(response, status_mask)
```

**Status masks:**
- 0x0001: Position only
- 0x0004: Velocity only
- 0x0008: Aux status only
- Combinations supported (bitwise OR)

### 3. Motion Control (servo_motion.py)

**Position reset:**
```python
def reset_position(self, position: int = 0):
    """Reset position counter without moving motor."""
    pos_bytes = [
        position & 0xFF,
        (position >> 8) & 0xFF,
        (position >> 16) & 0xFF,
        (position >> 24) & 0xFF
    ]
    self._device.send_command(CMD_RESET_POS, pos_bytes)
    self._state.position = position
```

**Stop command:**
```python
def stop(self, smooth: bool = False):
    """Stop motor without disabling amplifier."""
    if smooth:
        stop_ctrl = STOP_SMOOTH | AMP_ENABLE
    else:
        stop_ctrl = STOP_ABRUPT | AMP_ENABLE
    self._device.send_command(CMD_STOP_MOTOR, [stop_ctrl])
```

**Enable/disable:**
```python
def enable(self):
    stop_ctrl = STOP_ABRUPT | AMP_ENABLE
    self._device.send_command(CMD_STOP_MOTOR, [stop_ctrl])

def disable(self):
    self._device.send_command(CMD_STOP_MOTOR, [0x00])
```

### 4. Convenience Methods (servo.py)

**Simplified gain loading:**
```python
def load_gains(self, kp, kd, ki,
               il=255, ol=255, cl=255,
               el=16000, sr=1, db=0):
    """Load PID gains with defaults for testing."""
    self.set_gains(kp, kd, ki, il, ol, cl, el, sr, db)
```

**Device access helper:**
```python
def get_device(self, address: int, device_type: str):
    """Get or create device at address."""
    # Check existing devices first
    # Create new device if needed
    # Support 'LS-231SE' and 'SK-2310g2'
```

## Command Organization Fix

**Problem:** Initially placed servo-specific commands in `protocol.py`

**Solution:** Separated generic vs device-specific commands:

**protocol.py (generic LDCN):**
- CMD_SET_ADDRESS
- CMD_DEFINE_STATUS
- CMD_READ_STATUS
- CMD_SET_BAUD
- CMD_NOP
- CMD_HARD_RESET

**servo.py (LS-231SE specific):**
- CMD_RESET_POS
- CMD_LOAD_TRAJECTORY
- CMD_START_MOTION
- CMD_LOAD_GAINS
- CMD_STOP_MOTOR
- CMD_IO_CTRL
- CMD_SET_HOME_MODE
- CMD_CLEAR_BITS
- CMD_SAVE_AS_HOME
- CMD_ADD_PATHPOINT
- CMD_EXT

## State Management

**Shared state object** (`ServoState`) caches all servo data:
- Position, velocity, pos_error
- Status flags (power, servo_on, move_done, etc.)
- PID gains (kp, kd, ki)
- Home position
- Control signals (stop_cmd, pic_ae)

**Updated by:**
- `read_status()` - updates from hardware
- Motion commands - update expected state
- Diagnostic matching - uses current state

## Performance

- **Status read time:** 20.51ms average
- **Update rate:** 48.8 Hz (validated)
- **Speedup potential:** Position-only reads ~same as full status (hardware limited)
