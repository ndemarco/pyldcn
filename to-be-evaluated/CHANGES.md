# Changes to Existing Code

## Modified Files

### 1. pyldcn/protocol.py

**Change:** Removed servo-specific commands

**Before:**
```python
# Generic LDCN commands (supported by all device types)
CMD_RESET_POS = 0x00
CMD_LOAD_TRAJECTORY = 0x04
CMD_LOAD_GAINS = 0x06
CMD_STOP_MOTOR = 0x07
# ... many servo-specific commands
```

**After:**
```python
# Generic LDCN commands (supported by all device types)
CMD_SET_ADDRESS = 0x01
CMD_DEFINE_STATUS = 0x02
CMD_READ_STATUS = 0x03
CMD_SET_BAUD = 0x0A
CMD_NOP = 0x0E
CMD_HARD_RESET = 0x0F
```

**Rationale:** Servo-specific commands don't belong in generic protocol file. Moved to servo.py.

### 2. pyldcn/device.py

**Changes:**
1. Removed `CMD_RESET_POS` import (now in servo.py)
2. Removed `reset_position()` method from base class

**Before:**
```python
from .protocol import (
    CMD_NOP,
    CMD_DEFINE_STATUS,
    CMD_RESET_POS,  # ← Removed
)

def reset_position(self) -> None:  # ← Removed
    """Reset position counter to zero (if supported)."""
    self.send_command(CMD_RESET_POS)
```

**After:**
```python
from .protocol import (
    CMD_NOP,
    CMD_DEFINE_STATUS,
)
# reset_position() removed - not all devices support it
```

**Rationale:** Position reset is servo-specific, not supported by all LDCN devices (e.g., I/O modules).

### 3. pyldcn/network.py

**Addition:** Added `get_device()` method

```python
def get_device(self, address: int, device_type: Optional[str] = None) -> LDCNDevice:
    """
    Get or create a device at the specified address.

    Args:
        address: Device address (1-127)
        device_type: Optional device type (e.g., "LS-231SE", "SK-2310g2")

    Returns:
        LDCNDevice object at the specified address
    """
    # Check if device already exists
    for device in self.devices:
        if device.address == address:
            return device

    # Create new device if type provided
    if device_type is not None:
        if device_type == 'LS-231SE':
            new_device = LS231SE(self, address)
        elif device_type == 'SK-2310g2':
            new_device = SK2310g2(self, address)
        else:
            new_device = UnknownDevice(self, address)

        self.devices.append(new_device)
        return new_device

    raise ValueError(f"Device at address {address} not found")
```

**Rationale:** Simplifies device access pattern used in examples and tests.

### 4. pyldcn/devices/servo.py

**Major Additions:**

**A. Command constants (LS-231SE specific):**
```python
# LS-231SE Servo Drive Commands (Motor/Servo Specific)
CMD_RESET_POS = 0x00
CMD_LOAD_TRAJECTORY = 0x04
CMD_START_MOTION = 0x05
CMD_LOAD_GAINS = 0x06
CMD_STOP_MOTOR = 0x07
CMD_IO_CTRL = 0x08
CMD_SET_HOME_MODE = 0x09
CMD_CLEAR_BITS = 0x0B
CMD_SAVE_AS_HOME = 0x0C
CMD_ADD_PATHPOINT = 0x0D
CMD_EXT = 0x0E
```

**B. Simplified `load_gains()` method:**
```python
def load_gains(self, kp, kd, ki,
               il=255, ol=255, cl=255,
               el=16000, sr=1, db=0):
    """Load PID gains with sensible defaults."""
    self.set_gains(kp, kd, ki, il, ol, cl, el, sr, db)
```

**C. Stop command:**
```python
def stop(self):
    """Stop motor motion immediately."""
    self._motion.stop()
```

**D. Subsystem property accessors:**
```python
@property
def status(self):
    """Access to Status subsystem for advanced operations."""
    return self._status

@property
def motion(self):
    """Access to Motion subsystem for advanced operations."""
    return self._motion

@property
def io(self):
    """Access to IO subsystem for advanced operations."""
    return self._io
```

**E. Diagnostic convenience methods:**
```python
def is_ready(self, mode: str = "LDCN") -> bool:
    """Check if servo is ready for motion commands."""
    return self._status.is_operational(mode)

def get_condition(self, mode: str = "LDCN"):
    """Get current diagnostic condition."""
    return self._status.get_condition(mode)

def is_faulted(self, mode: str = "LDCN") -> bool:
    """Check if servo is in fault state."""
    return self._status.is_faulted(mode)

def print_status(self, include_leds: bool = False, mode: str = "LDCN"):
    """Print formatted status report."""
    print(self._status.format_status(include_leds, mode))
```

### 5. pyldcn/devices/servo_state.py

**Addition:** Control signal flags for diagnostic matching

```python
# Control Signals (needed for diagnostic condition matching)
stop_cmd: bool = False      # Stop motor command bit
pic_ae: bool = False        # Power driver enable (Pic_ae≡DE)
```

**Rationale:** Required for complete diagnostic pattern matching per SK-2310g2 model.

### 6. pyldcn/devices/servo_status.py

**Major Enhancements:**

**A. DEFINE_STATUS implementation:**
```python
def define_status(self, status_mask: int):
    """Configure which status items are returned by READ_STATUS."""
    from pyldcn.protocol import CMD_DEFINE_STATUS
    self._device.send_command(CMD_DEFINE_STATUS,
                             [status_mask & 0xFF, (status_mask >> 8) & 0xFF])

def read_status_custom(self, status_mask: int) -> Dict:
    """Read custom status items without changing DEFINE_STATUS."""
    from pyldcn.protocol import CMD_READ_STATUS
    response = self._device.send_command(CMD_READ_STATUS,
                                        [mask & 0xFF, (mask >> 8) & 0xFF])
    return self._parse_status(response, status_mask)

def read_full_status(self) -> Dict:
    """Read all available status items."""
    return self.read_status_custom(0x000F)
```

**B. Diagnostic integration:**
```python
def get_condition(self, mode: str = "LDCN"):
    """Get current diagnostic condition from cached status bytes."""
    return diag.match_diagnostic_condition(
        self._state.status_byte,
        self._state.aux_status,
        self._state.stop_cmd,
        self._state.pic_ae,
        mode
    )

def is_operational(self, mode: str = "LDCN") -> bool:
    """Check if servo is operational (ready for motion)."""
    condition = self.get_condition(mode)
    return diag.is_operational(condition)

def is_faulted(self, mode: str = "LDCN") -> bool:
    """Check if servo is in fault state."""
    condition = self.get_condition(mode)
    return diag.is_faulted(condition)

def format_status(self, include_leds: bool = False, mode: str = "LDCN") -> str:
    """Format status as human-readable string."""
    condition = self.get_condition(mode)
    return diag.format_diagnostic_status(
        self._state.status_byte,
        self._state.aux_status,
        condition,
        include_leds
    )
```

### 7. pyldcn/devices/servo_motion.py

**Major Enhancements:**

**A. Position reset:**
```python
def reset_position(self, position: int = 0):
    """Reset current position counter."""
    from .servo import CMD_RESET_POS
    pos_bytes = [
        position & 0xFF,
        (position >> 8) & 0xFF,
        (position >> 16) & 0xFF,
        (position >> 24) & 0xFF
    ]
    self._device.send_command(CMD_RESET_POS, pos_bytes)
    self._state.position = position
```

**B. Stop command:**
```python
def stop(self, smooth: bool = False):
    """Stop motor motion immediately without disabling amplifier."""
    from .servo import CMD_STOP_MOTOR
    if smooth:
        stop_ctrl = STOP_SMOOTH | AMP_ENABLE
    else:
        stop_ctrl = STOP_ABRUPT | AMP_ENABLE
    self._device.send_command(CMD_STOP_MOTOR, [stop_ctrl])
```

**C. Home position saving:**
```python
def save_home(self):
    """Save current position as home position."""
    from .servo import CMD_SAVE_AS_HOME
    self._device.send_command(CMD_SAVE_AS_HOME, [])
    if self._state.position is not None:
        self._state.home_position = self._state.position
```

**D. Path mode:**
```python
def add_path_point(self, position: int, velocity: int, accel: int):
    """Add path point to motion buffer."""
    from .servo import CMD_ADD_PATHPOINT
    point_data = struct.pack('<hhh', position, velocity, accel)
    self._device.send_command(CMD_ADD_PATHPOINT, list(point_data))
```

### 8. pyldcn/devices/servo_io.py

**Enhancement:** Brake and digital I/O control

```python
def set_brake(self, released: bool):
    """Control brake output (OUTbit0)."""
    from .servo import CMD_IO_CTRL
    io_byte = 0x01 if released else 0x00
    self._device.send_command(CMD_IO_CTRL, [io_byte])

def set_output(self, output_num: int, state: bool):
    """Set digital output state."""
    from .servo import CMD_IO_CTRL
    if output_num < 0 or output_num > 7:
        raise ValueError(f"Output {output_num} out of range (0-7)")
    io_byte = (1 << output_num) if state else 0
    self._device.send_command(CMD_IO_CTRL, [io_byte])

def read_limit_switches(self) -> Dict[str, bool]:
    """Read limit switch states from aux status."""
    return {
        'limit1': self._state.limit1,
        'limit2': self._state.limit2,
        'home': self._state.home_source
    }
```

## Import Changes

Updated imports in subsystem files to use servo-specific commands:

**servo_motion.py:**
```python
# Changed from: from pyldcn.protocol import CMD_*
# Changed to:
from .servo import CMD_RESET_POS, CMD_LOAD_TRAJECTORY, CMD_START_MOTION, CMD_SET_HOME_MODE, CMD_SAVE_AS_HOME, CMD_ADD_PATHPOINT
```

**servo_io.py:**
```python
from .servo import CMD_IO_CTRL, CMD_CLEAR_BITS
```

**servo_safety.py:**
```python
from .servo import CMD_EXT
from pyldcn.protocol import CMD_NOP  # NOP is generic LDCN
```

## Summary of Changes

| File | Type | Lines Changed | Impact |
|------|------|---------------|--------|
| protocol.py | Removal | -10 | Cleaned up command organization |
| device.py | Removal | -12 | Removed servo-specific base methods |
| network.py | Addition | +45 | Added get_device() helper |
| servo.py | Enhancement | +60 | Added commands, properties, methods |
| servo_state.py | Addition | +2 | Control signals for diagnostics |
| servo_status.py | Enhancement | +175 | DEFINE_STATUS, diagnostics |
| servo_motion.py | Enhancement | +114 | Position reset, stop, homing, path |
| servo_io.py | Enhancement | +77 | Brake, I/O control |

**Total Modifications:** ~470 lines added/changed in existing files
