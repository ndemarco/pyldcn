# LDCN Module Class Design

**Date:** 2025-10-29
**Status:** ✅ Implementation Complete - Awaiting Hardware Verification
**Last Updated:** 2025-11-10

---

## Purpose

This document describes the **design principles and architecture** for the pyldcn module. For implementation details, see the source code.

⚠️ **This is a design document, not a code reference.** The actual implementation is the Single Source of Truth (SSOT). Refer to source code for current API details.

---

## Class Hierarchy

```
LDCNNetwork                    # pyldcn/network.py
    │
    ├── devices: list[LDCNDevice]
    │
    └── LDCNDevice (ABC)       # pyldcn/network.py
            │
            ├── LS231SE        # pyldcn/devices/servo.py
            ├── SK2310g2       # pyldcn/devices/io.py
            └── LS773          # pyldcn/devices/io.py
```

---

## Design Principles

1. **Single Responsibility**: Each class handles one aspect (network, device, servo, I/O)
2. **Generic at Base**: LDCN protocol commands at network/base level
3. **Specific in Subclasses**: Device-specific commands in subclasses
4. **Single send_command()**: One implementation in LDCNNetwork, all else delegates
5. **Type Safety**: Full type hints for IDE support and validation
6. **Verification Tracking**: All methods marked UNVERIFIED until hardware tested
7. **Single Source of Truth**: Code is the authoritative reference, not documentation

---

## 1. LDCNNetwork Class

**Implementation:** [pyldcn/network.py](../../pyldcn/network.py)

**Purpose**: Manages serial communication and network-level LDCN protocol.

### Key Responsibilities

- Serial port management (open, close, baud rate control)
- LDCN protocol packet building and transmission
- Network initialization (reset, addressing, discovery)
- Device object management
- Baud rate detection and configuration

### Method Categories

**Connection Management:**
- `__init__()`, `open()`, `close()`, `_open_port()`

**Core Protocol:**
- `send_command()` - Single source of truth for LDCN communication

**Baud Rate Management:**
- `_try_baud()`, `auto_detect_baud()`, `set_baud_rate()`

**Network Initialization:**
- `reset()` - Hard reset all devices
- `address_devices()` - Sequential addressing
- `discover_devices()` - Query device types
- `verify_devices()` - Verify communication
- `create_device_objects()` - Create typed device objects
- **`initialize()`** - Adaptive initialization with multiple modes (VALIDATE, SOFT, READDRESS, FULL, AUTO)
- `validate_devices()` - Fast validation without state changes
- `soft_initialize()` - State-preserving discovery

**Context Manager:**
- `__enter__()`, `__exit__()`

See [pyldcn/network.py](../../pyldcn/network.py) for detailed documentation.

---

## 2. LDCNDevice Base Class

**Implementation:** [pyldcn/network.py](../../pyldcn/network.py)

**Purpose**: Abstract base class for all LDCN devices.

### Key Responsibilities

- Device-specific command delegation
- Generic LDCN protocol operations (NOP, define status, reset position)
- Abstract `read_status()` for subclass implementation

See [pyldcn/network.py](../../pyldcn/network.py) for detailed documentation.

---

## 3. LS231SE Class (Servo Drive)

**Implementation:** [pyldcn/devices/servo.py](../../pyldcn/devices/servo.py)

**Purpose**: Servo drive specific operations.

### Key Responsibilities

- 7-step servo initialization sequence
- PID gain configuration
- Motion control (trajectory loading, position commands)
- Status reading and fault detection
- Amplifier enable/disable

See [pyldcn/devices/servo.py](../../pyldcn/devices/servo.py) for detailed documentation.

---

## 4. SK2310g2 Class (I/O Controller / Supervisor)

**Implementation:** [pyldcn/devices/io.py](../../pyldcn/devices/io.py)

**Purpose**: I/O controller operations. The SK-2310g2 is a generic LDCN I/O device used as supervisory controller with safety and spindle control functions.

### Key Responsibilities

- Safety system monitoring (e-stop, guard doors, safe zones)
- Diagnostic code reading and decoding
- Power control and monitoring
- Digital I/O operations
- Analog I/O (spindle speed control)
- Spindle enable/disable

See [pyldcn/devices/io.py](../../pyldcn/devices/io.py) and [SUPERVISORY_CONTROLLER.md](../SUPERVISORY_CONTROLLER.md) for detailed documentation.

---

## 5. LS773 Class (Network I/O Node)

**Implementation:** [pyldcn/devices/io.py](../../pyldcn/devices/io.py)

**Purpose**: Generic I/O controller operations for the LS-773 Network I/O Node.

### Key Responsibilities

- General purpose digital I/O (10 inputs, 6+1 outputs)
- PWM control (20 KHz on outputs 1 & 2)
- 32-bit counter/timer with prescaler
- 3 analog inputs (8-bit, configurable range)
- Synchronized I/O capture and application

See [pyldcn/devices/io.py](../../pyldcn/devices/io.py) for detailed documentation.

---

## 6. Communications Management Layer (Future)

**Purpose**: Stealthy debug/monitoring with minimal impact.

### Features (Phase 3)

- Async status monitoring thread
- Configurable verbosity levels (ERROR, WARN, INFO, DEBUG, TRACE)
- Pluggable logging backends (console, file, network)
- Performance metrics (timing, errors, retries)
- Packet capture for wire-level debugging
- Intelligent scheduling to avoid disrupting 1kHz control loop

**Implementation**: Separate `LDCNDebugger` class or mixin.

---

## Usage Examples

See source code docstrings and example scripts:
- [pyldcn/network.py](../../pyldcn/network.py) - Class docstrings with examples
- [pyldcn/devices/servo.py](../../pyldcn/devices/servo.py) - Servo usage examples
- [pyldcn/devices/io.py](../../pyldcn/devices/io.py) - I/O controller examples

### Basic Usage Pattern

```python
from pyldcn.network import LDCNNetwork, InitMode

# Automatic adaptive initialization (recommended)
with LDCNNetwork('/dev/ttyUSB0') as network:
    num_devices, device_info = network.initialize(mode=InitMode.AUTO)
    network.set_baud_rate(125000)  # Upgrade speed after init

    # Work with devices
    servo = network.devices[0]
    servo.initialize()
    servo.move_to(position=10.0, velocity=100.0, accel=50.0, scale=2000.0)
```

---

## Implementation Status

### Completed (🟡 IMPLEMENTED - Awaiting Hardware Verification)

1. ✅ **Class Design Documentation** - Complete and detailed
2. ✅ **Module Structure** - Created modular architecture:
   - `pyldcn/network.py` (1252 lines) - LDCNNetwork and LDCNDevice base classes
   - `pyldcn/devices/servo.py` (595 lines) - LS231SE servo drive class
   - `pyldcn/devices/io.py` (1328 lines) - SK2310g2 and LS773 I/O controller classes
3. ✅ **Base Protocol Methods** - All implemented:
   - `send_command()` - Core LDCN protocol communication
   - `auto_detect_baud()` - Baud rate detection
   - `set_baud_rate()` - Network baud rate changes
   - `reset()` - Hard reset at all baud rates
   - `address_devices()` - Sequential device addressing
   - `discover_devices()` - Device type discovery
   - `verify_devices()` - Communication verification
4. ✅ **Adaptive Initialization** - Enhanced beyond original design:
   - `validate_devices()` - Fast validation without state changes
   - `soft_initialize()` - State-preserving discovery
   - `initialize()` - Multi-mode adaptive initialization (VALIDATE, SOFT, READDRESS, FULL, AUTO)
5. ✅ **Device-Specific Subclasses** - Fully implemented:
   - **LS231SE** - Servo drive with 7-step initialization, motion control, status reading
   - **SK2310g2** - Supervisory controller with safety monitoring, diagnostic decoding, I/O control
   - **LS773** - Generic I/O node with PWM, counter/timer, analog I/O
6. ✅ **High-Level Control** - Command layer implemented:
   - `pyldcn/command/axis.py` - Axis configuration and motion control
   - `pyldcn/command/machine.py` - Multi-axis machine coordination

### In Progress

7. 🔴 **Hardware Testing** - Ready for testing:
   - All methods marked 🔴 UNVERIFIED
   - Awaiting hardware access for verification
   - Need to compare behavior with original utilities byte-for-byte

### Next Steps

1. **Hardware Verification Testing**
   - Test basic protocol methods (send_command, status reading)
   - Verify baud detection and network initialization
   - Test servo initialization and motion control
   - Verify SK2310g2 diagnostic reading and safety monitoring
   - Mark verified methods as 🟢 VERIFIED

2. **Create Test Suite**
   - Unit tests for protocol methods
   - Integration tests with hardware
   - Regression tests against original utilities

3. **Documentation**
   - Add hardware test results and notes
   - Document any deviations from original behavior
   - Create user guide with examples

### Implementation Notes

**Current Status:** All designed functionality has been implemented in code (~3175 lines). The module is feature-complete and ready for hardware testing. The implementation includes enhancements beyond the original design, particularly the adaptive initialization system which provides state preservation and faster reconnection capabilities.

