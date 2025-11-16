# pyldcn - Python LDCN Communication Library

**A Python library for communicating with Logosol LDCN (Logosol Distributed Control Network) devices**

[![License: GPL v2+](https://img.shields.io/badge/License-GPL%20v2+-blue.svg)](https://www.gnu.org/licenses/gpl-2.0)
![Status: Alpha](https://img.shields.io/badge/Status-Alpha-orange)
![Python: 3.7+](https://img.shields.io/badge/Python-3.7+-blue)

---

## Overview

`pyldcn` is a Python library for communicating with Logosol LDCN devices including:
- **LS-231SE** servo drives
- **SK-2310g2** I/O controllers / supervisors
- Other LDCN-compatible devices

The library provides a clean, object-oriented interface with full type hints and comprehensive documentation.

### Key Features

- **Clean Architecture** - Object-oriented design with base classes and device-specific subclasses
- **Type Hints** - Full typing support for IDE autocomplete and static analysis
- **Comprehensive** - All LDCN protocol commands implemented
- **Well-Documented** - Extensive docstrings and documentation
- **Context Managers** - Pythonic `with` statement support
- **Zero Duplication** - Single source of truth for all protocol operations

### Architecture

The library uses a layered architecture with clear abstraction boundaries:

```
┌─────────────────────────────────────────────────────────────┐
│                      USER CODE                              │
│  (Examples, applications, scripts)                          │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│               NAMED HELPER METHODS                          │
│  servo.move_to(), device.read_status(), etc.               │
│  ✓ PREFERRED for user code                                 │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│              DEVICE LAYER (device.py)                       │
│  device.send_command(cmd, data)                             │
│  ✓ Use when implementing device methods                    │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│              NETWORK LAYER (network.py)                     │
│  network.send_command(addr, cmd, data)                      │
│  ⚠ Avoid from user code - use device methods               │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│              PROTOCOL LAYER (protocol.py)                   │
│  protocol.send_command(addr, cmd, data)                     │
│  ⛔ Infrastructure only - do not call from devices/user code│
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
                   Serial Hardware (RS-485)
```

**Best Practices:**
- ✅ **DO** use named helper methods: `servo.move_to()`, `device.read_status()`
- ✅ **DO** use `device.send_command()` when implementing new device methods
- ⚠️ **AVOID** calling `network.send_command()` from device classes or user code
- ⛔ **NEVER** call `protocol.send_command()` from device classes or user code

---

## Installation

### From Source

```bash
cd ~/projects/pyldcn
pip install -e .
```

### Dependencies

```bash
pip install pyserial
```

---

## Quick Start

### Basic Usage

```python
from pyldcn import LDCNNetwork

# Initialize network
with LDCNNetwork('/dev/ttyUSB0') as network:
    # Initialize at 19200 baud (LDCN default)
    num_devices, device_info = network.initialize()
    print(f"Found {num_devices} devices")

    # Upgrade to 125kbps
    network.set_baud_rate(125000)

    # Access servo drive
    servo = network.devices[0]  # LS231SE at address 1
    servo.initialize()
    servo.move_to(position=10.0, velocity=100.0, accel=50.0, scale=2000.0)
```

### Servo Control

```python
from pyldcn import LDCNNetwork, LS231SE

with LDCNNetwork('/dev/ttyUSB0') as network:
    network.initialize()
    network.set_baud_rate(125000)

    servo = network.devices[0]  # LS231SE

    # Initialize with custom gains
    servo.initialize(kp=2, kd=50, ki=0, el=2000)

    # Motion control
    servo.enable()
    servo.move_to(position=10.0, velocity=100.0, accel=50.0, scale=2000.0)

    # Status monitoring
    status = servo.read_status()
    print(f"Position: {status['position']}")
    print(f"Velocity: {status['velocity']}")

    # Fault checking
    faults = servo.check_faults(status['status'])
    if faults:
        print(f"Active faults: {', '.join(faults)}")
```

### I/O Controller

```python
from pyldcn import LDCNNetwork, SK2310g2

with LDCNNetwork('/dev/ttyUSB0') as network:
    network.initialize()
    network.set_baud_rate(125000)

    io = network.devices[5]  # SK2310g2 at address 6

    # Configure for full status
    io.configure()

    # Read diagnostic code
    diag_code = io.read_diagnostic()
    print(f"Diagnostic: 0x{diag_code:02X}")

    # Check power state
    if io.read_power_state():
        print("Power ON")

    # Wait for power button press
    if io.wait_for_power_button(timeout=30.0):
        print("Power button pressed!")
```

### High-Level Axis Control

```python
from pyldcn import LDCNNetwork
from pyldcn.command import AxisController
import logging

# Enable logging to see progress messages
logging.basicConfig(level=logging.INFO)

with LDCNNetwork('/dev/ttyUSB0') as network:
    network.initialize()
    network.set_baud_rate(125000)

    # Create axis controller from JSON configuration
    controller = AxisController(network, 'examples/fiveaxis_full_config.json')

    # Home an axis (automatically handles power-on sequence)
    controller.home_axis('X')

    # Move to absolute position
    controller.move_absolute('X', position=100.0, velocity=50.0)

    # Move relative distance
    controller.move_relative('Y', distance=25.0)

    # Wait for motion to complete
    controller.wait_for_motion_complete('X', timeout=10.0)

    # Get current position
    pos = controller.get_position('X')
    print(f"X position: {pos:.3f} mm")
```

### Logging Configuration

Control output verbosity using Python's logging module:

```python
import logging

# Show all info messages (default behavior)
logging.basicConfig(level=logging.INFO)

# Quiet mode - only warnings and errors
logging.basicConfig(level=logging.WARNING)

# Debug mode - very verbose
logging.basicConfig(level=logging.DEBUG)

# Custom format
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
```

## Testing

### Run Tests

```bash
# Full test suite
python3 examples/test_network.py

# Custom options
python3 examples/test_network.py --port /dev/ttyUSB1 --target-baud 115200

# Test specific servo
python3 examples/test_network.py --test-servo 2
```

### Test Coverage

The test suite includes:
1. Network initialization (reset, addressing, discovery)
2. Servo operations (initialization, status, faults)
3. I/O controller operations (configuration, diagnostics)
4. Continuous position monitoring

---

## Architecture

### Class Hierarchy

```
LDCNNetwork (network manager)
    ├── manages serial port and protocol operations
    ├── device discovery and initialization
    └── devices: List[LDCNDevice]
          │
          ├── LDCNDevice (abstract base)
          │     └── common LDCN operations
          │
          ├── LS231SE (servo drive)
          │     └── motion control, status, gains
          │
          └── SK2310g2 (I/O controller)
                └── I/O, spindle, safety monitoring
```

### Key Design Principles

1. **Single Responsibility** - Each class handles one aspect
2. **Generic at Base** - LDCN protocol commands in base classes
3. **Specific in Subclasses** - Device-specific operations in subclasses
4. **Single send_command()** - One implementation, all else delegates

---

## API Reference

### LDCNNetwork

**Network initialization and management**

```python
network = LDCNNetwork(port='/dev/ttyUSB0', timeout=0.2)
network.open()  # Opens at 19200 baud

# Initialize: reset → address → discover → verify
num_devices, device_info = network.initialize()

# Upgrade baud rate (separate step)
network.set_baud_rate(125000)

# Manual control (granular)
network.reset()
num_found = network.address_devices()
device_info = network.discover_devices()
responding = network.verify_devices(device_info)
network.create_device_objects(device_info)

network.close()
```

### LS231SE (Servo Drive)

**Motion control and status monitoring**

```python
servo = network.devices[0]  # LS231SE instance

# Initialize servo (7-step sequence)
servo.initialize(kp=2, kd=50, ki=0, el=2000, sr=20)

# Motion control
servo.enable()
servo.move_to(position=10.0, velocity=100.0, accel=50.0, scale=2000.0)
servo.move_to_counts(position=20000, velocity=100000, accel=50000)

# Status monitoring
status = servo.read_status()
pos_data = servo.read_position()
faults = servo.check_faults(status['status'])

# Configuration
servo.set_gains(kp=2, kd=50, ki=0, il=40, ol=255, cl=0, el=2000, sr=20, db=0)
servo.clear_faults()
servo.disable()
```

### SK2310g2 (I/O Controller)

**I/O and supervisory operations**

```python
io_controller = network.devices[5]  # SK2310g2 instance

# Configuration
io_controller.configure()

# Status and diagnostics
status = io_controller.read_status()
diag_code = io_controller.read_diagnostic()
power_on = io_controller.read_power_state()

# Power button detection
if io_controller.wait_for_power_button(timeout=30.0):
    print("Power button pressed!")

# Note: I/O operations are application-specific stubs
# Implement based on your wiring configuration
```

---

## Status

⚠️ **HARDWARE VERIFICATION STATUS: UNVERIFIED**

All functions in this library are marked as **UNVERIFIED** until tested against real LDCN hardware and behavior is compared with original utility scripts.

### Verification Process

1. ✅ **Implemented** - Function written based on protocol documentation
2. ⏳ **Hardware Test** - Run against real LDCN devices
3. ⏳ **Verify** - Compare behavior byte-for-byte with original scripts
4. ⏳ **Mark Verified** - Change status to ✅ VERIFIED

---

### Contributing

When adding new functions or fixing bugs:

1. Add comprehensive docstring with Args, Returns, Raises
2. Add type hints for all parameters and return values
3. Mark as 🔴 UNVERIFIED until hardware tested
4. Add test case to examples/test_network.py
5. Compare behavior with original utilities
6. Update verification status after testing

---

## Known Limitations

**SK2310g2 (I/O Controller):**
- Digital I/O methods: Stubs only (raise NotImplementedError)
- Analog I/O methods: Stubs only
- Spindle control: Stubs only
- Safety monitoring: Stubs only
- **Reason**: I/O channel mapping is application-specific

**Device Discovery:**
- Device ID constants are placeholders
- **Reason**: Actual device IDs must be read from hardware

---

## License

GPL v2 or later

Copyright (C) 2025 NickyDoes

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

---

## References

- **Logosol LDCN Documentation** - Hardware manuals and protocol specs
- **LinuxCNC Project** - CNC machine control software
- **Original Utilities** - Reference implementation in linuxcnc-logosol project

---

## Support

For issues, questions, or contributions, please open an issue on GitHub.
