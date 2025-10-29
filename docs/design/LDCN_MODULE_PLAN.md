# LDCN Python Module Refactoring Plan

**Goal:** Create a reusable Python module for LDCN network communication with object-oriented device abstraction.

**Start Date:** 2025-10-29

---

## Architecture Overview

```
LDCNNetwork (top-level class)
├── Serial port management
├── Protocol-level operations (reset, baud, addressing)
├── Device discovery and management
└── devices[] - list of LDCNDevice instances

LDCNDevice (abstract base class)
├── Common properties: address, network ref, device_type
├── Common methods: send_command(), read_status(), verify_checksum()
└── Protocol compliance

LS231SE (servo drive subclass)
├── Servo-specific properties: position, velocity, servo_on, fault
├── Servo methods: enable(), disable(), move_to(), set_gains(), home()
└── Status monitoring

SK2310g2 (supervisor subclass)
├── Supervisor properties: diagnostic_code, digital_inputs, power_state
├── Supervisor methods: read_diagnostic(), read_inputs(), set_outputs()
└── Safety monitoring
```

---

## Phase 1: Survey (Understanding Existing Code)

### Task 1.1: Clean up directory structure
**Status:** ✅ COMPLETED
**Actions Taken:**
- Merged `utilities/` into `utils/`
- Kept `utils/` (has superior ldcn_diagnostic.py with fault monitoring)
- Moved `test_position_command.py` to `utils/`
- Moved `test_servo_init.py` to `utils/`
- Deleted duplicate `utilities/` directory

**Result:** All Python utilities now in `/home/nick/projects/linuxcnc-logosol/utils/`

### Task 1.2: Create Python file inventory
**Status:** ✅ COMPLETED
**Output:** `PYTHON_FILE_INVENTORY.md` created with:
- 5 Python files documented
- Key classes and methods identified
- Common protocol constants extracted
- Device addressing scheme documented

**Files:**
1. `ldcn_diagnostic.py` - Diagnostic utility with fault monitoring
2. `ldcn_init.py` - Network initialization
3. `ldcn_monitor.py` - Real-time status monitoring
4. `test_servo_init.py` - Servo initialization testing
5. `test_position_command.py` - Position command testing

### Task 1.3: Extract functions from test files
**Status:** ✅ COMPLETED
**Output:** Updated PYTHON_FILE_INVENTORY.md with:
- `test_servo_init.py` - 5 functions extracted, 7-step servo init sequence documented
- `test_position_command.py` - 3 functions extracted, trajectory command format documented
- PID gain values documented
- Servo status parsing documented

### Task 1.4: Categorize functions
**Status:** ✅ COMPLETED
**Output:** Created `FUNCTION_CATEGORIZATION.md` with:
- **Category 1:** Protocol-Level Functions → LDCNNetwork (11 functions)
- **Category 2:** Servo-Specific Functions → LS231SE (9 functions)
- **Category 3:** Supervisor-Specific Functions → SK2310g2 (5 functions)
- **Category 4:** Utility Functions (6 functions)
- Complete function mapping table (25+ methods mapped)
- Protocol constants extracted and organized

---

## Phase 2: Design (Planning Architecture)

### Task 2.1: Define LDCNNetwork class interface
**Status:** ✅ COMPLETED
**Output:** `CLASS_DESIGN.md` created with complete LDCNNetwork interface:
- Full method signatures with type hints and docstrings
- Properties: port, baud_rate, serial, devices, timeout
- Core protocol: send_command() (single source of truth)
- Baud management: auto_detect_baud(), set_baud_rate(), _try_baud()
- Network init: reset(), address_devices(), verify_devices(), initialize()
- Device discovery: discover_devices()
- Context manager support: __enter__(), __exit__()
- Protocol constants: HEADER, commands, BAUD_RATES, timing constants

### Task 2.2: Define LDCNDevice base class interface
**Status:** ✅ COMPLETED
**Output:** LDCNDevice abstract base class defined in `CLASS_DESIGN.md`:
- Properties: network, address, device_type, model_id
- Methods: send_command() (delegates to network), nop(), define_status()
- Abstract method: read_status() (implemented by subclasses)
- String representation: __repr__()

### Task 2.3: Define device-specific subclass interfaces
**Status:** ✅ COMPLETED
**Output:** Complete LS231SE and SK2310g2 class definitions in `CLASS_DESIGN.md`:

**LS231SE (Servo Drive):**
- Properties: position, velocity, status_byte, aux_status, pos_error, gains (kp, kd, ki)
- Initialization: initialize() (7-step sequence)
- Status: read_status(), read_position(), decode_status_flags(), check_faults()
- Configuration: set_gains(), configure_status()
- Motion: move_to(), move_to_counts()
- Control: enable(), disable(), reset_position(), clear_faults()
- Constants: status bit masks, stop motor flags, status reporting bits

**SK2310g2 (Supervisor/I/O Controller):**
- Properties: diagnostic_code, power_state, digital_inputs, analog_inputs
- Configuration: configure()
- Status: read_status(), read_diagnostic(), read_power_state()
- Power: wait_for_power_button()
- I/O: read_digital_inputs(), set_digital_outputs(), read_analog_inputs()
- Constants: diagnostic codes

### Task 2.4: Reorganize protocol documentation
**Status:** ✅ COMPLETED
**Output:**
- Renamed PROTOCOL.md → LDCN_PROTOCOL.md (generic commands only)
- Created SERVO_COMMANDS.md for servo-specific commands
- Updated docs/README.md with new documentation structure
- Clear separation: generic LDCN protocol vs device-specific commands

### Task 2.5: Exception hierarchy
**Status:** ✅ COMPLETED
**Output:** Exception classes defined in `CLASS_DESIGN.md`:
- LDCNError (base)
- LDCNTimeoutError
- LDCNChecksumError
- LDCNDetectionError
- LDCNInitializationError

### Task 2.6: Communications management layer
**Status:** 📝 Outlined (Phase 3 future work)
**Output:** Section in `CLASS_DESIGN.md` outlining future features:
- Async status monitoring
- Configurable verbosity levels
- Pluggable logging backends
- Performance metrics
- Packet capture
- Non-intrusive to 1kHz control loop

---

## Phase 3: Implementation (Building the Module)

### Task 3.1: Create ldcn_network.py module
**Status:** ✅ COMPLETED
**Location:** `/home/nick/projects/linuxcnc-logosol/ldcn_network.py`
**Contents:**
- LDCNNetwork class (20+ methods implemented)
- LDCNDevice base class (abstract)
- LS231SE servo drive class (15+ methods implemented)
- SK2310g2 I/O controller class (base methods implemented, I/O stubs)
- Exception hierarchy (5 exception classes)
- Complete protocol constants
**Size:** ~1500 lines, 45 KB

### Task 3.2: Refactor protocol functions into LDCNNetwork
**Status:** ✅ COMPLETED
**Implemented:**
- `send_command()` - Single source of truth for LDCN communication
- `auto_detect_baud()` - Baud rate detection
- `set_baud_rate()` - Baud rate upgrade
- `reset()` - Hard reset
- `address_devices()` - Sequential addressing (auto-discovery)
- `discover_devices()` - Device type query (READ_STATUS bit 5)
- `verify_devices()` - Communication verification
- `create_device_objects()` - Device object instantiation
- `initialize()` - Complete initialization sequence

### Task 3.3: Refactor servo functions into LS231SE
**Status:** ✅ COMPLETED
**Implemented:**
- `initialize()` - 7-step servo initialization
- `read_status()` - Complete status with parsing
- `read_position()` - Fast position read
- `set_gains()` - PID gain configuration
- `move_to()` / `move_to_counts()` - Motion commands
- `enable()` / `disable()` - Amplifier control
- `clear_faults()` - Sticky bit clearing
- `check_faults()` - Fault detection
- `decode_status_flags()` - Status byte decoding

### Task 3.4: Refactor supervisor functions into SK2310g2
**Status:** 🟡 PARTIALLY COMPLETED
**Implemented:**
- `configure()` - Full status configuration
- `read_status()` - Status reading
- `read_diagnostic()` - Diagnostic code
- `read_power_state()` - Power button state
- `wait_for_power_button()` - Power button detection
**Stubs (TBD - application-specific I/O mapping):**
- Digital I/O operations
- Analog I/O operations
- Spindle control
- Safety monitoring (E-stop, covers, safe zone)

### Task 3.5: Add docstrings and type hints
**Status:** ✅ COMPLETED
**Completed:**
- All classes have comprehensive docstrings
- All methods have full type hints
- All parameters documented with Args/Returns/Raises
- Module-level docstring with usage examples
- 🔴 UNVERIFIED status markers on all functions

---

## Phase 4: Validation (Testing)

### Task 4.1: Create test script
**Status:** ✅ COMPLETED
**Location:** `/home/nick/projects/linuxcnc-logosol/test_ldcn_module.py`
**Tests Implemented:**
- Test 1: Network initialization (reset, address, discover, baud upgrade)
- Test 2: Servo operations (initialize, read status, check faults)
- Test 3: I/O controller operations (configure, read diagnostic, power state)
- Test 4: Continuous position reading (2 seconds at 10 Hz)
**Size:** ~400 lines

### Task 4.2: Test device discovery
**Status:** ⏳ READY FOR HARDWARE
**Verify:** Correct devices found at correct addresses

### Task 4.3: Test servo operations
**Status:** ⏳ READY FOR HARDWARE
**Verify:** Position read/write, enable/disable, status monitoring

### Task 4.4: Test supervisor operations
**Status:** ⏳ READY FOR HARDWARE
**Verify:** Diagnostic reading matches LED display

### Task 4.5: Compare with original scripts
**Status:** ⏳ READY FOR HARDWARE
**Verify:** New module produces same results as original utilities

---

## Target Usage Example

```python
#!/usr/bin/env python3
from ldcn_network import LDCNNetwork

# Initialize network
net = LDCNNetwork('/dev/ttyUSB0')
net.open()
net.discover_devices()

# Access supervisor
supervisor = net.devices[6]  # SK2310g2 at address 6
diagnostic = supervisor.read_diagnostic()
print(f"Supervisor diagnostic: 0x{diagnostic:02X} - {supervisor.diagnostic_description}")

# Access servo drive
servo_y = net.devices[1]  # LS231SE Y-axis at address 1
position = servo_y.read_position()
print(f"Y-axis position: {position}")

# Move servo
servo_y.enable()
servo_y.move_to(position + 1000, velocity=100)

# Clean up
net.close()
```

---

## Decision Log

### Decision 1: Use single file vs package
**Date:** TBD
**Decision:** TBD
**Rationale:** TBD

### Decision 2: Backwards compatibility
**Date:** TBD
**Decision:** TBD
**Rationale:** Should old scripts continue to work?

### Decision 3: Error handling strategy
**Date:** TBD
**Decision:** TBD
**Rationale:** Exceptions vs return codes?

---

## Notes

- Keep original utility scripts intact during development
- Test on real hardware frequently
- Document any protocol discoveries
- Compare serial communication byte-for-byte with working Python scripts

---

## Progress Tracking

**Phase 1:** ✅ COMPLETED (All survey tasks done!)
**Phase 2:** ✅ COMPLETED (All design tasks done!)
**Phase 3:** ✅ COMPLETED (Implementation done, ready for testing!)
**Phase 4:** ⏳ READY FOR HARDWARE (Test script ready, awaiting hardware validation)

**Last Updated:** 2025-10-29

**Module Status:** 🔴 UNVERIFIED - All ~1500 lines of code implemented but not tested on hardware

---

## Change Log

### 2025-10-29 14:00 - 🎉 PHASE 3 COMPLETE!
- ✅ Created ldcn_network.py module (~1500 lines, 45 KB)
- ✅ Implemented LDCNNetwork class (20+ methods)
  - `send_command()` - single source of truth for LDCN communication
  - `initialize()` - complete initialization sequence
  - `auto_detect_baud()`, `set_baud_rate()` - baud rate management
  - `reset()`, `address_devices()` - network initialization
  - `discover_devices()`, `verify_devices()` - device discovery
  - `create_device_objects()` - device instantiation
- ✅ Implemented LS231SE class (15+ methods)
  - `initialize()` - 7-step servo initialization
  - `move_to()`, `move_to_counts()` - motion control
  - `read_status()`, `read_position()` - status monitoring
  - `set_gains()` - PID configuration
  - `enable()`, `disable()` - amplifier control
  - `check_faults()`, `clear_faults()` - fault management
- ✅ Implemented SK2310g2 class (base methods + stubs)
  - `configure()`, `read_status()` - configuration and status
  - `read_diagnostic()`, `read_power_state()` - supervisory functions
  - `wait_for_power_button()` - power detection
  - Digital I/O, analog I/O, spindle control - stubs (TBD based on wiring)
- ✅ Added exception hierarchy (5 exception classes)
- ✅ Full type hints on all methods
- ✅ Comprehensive docstrings throughout
- ✅ Context manager support (`with` statement)
- ✅ Created test_ldcn_module.py (~400 lines)
  - 4 comprehensive test cases
  - ANSI color terminal output
  - Command-line options
- ✅ Created LDCN_MODULE_README.md
- ✅ **Phase 3 Implementation COMPLETE!**
- 📝 Ready for Phase 4: Hardware validation

### 2025-10-29 12:00
- ✅ Redesigned network initialization flow - always at 19200 baud
- ✅ Separated baud upgrade from initialization (set_baud_rate() is separate)
- ✅ Redesigned device discovery flow:
  - `address_devices()` - assigns addresses 1, 2, 3... (doesn't need device count)
  - `discover_devices()` - queries device IDs and versions (READ_STATUS with bit 5)
  - `verify_devices()` - confirms devices still responding (takes device list)
  - `create_device_objects()` - creates device objects from device info
- ✅ Added device ID constants and DEVICE_CLASS_MAP for device type mapping
- ✅ Updated initialize() to return (num_devices, device_info) tuple
- ✅ Documented granular initialization steps for advanced usage
- 📝 Clear separation: addressing → discovery → verification → object creation

### 2025-10-29 11:30
- ✅ Updated SK2310g2 class design with complete I/O capabilities
- ✅ Added spindle control methods (enable, disable, speed control)
- ✅ Added safety monitoring methods (E-stop, covers, safe zone)
- ✅ Added analog I/O methods (3 inputs, 1 output for spindle)
- ✅ Added power relay control (dual mechanical relays)
- ✅ Clarified device hierarchy: generic I/O controller → supervisory role
- ✅ Documented application-specific I/O channel mapping requirements
- 📝 SK2310g2 now has 25+ methods covering all hardware capabilities

### 2025-10-29 11:00
- ✅ Created CLASS_DESIGN.md with complete class interfaces
- ✅ Designed LDCNNetwork class (20+ methods, full type hints)
- ✅ Designed LDCNDevice abstract base class
- ✅ Designed LS231SE servo drive subclass (15+ methods)
- ✅ Designed SK2310g2 I/O controller subclass (initial 10+ methods)
- ✅ Defined exception hierarchy (5 exception classes)
- ✅ Renamed PROTOCOL.md → LDCN_PROTOCOL.md
- ✅ Created SERVO_COMMANDS.md for device-specific commands
- ✅ Updated docs/README.md with new structure
- ✅ **Phase 2 Design Complete!**
- 📝 Ready for Phase 3: Implementation

### 2025-10-29 09:30
- ✅ Added verification status system (🔴 UNVERIFIED → 🟡 IMPLEMENTED → 🟢 VERIFIED)
- ✅ Marked all functions as UNVERIFIED for testing
- ✅ Changed BRD_VALUES to BAUD_RATES
- ✅ Moved integration_test.py to utils/
- ✅ Added comprehensive function overlap analysis
- ✅ Documented critical finding: send_command() duplicated in ALL 6 files
- ✅ Reorganized documentation into docs/logosol/ and docs/ldcn_module/
- ✅ Created docs/README.md as documentation index

### 2025-10-29 08:30
- ✅ Extracted all functions from test_servo_init.py (5 functions)
- ✅ Extracted all functions from test_position_command.py (3 functions)
- ✅ Created FUNCTION_CATEGORIZATION.md
- ✅ Categorized 25+ functions into 4 categories
- ✅ Mapped all functions to target class methods
- ✅ **Phase 1 Survey Complete!**

### 2025-10-29 08:25
- ✅ Merged utilities/ into utils/ directory
- ✅ Moved test files to utils/
- ✅ Created PYTHON_FILE_INVENTORY.md
- 📝 Identified 6 Python files for refactoring
- 📝 Documented common protocol patterns
