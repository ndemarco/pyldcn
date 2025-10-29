# Python File Inventory

**Date:** 2025-10-29
**Location:** `/home/nick/projects/linuxcnc-logosol/utils/`

## Source Files for Refactoring

### 1. ldcn_diagnostic.py (11,416 bytes)
**Purpose:** LDCN Network Diagnostic Utility

**Key Features:**
- Auto-detecting current baud rate
- Testing communication with all devices
- Displaying device status and fault conditions
- Verifying power control functionality
- Monitoring servo faults (checksum errors, current limit, position error)
- Continuous fault monitoring

**Class:**
- `LDCNDiagnostic`

**Key Methods:**
- `send_command(address, command, data=[])` - Send LDCN command
- `try_baud(baud)` - Try communication at specific baud rate
- `auto_detect_baud()` - Auto-detect current baud rate
- `check_faults(status)` - Check status byte for fault conditions
- `decode_status(status)` - Decode status byte into human-readable flags
- `test_device(address, name)` - Test communication with single device
- `test_all_devices()` - Test all expected devices
- `test_faults()` - Continuously monitor servos for faults
- `test_power_detection()` - Test power-on detection from I/O controller
- `run(test_power, test_faults)` - Main diagnostic runner

**Status Bit Flags Defined:**
- `STATUS_MOVE_DONE = (1 << 0)`
- `STATUS_CKSUM_ERROR = (1 << 1)`
- `STATUS_CURRENT_LIMIT = (1 << 2)`
- `STATUS_POWER_ON = (1 << 3)`
- `STATUS_POS_ERROR = (1 << 4)`
- `STATUS_HOME_SOURCE = (1 << 5)`
- `STATUS_LIMIT2 = (1 << 6)`
- `STATUS_HOME_IN_PROG = (1 << 7)`

---

### 2. ldcn_init.py (9,899 bytes)
**Purpose:** LDCN Network Initialization Utility

**Key Features:**
- Auto-detecting and resetting network
- Addressing all devices sequentially
- Switching to target baud rate
- Configuring I/O controller

**Class:**
- `LDCNInitializer`

**Key Methods:**
- `send_command(address, command, data=[])` - Send LDCN command
- `open_serial(baud)` - Open serial port at specified baud rate
- `auto_detect_baud()` - Auto-detect current baud rate
- `hard_reset()` - Send hard reset command
- `address_devices()` - Address all devices sequentially
- `verify_devices()` - Verify all devices are responding
- `change_baud_rate(new_baud)` - Change baud rate of all devices
- `configure_io_controller()` - Configure I/O controller for full status
- `initialize()` - Complete initialization sequence

**Baud Rate Definitions:**
```python
BRD_VALUES = {
    9600: 0x81,
    19200: 0x3F,
    57600: 0x14,
    115200: 0x0A,
    125000: 0x27,
    312500: 0x0F,
    625000: 0x07,
    1250000: 0x03
}
```

---

### 3. ldcn_monitor.py (7,038 bytes)
**Purpose:** LDCN Network Status Monitor

**Key Features:**
- Continuous monitoring of LDCN network status
- Communication status with all devices
- Power-on state from I/O controller
- Real-time status updates

**Class:**
- `LDCNMonitor`

**Key Methods:**
- `send_command(address, command, data=[])` - Send LDCN command
- `auto_detect_baud()` - Auto-detect current baud rate
- `configure_io_controller()` - Configure I/O controller for full status
- `read_device_status(address)` - Read status from a device
- `format_status_bits(status)` - Format status byte as bit string
- `print_header()` - Print monitor header
- `print_device_status(address, info)` - Print status for one device
- `monitor()` - Main monitoring loop

**Device Names:**
```python
device_names = {
    1: "Servo 1",
    2: "Servo 2",
    3: "Servo 3",
    4: "Servo 4",
    5: "Servo 5",
    6: "I/O Ctrl"
}
```

---

### 4. test_servo_init.py (11,574 bytes)
**Purpose:** Servo Initialization Testing (matches Python script used during debugging)

**Key Features:**
- Complete servo drive initialization sequence
- Mimics C code init_drive() behavior
- Power button detection
- Full network reset and addressing
- Baud rate auto-detection and upgrade
- Supervisor configuration and monitoring

**Functions:**
- `send_command(ser, address, command, data_bytes=[])` - Send LDCN command
- `parse_servo_status(response, status_bits)` - Parse servo status response with multi-byte data
- `initialize_servo(ser, addr)` - Complete servo initialization (7 steps)
  - Step 1: Define status reporting
  - Step 2: Set PID gains (kp=2, kd=50, ki=0, il=40, ol=255, el=2000, sr=20)
  - Step 3: Load initial trajectory
  - Step 4: Enable servo (STOP_AMP_ENABLE | STOP_ABRUPT)
  - Step 5: Reset position counter
  - Step 6: Clear sticky status bits
  - Step 7: Read and verify status
- `try_communicate_at_baud(port, baud)` - Test communication at specific baud
- `find_current_baud(port)` - Auto-detect current network baud rate
- `main()` - Complete test sequence with power button detection

**Servo Status Parsing:**
- Handles variable-length responses based on status_bits
- Parses: position (4B), ad_value (1B), velocity (2B), aux (1B), following_error (2B)
- Extracts all status flags from status byte

**PID Gains Used:**
```python
kp = 2       # Position gain
kd = 50      # Velocity gain
ki = 0       # Integral gain
il = 40      # Integration limit
ol = 255     # Output limit
cl = 0       # Current limit
el = 2000    # Position error limit
sr = 20      # Servo rate divisor
db = 0       # Deadband
```

---

### 5. test_position_command.py (4,745 bytes)
**Purpose:** Position Command Testing

**Key Features:**
- Tests sending position commands to servos
- Monitors position during motion
- Calculates motion error
- Uses trajectory mode with start_now flag

**Functions:**
- `send_command(ser, address, command, data_bytes=[])` - Send LDCN command
- `read_drive_position(ser, addr)` - Read current position and status
- `send_position_command(ser, addr, position_counts)` - Send position trajectory command
- `main()` - Test sequence: read initial position, move 1mm, monitor motion

**Trajectory Command Format:**
```python
traj_ctrl = 0x80 | 0x10  # start_now=1, servo_mode=1
# Data: control(1B), position(4B), velocity(4B), accel(4B)
```

**Test Parameters:**
- SCALE = 2000.0 counts/mm
- Test move: 1.0 mm (2000 counts)
- Monitor rate: 10 Hz for 2 seconds

---

## Common Protocol Constants

**Header:**
- `HEADER = 0xAA`

**Commands:**
- `CMD_NOP = 0x0E` - No operation / status read
- `CMD_HARD_RESET = 0x0F` - Hard reset all devices
- `CMD_SET_ADDRESS = 0x01` - Set device address
- `CMD_SET_BAUD = 0x0A` - Set baud rate
- `CMD_DEFINE_STATUS = 0x02` - Define status reporting

**Common Baud Rates:**
- `[19200, 125000, 115200, 57600, 9600, 38400]`

**Default Devices:**
- Addresses 1-5: Servo drives (LS-231SE)
- Address 6: I/O Controller (SK-2310g2)

---

### 6. integration_test.py (8,400 bytes)
**Purpose:** Comprehensive integration test suite for LDCN network validation

**⚠️ WARNING: EXTENSIVE FUNCTION OVERLAP** - This file duplicates most protocol functions from other utilities!

**Key Features:**
- Class-based test framework with pass/fail tracking
- ANSI color terminal output
- Complete integration test sequence (8 tests)
- Test assertion framework
- Summary reporting with pass rates

**Classes:**
- `Colors` - ANSI color codes for terminal output
- `LDCNIntegrationTest` - Test suite runner

**Test Methods:** (All contain DUPLICATE functionality)
- `test_01_auto_detect()` - Auto-detect baud rate
  - **OVERLAPS:** ldcn_diagnostic.auto_detect_baud(), ldcn_init.auto_detect_baud(), test_servo_init.find_current_baud()
- `test_02_hard_reset()` - Hard reset network
  - **OVERLAPS:** ldcn_init.hard_reset(), test_servo_init (embedded in main)
- `test_03_address_devices()` - Address all devices
  - **OVERLAPS:** ldcn_init.address_devices(), test_servo_init (embedded in main)
- `test_04_verify_communication()` - Verify all devices responding
  - **OVERLAPS:** ldcn_init.verify_devices(), ldcn_diagnostic.test_all_devices()
- `test_05_baud_rate_change()` - Change to 125kbps
  - **OVERLAPS:** ldcn_init.change_baud_rate()
- `test_06_io_controller_config()` - Configure supervisor
  - **OVERLAPS:** ldcn_init.configure_io_controller(), ldcn_monitor.configure_io_controller(), test_servo_init (embedded)
- `test_07_power_detection()` - Power button detection
  - **OVERLAPS:** ldcn_diagnostic.test_power_detection(), test_servo_init (embedded in main)
- `test_08_servo_enable()` - Enable/disable servo test
  - **OVERLAPS:** test_servo_init.initialize_servo() step 4

**Utility Methods:**
- `send_command(address, command, data=[])` - Send LDCN command
  - **OVERLAPS:** ALL other files have this exact function
- `log(msg, status)` - Formatted logging with colors
- `test_header(name)` - Print test section header
- `assert_true(condition, message)` - Test assertion
- `run_all_tests()` - Execute complete test sequence
- `print_summary()` - Display test results

**Test Sequence:**
1. Auto-detect current baud rate
2. Reset to 19200 if needed
3. Hard reset network
4. Address 6 devices (minimum 5 required)
5. Verify communication
6. Change baud rate to 125kbps
7. Configure I/O controller
8. Test power detection (baseline)
9. Test servo enable/disable

**Overlap Summary:**
- **100% protocol overlap** with ldcn_init.py
- **~80% overlap** with test_servo_init.py
- **~50% overlap** with ldcn_diagnostic.py
- Uses SAME `send_command()` as all other files

**Unique Features:**
- Test framework (assert_true, pass/fail tracking)
- Color-coded terminal output
- Comprehensive test summary with pass rate calculation
- Structured test sequence validation

**Target for Refactoring:**
This file should be rewritten to USE the new ldcn_network module rather than duplicating all protocol functions. It serves as an excellent validation that our module design should support all these operations.

---

## Function Overlap Analysis

### Critical Overlap: `send_command()`
**Found in:** ALL 6 files
**Exact same implementation in:**
- ldcn_diagnostic.py
- ldcn_init.py
- ldcn_monitor.py
- test_servo_init.py
- test_position_command.py
- integration_test.py

**Must consolidate to:** `LDCNNetwork.send_command()` - single source of truth

### Major Overlap: Auto-detect Baud Rate
**Found in:** 4 files
- ldcn_diagnostic.py: `auto_detect_baud()`
- ldcn_init.py: `auto_detect_baud()`
- test_servo_init.py: `find_current_baud(port)`
- integration_test.py: `test_01_auto_detect()`

**Slight variations:** Test order of baud rates, error handling
**Must consolidate to:** `LDCNNetwork.auto_detect_baud()` with configurable baud rate list

### Major Overlap: Hard Reset
**Found in:** 3 files
- ldcn_init.py: `hard_reset()`
- test_servo_init.py: (embedded in main)
- integration_test.py: `test_02_hard_reset()`

**Identical implementation:** `bytes([0xAA, 0xFF, 0x0F, 0x0E])`
**Must consolidate to:** `LDCNNetwork.reset()`

### Major Overlap: Address Devices
**Found in:** 3 files
- ldcn_init.py: `address_devices()`
- test_servo_init.py: (embedded in main)
- integration_test.py: `test_03_address_devices()`

**Identical logic:** Loop 1-6, send SET_ADDRESS(0x01) to address 0x00
**Must consolidate to:** `LDCNNetwork.address_devices(num_devices)`

### Moderate Overlap: Baud Rate Change
**Found in:** 2 files
- ldcn_init.py: `change_baud_rate(new_baud)`
- integration_test.py: `test_05_baud_rate_change()`

**Must consolidate to:** `LDCNNetwork.set_baud_rate(baud)`

### Moderate Overlap: I/O Controller Config
**Found in:** 4 files
- ldcn_init.py: `configure_io_controller()`
- ldcn_monitor.py: `configure_io_controller()`
- test_servo_init.py: (embedded in main)
- integration_test.py: `test_06_io_controller_config()`

**Identical:** `send_command(6, 0x02, [0xFF, 0xFF])`
**Must consolidate to:** `SK2310g2.configure()`

---

## Next Steps

1. ~~Extract all functions from test_servo_init.py~~ ✅
2. ~~Extract all functions from test_position_command.py~~ ✅
3. ~~Extract all functions from integration_test.py~~ ✅
4. ~~Categorize all functions by type~~ ✅
5. **Identify and document all function overlaps** ✅
6. Begin architecture design with overlap elimination as priority

---

## Notes

- **CRITICAL:** 6 of 6 files duplicate `send_command()` - THIS MUST BE CONSOLIDATED FIRST
- All utilities use identical serial communication patterns
- Common command building: `(num_data << 4) | (command & 0x0F)`
- Checksum calculation: `(address + cmd_byte + sum(data)) & 0xFF`
- All use 0.02s delay after sending commands
- All use similar auto-detection logic
- integration_test.py is essentially a test harness for ldcn_init.py functionality
