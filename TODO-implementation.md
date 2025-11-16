# pyldcn Implementation TODO List

## SK-2310g2 Missing Features

### 1. CMD_READ_OUTPUT (0x0E) Implementation

**Priority: High**

Currently, the `read_digital_outputs()` method is referenced in example code but does not exist in the implementation.

**Required Work:**
- Implement `CMD_READ_OUTPUT` protocol handler in `pyldcn/protocol.py` (verify command code 0x0E)
- Add `read_digital_outputs()` method to SK2310g2 class in `pyldcn/devices/io.py`
- Parse response Byte0 (digital outputs 1-8) and Byte1 (internal control outputs 9-16)
- Decode Byte1 bits according to SK-2310g2 manual:
  - Bit 0: Output9 (Internal control)
  - Bit 1: Guard Lock (0=Unlocked, 1=Locked)
  - Bit 2: Output11 (Internal control)
  - Bit 3: Output12 (Manual override inhibit)
  - Bit 4: Safety Link Bridge (0=Normal, 1=Bridged)
  - Bit 5: Output14 (Internal control)
  - Bit 6: Output15 (Guard unlock control when J19 shorted)
  - Bit 7: Power ON Command (Software power control when J21 shorted)
- Add helper method `read_output_states()` returning dictionary with named fields
- Update examples/read_sk2310g2_status.py to verify functionality
- **Hardware verification required** - test on physical SK-2310g2 device

### 2. Analog Output Control

**Priority: Medium**

The `set_analog_output()` method exists but raises `NotImplementedError`.

**Required Work:**
- Verify hardware supports analog outputs (check SK-2310g2 manual specifications)
- Implement `CMD_SET_ANALOG_OUTPUT` or equivalent command
- Add validation for output channel (0-2) and value range (0-255)
- Test PWM output on Output4 (Byte0/Bit3) if applicable
- Update method docstring with hardware-verified behavior
- **Hardware verification required** - confirm analog output functionality exists

### 3. Safety Monitoring Methods

**Priority: High** (safety-critical functionality)

Multiple safety monitoring methods are stubs raising `NotImplementedError`:

**read_estop_state():**
- Parse emergency stop status from diagnostic codes 0x10-0x11
- Detect contact A/B state and timing violations (>100ms transition = fault)
- Return dict with: `{'active': bool, 'contact_fault': bool, 'description': str}`

**read_guard_state():**
- Parse guard door states from diagnostic codes 0x14-0x1F
- Determine Guard-1 and Guard-2 open/closed status
- Detect guard contact faults (diagnostic 0x0E, flashing LED pattern)
- Return dict with: `{'guard1_closed': bool, 'guard2_closed': bool, 'contact_fault': bool, 'locked': bool}`

**read_safe_zone_state():**
- Parse safe state bit from Byte1/Bit0
- Check Zero Speed mode status (J10-1, J10-4 jumper configuration)
- Verify machine in safe zone (home sensor CN8 or zero speed automation)
- Return dict with: `{'safe_state': bool, 'zero_speed': bool, 'home_sensor': bool}`

**read_guard_lock_state():**
- Replace "TBD" placeholder code
- Read Guard Lock output from CMD_READ_OUTPUT Byte1/Bit1 (once implemented)
- Parse lock/unlock status based on J19 jumper configuration
- Return dict with: `{'locked': bool, 'unlock_enabled': bool, 'automation_mode': bool}`

**General Requirements:**
- Cross-reference diagnostic codes with safety states
- Implement comprehensive error detection for dual-contact failures
- Add timing validation where applicable (e.g., 100ms contact transition requirement)
- Ensure methods work in all jumper configurations (recipes 1-3 from documentation)
- **Hardware verification required** - test with actual guard switches, e-stop buttons, and home sensors
