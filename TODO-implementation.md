# pyldcn Implementation TODO List

## SK-2310g2 Missing Features

### 1. Output State Tracking (No Hardware Read Support)

**Priority: High**

**Problem Analysis:**

There is **NO hardware command** to read output states from LDCN I/O devices:
- **CMD 0x0E is NOP** - Returns status data, not output states
- **CMD_DEFINE_STATUS (0x02)** - Does not return output status
- **Hardware does not support reading outputs** - Outputs are write-only

**Output State Determination:**

Output states must be **tracked in software** based on:

1. **Initial State** - State when device initialized
2. **Explicit Write Operations** - All `CMD_SET_OUTPUTS`, `CMD_WRITE_OUTPUT` commands
3. **Special Function Side Effects** - Some outputs change based on system state

**Device-Specific Complexity:**

**LS-773 (Generic I/O):**
- Simple case: Outputs = last written state
- Track byte0 and byte1 from all write operations

**SK-2310g2 (Supervisory Controller):**
- Complex case: Some outputs have **special functions** that override written values
- Special function behavior depends on **jumper configuration** (J10, J16, J19, J20, J21, etc.)
- Examples of special functions:
  - **Output 9 (Byte1/Bit1)**: Guard Lock - changes based on safe state, manual override, spindle stopped
  - **Output 12 (Byte1/Bit4)**: Safety Link Bridge - safety-critical, interlock with spindle
  - **Output 15 (Byte1/Bit7)**: Power ON - affected by J21 jumper, power button state
- Special function state can be **inferred** from:
  - Diagnostic codes (0x00-0x1F)
  - Input status (Byte1 safe_state, manual_override, spindle_stopped)
  - System mode (power state, guard states)
  - Jumper configuration (if known)

**Implementation Strategy:**

**Phase 1: Base Output State Tracker (IOController class)**
- Track basic output state (byte0, byte1) in `IOController` base class
- Intercept all output write methods (`set_outputs()`, `set_output_bit()`, etc.)
- Store shadow state in `self._output_state = {'byte0': 0x00, 'byte1': 0x00}`
- Implement `read_digital_outputs()` returning tracked state
- Implement `get_output_bit(bit)` for individual bit queries

**Phase 2: SK-2310g2 Special Function Inference (SK2310g2 class)**
- Override `read_digital_outputs()` to merge tracked state with inferred special functions
- Infer special function output states from:
  - `diagnostic_code` - Guard Lock state from codes 0x14-0x1F
  - `status['safe_state']` - Safe state affects Guard Lock
  - `status['manual_override']` - Manual override affects multiple outputs
  - `status['spindle_stopped']` - Spindle state affects Guard Lock unlock
- Document jumper dependencies for each special function
- Mark inferred values with confidence level (definite vs. probable)

**Phase 3: Jumper Configuration Tracking (Future)**
- Add optional jumper configuration to SK2310g2 initialization
- Use jumper config to improve special function inference accuracy
- Validate inferred states against jumper-dependent behaviors

**Required Work:**

**IOController Base Class** (`pyldcn/devices/io.py`):
- Add `_output_state` dictionary to `__init__()`
- Intercept `set_outputs()` to update `_output_state`
- Implement `read_digital_outputs()` returning `_output_state`
- Implement `get_output_bit(bit)` helper method
- Add `reset_output_state()` for initialization/reset scenarios

**SK2310g2 Device Class** (`pyldcn/devices/io.py`):
- Override `read_digital_outputs()` to add special function inference
- Implement `_infer_guard_lock_state()` using diagnostic codes
- Implement `_infer_power_state()` using diagnostic codes
- Implement `_infer_safety_link_bridge()` (always 0 if spindle enabled)
- Document which outputs are write-only vs. inferred vs. tracked
- Add docstring warnings about inference limitations

**Documentation Requirements:**
- Document that outputs are **NOT readable from hardware**
- Explain tracked vs. inferred output states
- List jumper dependencies for special functions
- Warn about state desync if outputs changed externally (impossible but document)

**Testing Strategy:**
- Unit tests for output state tracking (write then read)
- Integration tests comparing inferred states with known diagnostic codes
- **Hardware verification**: Set outputs, read diagnostics, verify inference accuracy

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
