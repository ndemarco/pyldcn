# pyldcn Implementation TODO List

## SK-2310g2 Missing Features

### 1. Output State Tracking (No Hardware Read Support)

**Priority: High**

**Background:**
Hardware does not support reading outputs (CMD 0x0E is NOP). Outputs are write-only and must be tracked in software ("shadow state"). SK-2310g2 has special functions (Guard Lock, Safety Link Bridge, Power ON) that override written values and must be inferred from diagnostic codes and status bits.

**Remaining Work:**

**IOController Base Class** (`pyldcn/devices/io.py`):
- ❌ Hook `_reset_shadow_state()` into `CMD_HARD_RESET` handler

**SK2310g2 Device Class** (`pyldcn/devices/io.py`):
- ❌ Implement `_infer_guard_lock_state()` using diagnostic codes
- ❌ Implement `_infer_power_on_state()` using diagnostic codes
- ❌ Implement `_infer_safety_link_bridge_state()` (always 0 if spindle enabled)
- ❌ Document which outputs are write-only vs. inferred vs. tracked
- ❌ Add docstring warnings about inference limitations

**Documentation:**
- ❌ Document shadow state system and special function inference
- ❌ List jumper dependencies for special functions

**Testing:**
- ❌ Unit tests for output state tracking (write then read)
- ❌ Integration tests comparing inferred states with known diagnostic codes
- ❌ Hardware verification: Set outputs, read diagnostics, verify inference accuracy

### 2. Analog I/O Implementation (PWM Outputs)

**Priority: Medium**

**Background:**
PWM outputs use inverted logic (0=100% ON, 255=OFF). `CMD_SET_PWM_IO (0x04)` takes 2 bytes `[pwm1, pwm2]`. Base implementation complete with shadow state tracking.

**Remaining Work:**

**IOController Base Class:**
- ❌ Hook PWM reset into `CMD_HARD_RESET` handler

**SK-2310g2 Spindle PWM Helpers:**
- ❌ Implement `set_spindle_speed_pwm(value)` - Sets PWM1 raw value (0-255, inverted)
- ❌ Implement `set_spindle_speed_percent(percent)` - Converts 0-100% to inverted PWM
  - Formula: `pwm_value = int(255 - (percent / 100.0 * 255))`
- ❌ Implement `read_spindle_load()` - Reads AIN0 raw value (0-255)
- ❌ Implement `read_spindle_load_percent()` - Converts AIN0 to percentage
  - Formula: `percent = (ain0_value / 255.0) * 100.0`

**General Purpose Analog Input Helpers:**
- ❌ Implement `read_analog_percent(channel)` - Converts ADC to percentage
  - Formula: `percent = (adc_value / 255.0) * 100.0`

**Documentation Inconsistencies:**

**CRITICAL - CMD_READ_OUTPUT does not exist:**
- ❌ `docs/SK-2310g2_supervisor.md:360`: Entire section "CMD_READ_OUTPUT (0x0E)" is WRONG
  - Claims 0x0E reads output states - FALSE, 0x0E is NOP for I/O controllers
  - References at lines 113, 609, 647 also wrong
  - Section should be deleted or rewritten to explain shadow state approach
- ❌ `docs/io_commands.md:75`: Correctly states "No command exists to read outputs" but contradicts supervisor doc

**Voltage references (should be raw 0-255 values):**
- ❌ Docs: `sk-2310g2_io_mapping.md` lines 34, 37, 215-217, 227-229, 236, 280-281, 420-421, 464
- ❌ Docs: `io_commands.md` lines 12, 47-48, 56-58, 62
- ❌ Code: `read_analog_inputs()` (line 1243) returns voltages instead of raw 0-255
- ❌ Code: `set_analog_output()` (line 1267) accepts voltage instead of raw PWM 0-255
- ❌ Code: `set_spindle_speed_voltage()` (line 1296) voltage API conflicts with raw PWM
- ❌ Code: `set_spindle_speed_percent()` (line 1314) correct approach but needs PWM backend

**PWM vs Digital Output confusion:**
- ❌ `docs/io_commands.md:150`: Says "OUTPUT 1 and OUTPUT 2 must be enabled via Set Outputs first"
- ❌ `docs/io_commands.md:50`: Says SK-2310g2 has "OUTPUT 4 (20 KHz PWM)"
- ❌ Clarify: Are PWM1/PWM2 independent analog outputs or tied to digital outputs?

**Missing documentation:**
- ❌ Document that PWM1 and AIN0 are spindle-dedicated in SK-2310g2
- ❌ Document that AIN1 and AIN2 are general purpose
- ❌ Add note: "For voltage calculations, user must calibrate based on hardware"

**Testing:**
- ❌ Unit tests for percent conversions (0%, 50%, 100%)
- ❌ Unit tests for PWM inverted logic (0→100%, 255→0%)
- ❌ Hardware verification: Set PWM values, measure duty cycle, verify inverted logic

### 3. Safety Monitoring Methods

**Priority: High** (safety-critical functionality)

**Background:**
Multiple safety monitoring methods are stubs raising `NotImplementedError`. These parse diagnostic codes and status bits to determine safety states.

**Remaining Work:**

**Emergency Stop Monitoring:**
- ❌ Implement `read_estop_state()` - Parse diagnostic codes 0x10-0x11
- ❌ Detect contact A/B state and timing violations (>100ms transition = fault)
- ❌ Return: `{'active': bool, 'contact_fault': bool, 'description': str}`

**Guard Door Monitoring:**
- ❌ Implement `read_guard_state()` - Parse diagnostic codes 0x14-0x1F
- ❌ Determine Guard-1 and Guard-2 open/closed status
- ❌ Detect guard contact faults (diagnostic 0x0E, flashing LED pattern)
- ❌ Return: `{'guard1_closed': bool, 'guard2_closed': bool, 'contact_fault': bool, 'locked': bool}`

**Safe Zone Monitoring:**
- ❌ Implement `read_safe_zone_state()` - Parse safe state bit from Byte1/Bit0
- ❌ Check Zero Speed mode status (J10-1, J10-4 jumper configuration)
- ❌ Verify machine in safe zone (home sensor CN8 or zero speed automation)
- ❌ Return: `{'safe_state': bool, 'zero_speed': bool, 'home_sensor': bool}`

**Guard Lock State:**
- ❌ Update `read_guard_lock_state()` to use shadow state (remove "TBD" placeholder)
- ❌ Parse lock/unlock status based on J19 jumper configuration
- ❌ Return: `{'locked': bool, 'unlock_enabled': bool, 'automation_mode': bool}`

**Testing:**
- ❌ Cross-reference diagnostic codes with safety states
- ❌ Implement error detection for dual-contact failures
- ❌ Add timing validation (e.g., 100ms contact transition requirement)
- ❌ Ensure methods work in all jumper configurations (recipes 1-3 from documentation)
- ❌ Hardware verification: test with actual guard switches, e-stop buttons, and home sensors
