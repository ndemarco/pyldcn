# LS-231SE Servo Test Plan

**Author:** NickyDoes
**Date:** 2025-11-14
**Purpose:** Automated testing of servo implementation without manual intervention

---

## Test Levels

| Level | Category | Risk | Motion Required | Duration |
|-------|----------|------|-----------------|----------|
| 1 | Basic Status | Low | None | ~30s |
| 2 | Status Configuration | Low | None | ~45s |
| 3 | Simple Motion | Low | Minimal (<500 counts) | ~60s |
| 4 | Advanced Motion | Medium | Moderate | ~120s |
| 5 | Integration | Medium | Full range | ~180s |

---

## Level 1: Basic Status Tests

**File:** `test_servo_basic.py`
**Requirements:** Power on, no motion needed
**Safety:** Can run without motor power

### Test Classes

| Test Class | Tests | Purpose |
|------------|-------|---------|
| `TestServoEnableDisable` | 4 | Verify enable/disable state transitions |
| `TestStatusParsing` | 5 | Verify status byte parsing and flag extraction |
| `TestDiagnosticMatching` | 5 | Verify diagnostic condition pattern matching |
| `TestStopCommand` | 2 | Verify stop command acceptance |
| `TestFaultDetection` | 3 | Verify fault detection and clearing |

### Key Tests

| Test | What It Validates | Expected Result |
|------|-------------------|-----------------|
| `test_initial_state()` | Initial power-up condition | Detects "No Motor Power after LDCN Init" |
| `test_enable_detection()` | Enable command effect | Detects power or servo_on flag change |
| `test_disable_detection()` | Disable command effect | Detects power or servo_on flag change |
| `test_enable_disable_cycle()` | Complete state cycle | Sees at least 2 different states |
| `test_status_byte_reading()` | Status byte read | Returns valid 8-bit integer |
| `test_flag_extraction()` | Individual flag parsing | All flags are boolean |
| `test_condition_detection()` | Diagnostic matching | Detects valid condition |
| `test_operational_check()` | Ready/faulted check | Not both ready and faulted |
| `test_layer_access()` | Three-layer architecture | Can access all 3 layers |
| `test_stop_command_accepted()` | Stop command | Completes without error |
| `test_fault_detection()` | Fault state detection | Returns fault status |
| `test_clear_faults()` | Fault clearing | Clears sticky bits |

### Pass Criteria
- All tests pass
- No exceptions raised
- Enable/disable causes detectable state change
- Diagnostic condition detected for all states
- Stop command accepted

---

## Level 2: Status Configuration Tests

**File:** `test_servo_status.py`
**Requirements:** Power on, no motion needed
**Safety:** Can run without motor power

### Test Classes

| Test Class | Tests | Purpose |
|------------|-------|---------|
| `TestDefineStatus` | 5 | DEFINE_STATUS command functionality |
| `TestStatusParsing` | 4 | Status response parsing |
| `TestStatusCaching` | 3 | State caching in ServoState |
| `TestStatusPerformance` | 2 | Status read timing |

### Key Tests

| Test | What It Validates | Expected Result |
|------|-------------------|-----------------|
| `test_define_status_command()` | DEFINE_STATUS sending | Command accepted |
| `test_custom_status_read()` | Custom item reading | Returns requested items |
| `test_full_status_read()` | All items reading | Returns all available items |
| `test_position_only_read()` | Fast position read | Returns position quickly |
| `test_status_mask_variations()` | Various mask configs | All configs work |
| `test_status_flag_parsing()` | Flag dictionary parsing | Returns valid flag dict |
| `test_position_parsing()` | 32-bit signed position | Returns signed integer |
| `test_state_updates_on_read()` | State caching | State updated after read |
| `test_property_access()` | Property accessors | All properties accessible |
| `test_status_read_timing()` | Read performance | Position-only faster than full |
| `test_status_update_rate()` | Achievable update rate | Measures Hz |

### Pass Criteria
- DEFINE_STATUS command accepted
- Custom status reads return requested items
- Position-only read faster than full status
- State properly cached and accessible
- Update rate >10 Hz

---

## Level 3: Simple Motion Tests

**File:** `test_servo_motion.py`
**Requirements:** Motor enabled, small moves allowed
**Safety:** Small moves (<500 counts), immediate stops

### Test Classes

| Test Class | Tests | Purpose |
|------------|-------|---------|
| `TestPositionReset` | 4 | Position counter reset |
| `TestStopEffectiveness` | 3 | Stop command timing and effectiveness |
| `TestMoveDoneFlag` | 3 | Move_done flag transitions |
| `TestSmallMoves` | 3 | Small controlled moves |
| `TestMoveDirection` | 2 | Forward/reverse moves |

### Key Tests

| Test | What It Validates | Expected Result |
|------|-------------------|-----------------|
| `test_reset_to_zero()` | Reset to 0 | Position == 0 after reset |
| `test_reset_to_custom_value()` | Reset to 12345 | Position == 12345 |
| `test_reset_negative_value()` | Reset to -5000 | Position == -5000 |
| `test_multiple_resets()` | Sequential resets | All resets work |
| `test_stop_immediately()` | Stop when stationary | move_done true |
| `test_move_and_stop()` | Start move then stop | Stops before target or completes |
| `test_stop_timing()` | Stop command latency | Measures stop time |
| `test_initial_move_done()` | Initial state | move_done true when stationary |
| `test_move_done_transition()` | move_done during move | Captures state changes |
| `test_small_move_command()` | 50-count move | Command accepted |
| `test_position_tracking()` | Position updates | Position changes during move |
| `test_positive_move()` | +100 count move | Moves in positive direction |
| `test_negative_move()` | -100 count move | Moves in negative direction |

### Pass Criteria
- Position reset works for 0, positive, negative values
- Stop command effective within reasonable time
- move_done flag transitions correctly
- Position tracking shows movement
- Both directions work

---

## Level 4: Advanced Motion Tests

**File:** `test_servo_advanced.py` (future)
**Requirements:** Motor enabled, full motion range
**Safety:** Requires limit switches or safe workspace

### Planned Tests

| Test Category | Tests | Purpose |
|---------------|-------|---------|
| Homing | 4 | Limit and index homing sequences |
| Path Mode | 3 | Path point execution |
| I/O Control | 3 | Brake and digital I/O |
| Safety Features | 3 | Watchdog, limit stop |
| Coordinated Motion | 2 | Multi-axis synchronization |

---

## Level 5: Integration Tests

**File:** `test_servo_integration.py` (future)
**Requirements:** Full system, multiple axes
**Safety:** Controlled environment

### Planned Tests

| Test Category | Tests | Purpose |
|---------------|-------|---------|
| Multi-Axis | 3 | Synchronized motion |
| Fault Recovery | 3 | Automatic fault handling |
| Long-Running | 2 | Extended operation |
| Configuration | 2 | Settings persistence |

---

## Running Tests

### Prerequisites

```bash
# Install pytest if not already installed
pip install pytest

# Connect servo at address 1 on /dev/ttyUSB0
# Ensure motor power is available (for motion tests)
```

### Run All Tests

```bash
# From pyldcn root directory
pytest tests/ -v
```

### Run Specific Level

```bash
# Level 1 - Basic (safest, no motion)
pytest tests/test_servo_basic.py -v -s

# Level 2 - Status configuration
pytest tests/test_servo_status.py -v -s

# Level 3 - Simple motion (requires motor power)
pytest tests/test_servo_motion.py -v -s
```

### Run Specific Test Class

```bash
pytest tests/test_servo_basic.py::TestServoEnableDisable -v -s
```

### Run Specific Test

```bash
pytest tests/test_servo_basic.py::TestServoEnableDisable::test_enable_detection -v -s
```

---

## Test Results Format

### Pass Example
```
test_servo_basic.py::TestServoEnableDisable::test_enable_detection PASSED

  Initial power state: False
  Enabling servo...
  After enable: power=True, servo_on=True
```

### Fail Example
```
test_servo_basic.py::TestServoEnableDisable::test_enable_detection FAILED

AssertionError: Should detect enable command effect
```

---

## Safety Considerations

### Before Running Tests

| Check | Requirement |
|-------|-------------|
| Workspace | Clear of obstructions |
| Limits | Limit switches functional (if installed) |
| E-Stop | Emergency stop accessible |
| Gains | PID gains appropriate for motor |
| Power | Motor power supply on (for motion tests) |

### During Tests

- Monitor servo operation
- Be ready to power off if unexpected behavior
- Start with Level 1 tests (no motion) first
- Only proceed to motion tests when Level 1/2 pass

### Test Failures

If tests fail:
1. Check physical connections
2. Verify servo address (default: 1)
3. Check serial port (default: /dev/ttyUSB0)
4. Verify motor power (for motion tests)
5. Review servo diagnostic condition
6. Check for fault states

---

## Expected Test Duration

| Test File | Tests | Duration | Notes |
|-----------|-------|----------|-------|
| `test_servo_basic.py` | 19 | ~30s | No motion required |
| `test_servo_status.py` | 14 | ~45s | No motion required |
| `test_servo_motion.py` | 15 | ~60s | Small moves only |
| **Total (Levels 1-3)** | **48** | **~2.5 min** | Safe for initial testing |

---

## Test Coverage

### Commands Tested

| Command | Test Level | Coverage |
|---------|-----------|----------|
| Enable/Disable | 1 | ✓ Full |
| Stop | 1, 3 | ✓ Full |
| Read Status | 1, 2, 3 | ✓ Full |
| Define Status | 2 | ✓ Full |
| Reset Position | 3 | ✓ Full |
| Move To Counts | 3 | ✓ Basic |
| Clear Faults | 1 | ✓ Full |
| Load Gains | All | ✓ Setup only |

### Subsystems Tested

| Subsystem | Test Level | Coverage |
|-----------|-----------|----------|
| Status (`servo_status`) | 1, 2 | ✓ Full |
| Motion (`servo_motion`) | 3 | ✓ Basic |
| Diagnostics (`servo_diagnostics`) | 1 | ✓ Full |
| State (`servo_state`) | 1, 2 | ✓ Full |
| I/O (`servo_io`) | - | Future |
| Safety (`servo_safety`) | - | Future |

---

## CI/CD Integration

### Automated Testing

```yaml
# .github/workflows/test.yml
name: Servo Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Install dependencies
        run: pip install pytest pyserial
      - name: Run basic tests (no hardware)
        run: pytest tests/test_servo_diagnostics_unit.py
```

### Hardware-in-Loop Testing

For tests requiring actual hardware:
- Run on dedicated test bench
- Schedule during off-hours
- Automatic email on failures

---

## Maintenance

### Adding New Tests

1. Choose appropriate test level (1-5)
2. Add to relevant test file
3. Update this plan document
4. Document pass criteria
5. Update coverage table

### Test Review Schedule

- Weekly: Review failed tests
- Monthly: Update test plan
- Quarterly: Add new test coverage
- Annually: Full test suite review
