#!/usr/bin/env python3
"""Test script for new diagnostic code structure."""

from pyldcn.devices.sk2310g2 import (
    DIAGNOSTIC_CODES,
    get_diagnostic_state,
    is_power_ready,
    is_power_enabled,
    is_guard_open,
    is_guard_closed,
    is_manual_override_active,
    is_safe_zone,
    is_spindle_stopped,
    get_error_type,
)


def test_diagnostic_structure():
    """Test that diagnostic code structure is working correctly."""

    print("=" * 80)
    print("DIAGNOSTIC CODE STRUCTURE TEST")
    print("=" * 80)

    # Test 1: Verify all 32 codes exist
    print("\nTest 1: Verify all 32 codes (0x00-0x1F) exist")
    assert len(DIAGNOSTIC_CODES) == 32, f"Expected 32 codes, got {len(DIAGNOSTIC_CODES)}"
    print(f"  ✓ All 32 diagnostic codes present")

    # Test 2: Verify all codes have valid descriptions
    print("\nTest 2: Verify all codes have valid descriptions")
    for code in range(0x00, 0x20):
        state = get_diagnostic_state(code)
        assert state is not None, f"Code 0x{code:02X} not found"
        assert state.condition, f"Code 0x{code:02X} has empty condition"
    assert get_diagnostic_state(0x00).condition == "Power OFF delay in progress"
    assert get_diagnostic_state(0x1F).condition == "Normal operation - All guards closed"
    print(f"  ✓ All codes have valid descriptions")

    # Test 3: Test code 0x1E (the example from the user)
    print("\nTest 3: Test code 0x1E attributes")
    state = get_diagnostic_state(0x1E)
    print(f"  Code: 0x{state.code:02X}")
    print(f"  Condition: {state.condition}")
    print(f"  Guard 1: {state.guard_1}")
    print(f"  Guard 2: {state.guard_2}")
    print(f"  Safe Zone: {state.safe_zone}")
    print(f"  Spindle Stopped: {state.spindle_stopped}")
    print(f"  Manual Override: {state.manual_override}")
    print(f"  Power Ready: {state.power_ready}")
    print(f"  Power Enabled: {state.power_enabled}")

    assert state.guard_1 == "OPEN", f"Expected Guard 1 OPEN, got {state.guard_1}"
    assert state.guard_2 == "CLOSED", f"Expected Guard 2 CLOSED, got {state.guard_2}"
    assert state.safe_zone == True, f"Expected safe_zone True, got {state.safe_zone}"
    assert state.spindle_stopped == True, f"Expected spindle_stopped True, got {state.spindle_stopped}"
    assert state.manual_override == False, f"Expected manual_override False, got {state.manual_override}"
    assert state.power_ready == "ON", f"Expected power_ready ON, got {state.power_ready}"
    assert state.power_enabled == "ON", f"Expected power_enabled ON, got {state.power_enabled}"
    print(f"  ✓ All attributes correct for 0x1E")

    # Test 4: Test query functions
    print("\nTest 4: Test query functions for code 0x1E")
    assert is_power_ready(0x1E) == True
    assert is_power_enabled(0x1E) == True
    assert is_guard_open(0x1E, 1) == True
    assert is_guard_open(0x1E, 2) == False
    assert is_guard_closed(0x1E, 1) == False
    assert is_guard_closed(0x1E, 2) == True
    assert is_manual_override_active(0x1E) == False
    assert is_safe_zone(0x1E) == True
    assert is_spindle_stopped(0x1E) == True
    assert get_error_type(0x1E) == None
    print(f"  ✓ All query functions work correctly")

    # Test 5: Test simple code (0x01 - Initializing)
    print("\nTest 5: Test simple code 0x01 (Initializing)")
    state = get_diagnostic_state(0x01)
    print(f"  Condition: {state.condition}")
    print(f"  Guard 1: {state.guard_1}")
    print(f"  Guard 2: {state.guard_2}")
    print(f"  Error Type: {state.error_type}")

    assert state.guard_1 == None, f"Expected guard_1 None, got {state.guard_1}"
    assert state.guard_2 == None, f"Expected guard_2 None, got {state.guard_2}"
    assert state.safe_zone == None
    assert state.spindle_stopped == None
    assert state.manual_override == None
    assert state.power_ready == "OFF"
    assert state.power_enabled == "OFF"
    assert state.error_type == "INIT"
    print(f"  ✓ Simple code has None for non-applicable attributes")

    # Test 6: Test fault code (0x0A - Safety Link Error)
    print("\nTest 6: Test fault code 0x0A (Safety Link Error)")
    assert get_error_type(0x0A) == "FAULT"
    assert is_power_ready(0x0A) == False
    assert is_power_enabled(0x0A) == False
    print(f"  ✓ Fault code correctly classified")

    # Test 7: Test manual override codes (0x18-0x1B)
    print("\nTest 7: Test manual override codes (0x18-0x1B)")
    for code in [0x18, 0x19, 0x1A, 0x1B]:
        assert is_manual_override_active(code) == True
        assert is_power_enabled(code) == True
    print(f"  ✓ Manual override codes correctly identified")

    # Test 8: Test normal operation (0x1F)
    print("\nTest 8: Test normal operation code 0x1F")
    state = get_diagnostic_state(0x1F)
    assert state.guard_1 == "CLOSED"
    assert state.guard_2 == "CLOSED"
    assert state.safe_zone == True
    assert state.manual_override == False
    assert state.power_ready == "ON"
    assert state.power_enabled == "ON"
    assert state.error_type == None
    print(f"  ✓ Normal operation code correct")

    print("\n" + "=" * 80)
    print("ALL TESTS PASSED!")
    print("=" * 80)


if __name__ == "__main__":
    test_diagnostic_structure()
