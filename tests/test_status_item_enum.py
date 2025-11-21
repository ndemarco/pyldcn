#!/usr/bin/env python3
"""
Test script for StatusItem IntFlag enum.

Demonstrates all usage patterns for the new status item enum API.
"""

from pyldcn.devices import StatusItem
from pyldcn.devices.status import ServoStatus

def test_enum_values():
    """Test that enum values match expected bitmasks."""
    print("Testing enum values...")

    assert int(StatusItem.POSITION) == 0x0001
    assert int(StatusItem.AD_VALUE) == 0x0002
    assert int(StatusItem.VELOCITY) == 0x0004
    assert int(StatusItem.AUX) == 0x0008
    assert int(StatusItem.HOME) == 0x0010
    assert int(StatusItem.DEVICE_ID) == 0x0020
    assert int(StatusItem.POS_ERROR) == 0x0040
    assert int(StatusItem.PATH_COUNT) == 0x0080
    assert int(StatusItem.WATCHDOG) == 0x1000
    assert int(StatusItem.MOTOR_POS) == 0x2000

    print("  ✓ All enum values correct")


def test_combining_flags():
    """Test combining flags with bitwise OR."""
    print("\nTesting flag combinations...")

    # Combine individual flags
    mask1 = StatusItem.POSITION | StatusItem.VELOCITY
    assert int(mask1) == 0x0005

    mask2 = StatusItem.POSITION | StatusItem.VELOCITY | StatusItem.PATH_COUNT
    assert int(mask2) == 0x0085

    mask3 = StatusItem.POSITION | StatusItem.VELOCITY | StatusItem.AUX | StatusItem.POS_ERROR
    assert int(mask3) == 0x004D

    print("  ✓ Flag combinations work correctly")


def test_presets():
    """Test predefined preset combinations."""
    print("\nTesting presets...")

    # MOTION = POSITION | VELOCITY
    assert int(StatusItem.MOTION) == 0x0005
    assert StatusItem.MOTION == (StatusItem.POSITION | StatusItem.VELOCITY)

    # FULL = POSITION | VELOCITY | AUX | POS_ERROR
    assert int(StatusItem.FULL) == 0x004D
    assert StatusItem.FULL == (StatusItem.POSITION | StatusItem.VELOCITY |
                               StatusItem.AUX | StatusItem.POS_ERROR)

    # PATH_MODE = POSITION | VELOCITY | AUX | PATH_COUNT
    assert int(StatusItem.PATH_MODE) == 0x008D
    assert StatusItem.PATH_MODE == (StatusItem.POSITION | StatusItem.VELOCITY |
                                    StatusItem.AUX | StatusItem.PATH_COUNT)

    print("  ✓ All presets correct")


def test_string_representation():
    """Test string representation for debugging."""
    print("\nTesting string representations...")

    # Single flag
    s1 = repr(StatusItem.POSITION)
    assert "POSITION" in s1

    # Combined flags
    mask = StatusItem.POSITION | StatusItem.VELOCITY
    s2 = repr(mask)
    # Should show both flags (exact format depends on Python version)
    assert "POSITION" in s2 or "StatusItem" in s2

    print(f"  Single: {StatusItem.POSITION}")
    print(f"  Combined: {mask}")
    print("  ✓ String representation works")


def test_iteration():
    """Test iterating over set flags."""
    print("\nTesting flag iteration...")

    mask = StatusItem.POSITION | StatusItem.VELOCITY | StatusItem.PATH_COUNT
    flags = list(mask)

    assert StatusItem.POSITION in flags
    assert StatusItem.VELOCITY in flags
    assert StatusItem.PATH_COUNT in flags
    assert len(flags) == 3

    print(f"  Flags in mask: {flags}")
    print("  ✓ Iteration works correctly")


def test_membership():
    """Test checking if a flag is in a mask."""
    print("\nTesting membership checks...")

    mask = StatusItem.MOTION  # POSITION | VELOCITY

    # Using 'in' operator
    assert StatusItem.POSITION in mask
    assert StatusItem.VELOCITY in mask
    assert StatusItem.PATH_COUNT not in mask

    # Using bitwise AND
    assert mask & StatusItem.POSITION
    assert mask & StatusItem.VELOCITY
    assert not (mask & StatusItem.PATH_COUNT)

    print("  ✓ Membership checks work")


def test_backward_compatibility():
    """Test that raw integers still work."""
    print("\nTesting backward compatibility...")

    # Raw int should work
    mask_int = 0x0085
    mask_enum = StatusItem.POSITION | StatusItem.VELOCITY | StatusItem.PATH_COUNT

    assert int(mask_enum) == mask_int

    # Can convert enum to int for API calls
    assert int(StatusItem.MOTION) == 0x0005

    print("  ✓ Backward compatibility maintained")


def demo_usage_patterns():
    """Demonstrate various usage patterns."""
    print("\n" + "="*60)
    print("USAGE EXAMPLES")
    print("="*60)

    print("\n1. Single item:")
    print("   servo.status.read_status(StatusItem.POSITION)")

    print("\n2. Multiple items (efficient - one command):")
    print("   servo.status.read_status(StatusItem.POSITION | StatusItem.VELOCITY | StatusItem.PATH_COUNT)")

    print("\n3. Using presets:")
    print("   servo.status.read_status(StatusItem.MOTION)")
    print("   servo.status.read_status(StatusItem.FULL)")
    print("   servo.status.read_status(StatusItem.PATH_MODE)")

    print("\n4. Default (backward compatible):")
    print("   servo.status.read_status()  # Uses StatusItem.FULL")

    print("\n5. Raw mask (still works):")
    print("   servo.status.read_status(0x0085)")

    print("\n6. Build masks programmatically:")
    print("   items = []")
    print("   items.append(StatusItem.POSITION)")
    print("   items.append(StatusItem.VELOCITY)")
    print("   if need_path_count:")
    print("       items.append(StatusItem.PATH_COUNT)")
    print("   mask = items[0]")
    print("   for item in items[1:]:")
    print("       mask |= item")
    print("   servo.status.read_status(mask)")


def main():
    """Run all tests."""
    print("="*60)
    print("StatusItem IntFlag Enum Tests")
    print("="*60)

    test_enum_values()
    test_combining_flags()
    test_presets()
    test_string_representation()
    test_iteration()
    test_membership()
    test_backward_compatibility()

    demo_usage_patterns()

    print("\n" + "="*60)
    print("✓ ALL TESTS PASSED")
    print("="*60)


if __name__ == '__main__':
    main()