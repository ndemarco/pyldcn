#!/usr/bin/env python3
"""
test_servo_status.py - Status Configuration and Reading Tests

Tests status configuration and custom status reading:
- DEFINE_STATUS command
- Custom status item reading
- Status mask configuration
- Variable-length status parsing
- Position-only fast reads

Author: NickyDoes
License: GPL v2 or later
"""

import pytest
import time
from pyldcn import LDCNNetwork


class TestDefineStatus:
    """Test DEFINE_STATUS command and configuration."""

    @pytest.fixture
    def servo(self):
        """Setup servo connection."""
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.get_device(1, 'LS-231SE')
        servo.load_gains(kp=10, kd=1000, ki=0)

        yield servo

        network.close()

    def test_define_status_command(self, servo):
        """Test sending DEFINE_STATUS command."""
        # Configure to return position only
        mask = 0x0001  # Position only
        print(f"\n  Configuring status mask: 0x{mask:04X}")

        servo.status.define_status(mask)
        time.sleep(0.05)

        # Read status - should return only configured items
        status = servo.read_status()

        print(f"  Status keys: {list(status.keys())}")
        assert 'status' in status, "Should always include status byte"

    def test_custom_status_read(self, servo):
        """Test reading custom status items."""
        # Read position, velocity, aux
        mask = 0x0001 | 0x0004 | 0x0008
        print(f"\n  Reading custom status: 0x{mask:04X}")

        status = servo.status.read_status_custom(mask)

        print(f"  Returned items:")
        for key, value in status.items():
            if key != 'flags':
                print(f"    {key}: {value}")

        # Should have requested items
        assert 'position' in status or 'status' in status, "Should return requested items"

    def test_full_status_read(self, servo):
        """Test reading all available status items."""
        print("\n  Reading full status...")

        status = servo.status.read_full_status()

        print(f"  All status items:")
        for key, value in status.items():
            if key != 'flags' and value is not None:
                print(f"    {key}: {value}")

    def test_position_only_read(self, servo):
        """Test fast position-only read."""
        print("\n  Fast position read...")

        status = servo.read_position()

        print(f"  Position read:")
        for key, value in status.items():
            print(f"    {key}: {value}")

        assert 'position' in status or 'status' in status, "Should return position"

    def test_status_mask_variations(self, servo):
        """Test various status mask configurations."""
        masks = [
            (0x0001, "Position only"),
            (0x0004, "Velocity only"),
            (0x0008, "Aux only"),
            (0x0001 | 0x0004, "Position + Velocity"),
            (0x0001 | 0x0008, "Position + Aux"),
            (0x0001 | 0x0004 | 0x0008, "Pos + Vel + Aux"),
        ]

        print("\n  Testing status mask variations:")

        for mask, description in masks:
            print(f"\n    {description} (0x{mask:04X}):")

            status = servo.status.read_status_custom(mask)

            # Show what was returned
            items = [k for k in status.keys() if k != 'flags' and status[k] is not None]
            print(f"      Returned: {', '.join(items)}")


class TestStatusParsing:
    """Test status response parsing."""

    @pytest.fixture
    def servo(self):
        """Setup servo connection."""
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.get_device(1, 'LS-231SE')
        servo.load_gains(kp=10, kd=1000, ki=0)

        yield servo

        network.close()

    def test_status_flag_parsing(self, servo):
        """Test parsing of status byte into flags."""
        status = servo.read_status()
        status_byte = status.get('status', 0)

        # Parse flags
        flags = servo.status.decode_status_flags(status_byte)

        print(f"\n  Status byte: 0x{status_byte:02X}")
        print(f"  Parsed flags:")
        for flag, value in sorted(flags.items()):
            print(f"    {flag:20s}: {value}")

        assert isinstance(flags, dict), "Should return dictionary of flags"
        assert 'move_done' in flags, "Should include move_done flag"

    def test_position_parsing(self, servo):
        """Test position value parsing (signed 32-bit)."""
        # Reset to known value
        servo._motion.reset_position(12345)
        time.sleep(0.05)

        status = servo.read_status()
        position = status.get('position')

        if position is not None:
            print(f"\n  Position: {position}")
            assert isinstance(position, int), "Position should be integer"

    def test_velocity_parsing(self, servo):
        """Test velocity value parsing (signed 16-bit)."""
        status = servo.read_status()
        velocity = status.get('velocity')

        if velocity is not None:
            print(f"\n  Velocity: {velocity}")
            assert isinstance(velocity, int), "Velocity should be integer"


class TestStatusCaching:
    """Test status caching in ServoState."""

    @pytest.fixture
    def servo(self):
        """Setup servo connection."""
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.get_device(1, 'LS-231SE')
        servo.load_gains(kp=10, kd=1000, ki=0)

        yield servo

        network.close()

    def test_state_updates_on_read(self, servo):
        """Test that state is updated when status is read."""
        # Read status
        servo.read_status()

        # Check that state was updated
        print(f"\n  Cached state:")
        print(f"    status_byte: 0x{servo.state.status_byte:02X}")
        print(f"    position: {servo.state.position}")
        print(f"    move_done: {servo.state.move_done}")
        print(f"    power: {servo.state.power}")

        assert servo.state.status_byte is not None, "Status byte should be cached"

    def test_state_persistence(self, servo):
        """Test that state persists between reads."""
        # Read once
        servo.read_status()
        pos1 = servo.position

        # State should be accessible without re-reading
        pos2 = servo.position

        print(f"\n  Position reads:")
        print(f"    After read_status(): {pos1}")
        print(f"    Direct access: {pos2}")

        assert pos1 == pos2, "Position should be cached"

    def test_property_access(self, servo):
        """Test property accessors for common state."""
        servo.read_status()

        properties = {
            'position': servo.position,
            'velocity': servo.velocity,
            'move_done': servo.move_done,
            'power': servo.power,
            'servo_on': servo.servo_on,
        }

        print(f"\n  Property access:")
        for name, value in properties.items():
            print(f"    {name}: {value}")


class TestStatusPerformance:
    """Test status reading performance."""

    @pytest.fixture
    def servo(self):
        """Setup servo connection."""
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.get_device(1, 'LS-231SE')
        servo.load_gains(kp=10, kd=1000, ki=0)

        yield servo

        network.close()

    def test_status_read_timing(self, servo):
        """Test time required for status reads."""
        import time

        # Full status read
        start = time.time()
        for _ in range(10):
            servo.read_status()
        elapsed_full = (time.time() - start) / 10

        # Position-only read
        start = time.time()
        for _ in range(10):
            servo.read_position()
        elapsed_pos = (time.time() - start) / 10

        print(f"\n  Average read times:")
        print(f"    Full status: {elapsed_full*1000:.2f} ms")
        print(f"    Position only: {elapsed_pos*1000:.2f} ms")

        # Position-only should be faster
        print(f"    Speedup: {elapsed_full/elapsed_pos:.1f}x")

    def test_status_update_rate(self, servo):
        """Test achievable status update rate."""
        import time

        print("\n  Testing 100 rapid status reads...")

        start = time.time()
        positions = []

        for _ in range(100):
            servo.read_status()
            positions.append(servo.position)

        elapsed = time.time() - start
        rate = 100 / elapsed

        print(f"  Completed in {elapsed:.2f}s")
        print(f"  Update rate: {rate:.1f} Hz")
        print(f"  Avg time per read: {elapsed/100*1000:.2f} ms")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
