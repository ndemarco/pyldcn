#!/usr/bin/env python3
"""
test_servo_motion.py - Motion Command Tests

Tests motion-related commands with minimal physical movement:
- Position reset
- Small test moves with immediate stop
- Move_done flag transitions
- Position tracking
- Stop command effectiveness

These tests use small movements and immediate stops for safety.

Author: NickyDoes
License: GPL v2 or later
"""

import pytest
import time
from pyldcn import LDCNNetwork


class TestPositionReset:
    """Test position reset functionality."""

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

        # Reset to zero after test
        servo._motion.reset_position(0)
        network.close()

    def test_reset_to_zero(self, servo):
        """Test resetting position to zero."""
        print("\n  Resetting position to 0...")
        servo._motion.reset_position(0)
        time.sleep(0.05)

        servo.read_status()
        pos = servo.position

        print(f"  Position after reset: {pos}")
        assert pos == 0, "Position should be 0 after reset"

    def test_reset_to_custom_value(self, servo):
        """Test resetting position to custom value."""
        test_value = 12345

        print(f"\n  Resetting position to {test_value}...")
        servo._motion.reset_position(test_value)
        time.sleep(0.05)

        servo.read_status()
        pos = servo.position

        print(f"  Position after reset: {pos}")
        assert pos == test_value, f"Position should be {test_value} after reset"

    def test_reset_negative_value(self, servo):
        """Test resetting position to negative value."""
        test_value = -5000

        print(f"\n  Resetting position to {test_value}...")
        servo._motion.reset_position(test_value)
        time.sleep(0.05)

        servo.read_status()
        pos = servo.position

        print(f"  Position after reset: {pos}")
        assert pos == test_value, f"Position should be {test_value} after reset"

    def test_multiple_resets(self, servo):
        """Test multiple position resets."""
        values = [0, 1000, -500, 9999, 0]

        print("\n  Testing multiple resets:")
        for val in values:
            servo._motion.reset_position(val)
            time.sleep(0.05)
            servo.read_status()
            pos = servo.position
            print(f"    Reset to {val:6d} -> Position: {pos:6d}")
            assert pos == val, f"Position should match reset value {val}"


class TestStopEffectiveness:
    """Test stop command effectiveness and timing."""

    @pytest.fixture
    def servo(self):
        """Setup servo connection."""
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.get_device(1, 'LS-231SE')
        servo.load_gains(kp=10, kd=1000, ki=0)
        servo.enable()
        time.sleep(0.1)

        yield servo

        servo.stop()
        servo._motion.reset_position(0)
        network.close()

    def test_stop_immediately(self, servo):
        """Test immediate stop (no motion)."""
        servo.read_status()
        initial_done = servo.move_done

        print(f"\n  Initial move_done: {initial_done}")

        # Stop
        servo.stop()
        time.sleep(0.05)

        servo.read_status()
        after_done = servo.move_done

        print(f"  After stop move_done: {after_done}")
        assert after_done, "Should be done after stop"

    def test_move_and_stop(self, servo):
        """Test starting small move and stopping immediately."""
        # Start small move
        print("\n  Starting move to 500...")
        servo.move_to_counts(position=500, velocity=500, accel=50)
        time.sleep(0.01)  # Brief delay to start motion

        servo.read_status()
        print(f"  After move command: move_done={servo.move_done}, pos={servo.position}")

        # Immediate stop
        print("  Stopping...")
        servo.stop()
        time.sleep(0.05)

        servo.read_status()
        stopped_pos = servo.position
        print(f"  After stop: move_done={servo.move_done}, pos={stopped_pos}")

        # Should have stopped (position should be less than target)
        # unless move was already complete
        assert servo.move_done or stopped_pos < 500, "Should stop before reaching target or be done"

    def test_stop_timing(self, servo):
        """Test time required for stop command."""
        import time

        # Start move
        servo.move_to_counts(position=1000, velocity=500, accel=50)
        time.sleep(0.01)

        # Stop and measure time
        start = time.time()
        servo.stop()
        elapsed = time.time() - start

        print(f"\n  Stop command time: {elapsed*1000:.2f} ms")

        # Verify stopped
        time.sleep(0.05)
        servo.read_status()
        print(f"  Move done after stop: {servo.move_done}")


class TestMoveDoneFlag:
    """Test move_done flag behavior."""

    @pytest.fixture
    def servo(self):
        """Setup servo connection."""
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.get_device(1, 'LS-231SE')
        servo.load_gains(kp=10, kd=1000, ki=0)
        servo.enable()
        time.sleep(0.1)

        yield servo

        servo.stop()
        servo._motion.reset_position(0)
        network.close()

    def test_initial_move_done(self, servo):
        """Test initial state of move_done flag."""
        servo.read_status()
        initial = servo.move_done

        print(f"\n  Initial move_done: {initial}")
        # Should be true when stationary
        assert initial, "Should be done when stationary"

    def test_move_done_transition(self, servo):
        """Test move_done flag transitions during small move."""
        # Start small move
        print("\n  Starting move...")
        servo.move_to_counts(position=100, velocity=200, accel=50)

        # Sample move_done during move
        states = []
        for i in range(20):
            servo.read_status()
            states.append((i, servo.move_done, servo.position))
            time.sleep(0.01)
            if servo.move_done and i > 5:
                break

        # Show transition
        print("  Move_done transitions:")
        for i, done, pos in states:
            print(f"    Sample {i:2d}: done={done}, pos={pos:6d}")

        # Should see at least some samples
        assert len(states) > 0, "Should capture state samples"

    def test_move_done_after_stop(self, servo):
        """Test move_done after stop command."""
        # Start move
        servo.move_to_counts(position=500, velocity=300, accel=50)
        time.sleep(0.02)

        # Stop
        servo.stop()
        time.sleep(0.05)

        # Check move_done
        servo.read_status()
        print(f"\n  Move_done after stop: {servo.move_done}")
        assert servo.move_done, "Should be done after stop"


class TestSmallMoves:
    """Test small controlled moves for position tracking."""

    @pytest.fixture
    def servo(self):
        """Setup servo connection."""
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.get_device(1, 'LS-231SE')
        servo.load_gains(kp=10, kd=1000, ki=0)
        servo.enable()
        time.sleep(0.1)
        servo._motion.reset_position(0)

        yield servo

        servo.stop()
        servo._motion.reset_position(0)
        network.close()

    def test_small_move_command(self, servo):
        """Test sending small move command."""
        target = 50

        print(f"\n  Moving to {target}...")
        servo.move_to_counts(position=target, velocity=100, accel=20)

        # Sample a few times then stop
        for i in range(5):
            servo.read_status()
            print(f"    Sample {i}: pos={servo.position:6d}, done={servo.move_done}")
            time.sleep(0.02)

        # Stop
        servo.stop()
        time.sleep(0.05)

        servo.read_status()
        final_pos = servo.position
        print(f"  Final position: {final_pos}")

    def test_position_tracking(self, servo):
        """Test position tracking during small move."""
        target = 200

        # Reset position
        servo._motion.reset_position(0)
        time.sleep(0.05)

        # Start move
        servo.move_to_counts(position=target, velocity=200, accel=30)

        # Track position
        positions = []
        for _ in range(15):
            servo.read_status()
            positions.append(servo.position)
            time.sleep(0.01)
            if servo.move_done:
                break

        # Stop
        servo.stop()

        print(f"\n  Position tracking:")
        print(f"    Samples: {len(positions)}")
        print(f"    Start: {positions[0]}, End: {positions[-1]}")
        print(f"    Target: {target}")

        # Position should have changed
        assert positions[-1] != positions[0] or len(positions) == 1, "Position should change or move complete immediately"

    def test_position_consistency(self, servo):
        """Test position reading consistency."""
        # Read position multiple times without moving
        servo.read_status()
        pos1 = servo.position

        time.sleep(0.01)

        servo.read_status()
        pos2 = servo.position

        print(f"\n  Position consistency:")
        print(f"    Reading 1: {pos1}")
        print(f"    Reading 2: {pos2}")

        # Should be same (or very close if servo hunting)
        diff = abs(pos2 - pos1) if (pos1 is not None and pos2 is not None) else 0
        print(f"    Difference: {diff}")


class TestMoveDirection:
    """Test move direction detection."""

    @pytest.fixture
    def servo(self):
        """Setup servo connection."""
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.get_device(1, 'LS-231SE')
        servo.load_gains(kp=10, kd=1000, ki=0)
        servo.enable()
        time.sleep(0.1)
        servo._motion.reset_position(0)

        yield servo

        servo.stop()
        servo._motion.reset_position(0)
        network.close()

    def test_positive_move(self, servo):
        """Test positive direction move."""
        servo._motion.reset_position(0)
        time.sleep(0.05)

        print("\n  Positive move (+100)...")
        servo.move_to_counts(position=100, velocity=150, accel=25)

        # Sample a few times
        for i in range(5):
            servo.read_status()
            print(f"    Sample {i}: pos={servo.position:6d}")
            time.sleep(0.02)

        servo.stop()

    def test_negative_move(self, servo):
        """Test negative direction move."""
        servo._motion.reset_position(0)
        time.sleep(0.05)

        print("\n  Negative move (-100)...")
        servo.move_to_counts(position=-100, velocity=150, accel=25)

        # Sample a few times
        for i in range(5):
            servo.read_status()
            print(f"    Sample {i}: pos={servo.position:6d}")
            time.sleep(0.02)

        servo.stop()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
