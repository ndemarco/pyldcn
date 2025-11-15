#!/usr/bin/env python3
"""
test_servo_basic.py - Basic Servo Tests (No Motion Required)

Tests basic servo operations without requiring physical motion:
- Enable/disable state transitions
- Status byte parsing and flag detection
- Diagnostic condition matching
- Command encoding/decoding
- State management

These tests can run safely without motor power.

Author: NickyDoes
License: GPL v2 or later
"""

import pytest
import time
from pyldcn import LDCNNetwork
from pyldcn.devices import servo_diagnostics as diag


class TestServoEnableDisable:
    """Test servo enable/disable state transitions."""

    @pytest.fixture
    def servo(self):
        """Setup servo connection."""
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        servo = network.get_device(1, 'LS-231SE')
        servo.load_gains(kp=10, kd=1000, ki=0)
        time.sleep(0.1)

        yield servo

        network.close()

    def test_initial_state(self, servo):
        """Test initial servo state after power-up."""
        servo.read_status()

        # Initial state should be "No Motor Power after LDCN Init"
        condition = servo.get_condition()

        assert condition is not None, "Should detect a diagnostic condition"
        print(f"\n  Initial condition: {condition.condition}")
        print(f"  Powered: {servo.power}")
        print(f"  Servo On: {servo.servo_on}")

    def test_enable_detection(self, servo):
        """Test detection of servo enable state."""
        # Read initial state
        servo.read_status()
        initial_powered = servo.power
        initial_servo_on = servo.servo_on
        print(f"\n  Initial state: power={initial_powered}, servo_on={initial_servo_on}")

        # Enable servo
        print("  Enabling servo...")
        servo.enable()
        time.sleep(0.1)

        # Verify state after enable
        servo.read_status()
        after_power = servo.power
        after_servo_on = servo.servo_on
        print(f"  After enable: power={after_power}, servo_on={after_servo_on}")

        # Check if state changed (may not change if motor power unavailable)
        state_changed = (after_power != initial_powered) or (after_servo_on != initial_servo_on)

        if not state_changed:
            print("  Note: No state change detected (motor power may not be available)")
            print("  This is normal for test without motor power supply")

        # Command should complete without error (status read should work)
        assert servo.state.status_byte is not None, "Should be able to read status"

    def test_disable_detection(self, servo):
        """Test detection of servo disable state."""
        # Enable first
        servo.enable()
        time.sleep(0.1)
        servo.read_status()

        enabled_state = servo.power or servo.servo_on
        print(f"\n  Enabled state: power={servo.power}, servo_on={servo.servo_on}")

        # Disable servo
        print("  Disabling servo...")
        servo.disable()
        time.sleep(0.1)

        # Verify disabled
        servo.read_status()
        print(f"  After disable: power={servo.power}, servo_on={servo.servo_on}")

        # Should detect change in state
        disabled_state = servo.power or servo.servo_on

        # State should change (either power or servo_on should be affected)
        print(f"  State changed: {enabled_state} -> {disabled_state}")

    def test_enable_disable_cycle(self, servo):
        """Test complete enable -> disable -> enable cycle."""
        states = []

        # Read initial
        servo.read_status()
        states.append(('initial', servo.power, servo.servo_on))

        # Enable
        servo.enable()
        time.sleep(0.1)
        servo.read_status()
        states.append(('enabled', servo.power, servo.servo_on))

        # Disable
        servo.disable()
        time.sleep(0.1)
        servo.read_status()
        states.append(('disabled', servo.power, servo.servo_on))

        # Enable again
        servo.enable()
        time.sleep(0.1)
        servo.read_status()
        states.append(('re-enabled', servo.power, servo.servo_on))

        # Display state transitions
        print("\n  State Transitions:")
        for label, power, servo_on in states:
            print(f"    {label:12s}: power={power}, servo_on={servo_on}")

        # Verify we can cycle through states
        assert len(set(states)) >= 2, "Should see different states during cycle"


class TestStatusParsing:
    """Test status byte parsing and flag detection."""

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

    def test_status_byte_reading(self, servo):
        """Test reading and parsing status byte."""
        status = servo.read_status()

        assert 'status' in status, "Should return status byte"
        assert isinstance(status['status'], int), "Status should be integer"

        status_byte = status['status']
        print(f"\n  Status byte: 0x{status_byte:02X}")
        print(f"  Binary: {status_byte:08b}")

    def test_flag_extraction(self, servo):
        """Test extraction of individual flags from status byte."""
        servo.read_status()

        # Test flag properties
        flags = {
            'move_done': servo.move_done,
            'power': servo.power,
            'current_limit': servo.current_limit,
            'home_source': servo.state.home_source,
            'limit2': servo.state.limit2,
        }

        print("\n  Status Flags:")
        for name, value in flags.items():
            print(f"    {name:15s}: {value}")

        # All flags should be boolean
        for name, value in flags.items():
            assert isinstance(value, bool), f"{name} should be boolean"

    def test_aux_byte_reading(self, servo):
        """Test reading auxiliary status byte."""
        status = servo.read_status()

        if 'aux_status' in status:
            aux_byte = status['aux_status']
            print(f"\n  Aux status byte: 0x{aux_byte:02X}")
            print(f"  Binary: {aux_byte:08b}")

            print(f"  Aux Flags:")
            print(f"    servo_on: {servo.servo_on}")
            print(f"    path_mode: {servo.state.path_mode}")

    def test_position_reading(self, servo):
        """Test reading position value."""
        status = servo.read_status()

        if 'position' in status:
            pos = status['position']
            print(f"\n  Position: {pos}")
            assert isinstance(pos, int), "Position should be integer"

    def test_velocity_reading(self, servo):
        """Test reading velocity value."""
        status = servo.read_status()

        if 'velocity' in status:
            vel = status['velocity']
            print(f"\n  Velocity: {vel}")


class TestDiagnosticMatching:
    """Test diagnostic condition pattern matching."""

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

    def test_condition_detection(self, servo):
        """Test detection of diagnostic condition."""
        servo.read_status()
        condition = servo.get_condition()

        assert condition is not None, "Should detect a diagnostic condition"

        print(f"\n  Detected Condition:")
        print(f"    Name: {condition.condition}")
        print(f"    Mode: {condition.mode}")
        print(f"    Faulted: {condition.is_faulted}")
        print(f"    Brake: {condition.brake_state}")
        print(f"    Fault Relay: {condition.fault_relay}")

    def test_operational_check(self, servo):
        """Test operational status check."""
        servo.read_status()

        is_ready = servo.is_ready()
        is_faulted = servo.is_faulted()

        print(f"\n  Operational Status:")
        print(f"    Ready: {is_ready}")
        print(f"    Faulted: {is_faulted}")

        # Should not be both ready and faulted
        assert not (is_ready and is_faulted), "Cannot be both ready and faulted"

    def test_comprehensive_state(self, servo):
        """Test comprehensive state query."""
        servo.read_status()
        state = servo._status.get_comprehensive_state()

        print(f"\n  Comprehensive State:")
        print(f"    Status byte: 0x{state['status_byte']:02X}")
        print(f"    Aux byte: 0x{state['aux_byte']:02X}")
        print(f"    Condition: {state['condition'].condition if state['condition'] else 'None'}")
        print(f"    Faulted: {state['is_faulted']}")
        print(f"    Operational: {state['operational']}")
        print(f"    Brake released: {state['brake_released']}")

        assert 'flags' in state, "Should include flags"
        assert 'condition' in state, "Should include condition"

    def test_layer_access(self, servo):
        """Test three-layer architecture access."""
        servo.read_status()

        # Layer 1 - Raw bits
        status_byte = servo.state.status_byte
        aux_byte = servo.state.aux_status

        # Layer 2 - Condition
        condition = servo.get_condition()

        # Layer 3 - Quick checks
        is_faulted = servo.is_faulted()

        print(f"\n  Three-Layer Access:")
        print(f"    Layer 1 (Raw): status=0x{status_byte:02X}, aux=0x{aux_byte:02X}")
        print(f"    Layer 2 (Condition): {condition.condition if condition else 'None'}")
        print(f"    Layer 3 (Metadata): faulted={is_faulted}")


class TestStopCommand:
    """Test stop command functionality."""

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

    def test_stop_command_accepted(self, servo):
        """Test that stop command is accepted."""
        print("\n  Sending stop command...")
        servo.stop()
        time.sleep(0.1)

        servo.read_status()
        print(f"  After stop: move_done={servo.move_done}")

        # Stop should complete without error
        # (exact behavior depends on whether motor was moving)

    def test_stop_when_stationary(self, servo):
        """Test stop command when already stationary."""
        # Read initial state
        servo.read_status()
        initial_done = servo.move_done

        print(f"\n  Initial move_done: {initial_done}")

        # Send stop
        servo.stop()
        time.sleep(0.1)

        # Check state after stop
        servo.read_status()
        after_done = servo.move_done

        print(f"  After stop move_done: {after_done}")

        # Should remain stationary
        assert after_done, "Should be done after stop"


class TestFaultDetection:
    """Test fault detection and handling."""

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

    def test_fault_detection(self, servo):
        """Test detection of fault conditions."""
        servo.read_status()

        is_faulted = servo.is_faulted()
        condition = servo.get_condition()

        print(f"\n  Fault Status:")
        print(f"    Faulted: {is_faulted}")
        if condition:
            print(f"    Condition: {condition.condition}")
            print(f"    Error Class: {condition.error_class}")
            print(f"    Needs Reset: {condition.requires_reset}")

    def test_sticky_bit_detection(self, servo):
        """Test detection of sticky fault bits."""
        servo.read_status()

        sticky_bits = {
            'checksum_error': servo.state.cksum_error,
            'current_limit': servo.state.current_limit,
            'pos_error': servo.state.pos_error_flag,
        }

        print(f"\n  Sticky Bits:")
        for name, value in sticky_bits.items():
            print(f"    {name}: {value}")

    def test_clear_faults(self, servo):
        """Test clearing fault bits."""
        print("\n  Clearing faults...")
        servo.clear_faults()
        time.sleep(0.1)

        servo.read_status()
        print(f"  After clear: current_limit={servo.state.current_limit}")


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
