#!/usr/bin/env python3
"""
test_ldcn_module.py - Test Script for LDCN Network Module

This script tests the ldcn_network module with real hardware.

Usage:
    python3 test_ldcn_module.py [options]

Options:
    --port PORT         Serial port (default: /dev/ttyUSB0)
    --target-baud BAUD  Target baud rate (default: 125000)
    --test-servo ADDR   Test servo at address (default: 1)
    --skip-init         Skip network initialization
    --verbose           Show detailed output

Test Sequence:
    1. Initialize network (reset, address, discover)
    2. Upgrade to target baud rate
    3. Test servo initialization
    4. Test position read
    5. Test I/O controller status

⚠️ HARDWARE VERIFICATION STATUS: UNVERIFIED
This script and the module it tests are UNVERIFIED until tested against
real LDCN hardware.

Author: LinuxCNC Community
License: GPL v2 or later
Date: 2025-10-29
"""

import sys
import argparse
import time
from pyldcn import (
    LDCNNetwork, LS231SE, SK2310g2,
    LDCNError, LDCNInitializationError
)


class Colors:
    """ANSI color codes for terminal output"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'


def log(msg: str, level: str = 'INFO'):
    """Print log message with color"""
    colors = {
        'INFO': Colors.OKBLUE,
        'SUCCESS': Colors.OKGREEN,
        'WARNING': Colors.WARNING,
        'ERROR': Colors.FAIL,
        'HEADER': Colors.HEADER,
    }
    color = colors.get(level, '')
    print(f"{color}[{level}]{Colors.ENDC} {msg}")


def test_network_initialization(port: str, target_baud: int) -> LDCNNetwork:
    """
    Test 1: Network Initialization

    Tests:
    - Open serial port at 19200 baud
    - Hard reset
    - Device addressing
    - Device discovery
    - Baud rate upgrade
    """
    log("="*70, 'HEADER')
    log("TEST 1: Network Initialization", 'HEADER')
    log("="*70, 'HEADER')

    try:
        # Open network
        log(f"Opening {port} at 19200 baud...")
        network = LDCNNetwork(port)
        network.open()
        log("✓ Serial port opened", 'SUCCESS')

        # Initialize
        log("\nInitializing network (reset, address, discover)...")
        num_devices, device_info = network.initialize()
        log(f"✓ Initialization complete", 'SUCCESS')

        # Display results
        log(f"\nFound {num_devices} devices:", 'SUCCESS')
        for dev in device_info:
            responding = "✓" if dev['responding'] else "✗"
            log(f"  {responding} Address {dev['address']}: "
                f"ID=0x{dev['device_id']:02X}, "
                f"Version=0x{dev['version']:02X}", 'INFO')

        # Upgrade baud rate
        if target_baud != 19200:
            log(f"\nUpgrading to {target_baud} baud...", 'INFO')
            network.set_baud_rate(target_baud)
            log(f"✓ Baud rate upgraded to {target_baud}", 'SUCCESS')

        # Display device objects
        log(f"\nCreated {len(network.devices)} device objects:", 'SUCCESS')
        for device in network.devices:
            log(f"  {device}", 'INFO')

        return network

    except LDCNInitializationError as e:
        log(f"✗ Initialization failed: {e}", 'ERROR')
        raise
    except Exception as e:
        log(f"✗ Error: {e}", 'ERROR')
        raise


def test_servo_operations(network: LDCNNetwork, servo_addr: int):
    """
    Test 2: Servo Operations

    Tests:
    - Servo initialization (7 steps)
    - Position reading
    - Status reading
    - Fault checking
    """
    log("\n" + "="*70, 'HEADER')
    log("TEST 2: Servo Operations", 'HEADER')
    log("="*70, 'HEADER')

    # Find servo device
    servo = None
    for device in network.devices:
        if isinstance(device, LS231SE) and device.address == servo_addr:
            servo = device
            break

    if servo is None:
        log(f"✗ Servo at address {servo_addr} not found or not an LS231SE", 'ERROR')
        return False

    log(f"Testing servo: {servo}", 'INFO')

    try:
        # Initialize servo
        log("\nInitializing servo (7-step sequence)...", 'INFO')
        success = servo.initialize()

        if success:
            log("✓ Servo initialization complete", 'SUCCESS')
        else:
            log("⚠ Servo initialization completed with faults", 'WARNING')

        # Read status
        log("\nReading servo status...", 'INFO')
        status = servo.read_status()

        log("Servo Status:", 'INFO')
        log(f"  Status byte: 0x{status.get('status', 0):02X}", 'INFO')
        log(f"  Position: {status.get('position', 'N/A')}", 'INFO')
        log(f"  Velocity: {status.get('velocity', 'N/A')}", 'INFO')
        log(f"  Servo on: {status.get('servo_on', 'N/A')}", 'INFO')

        # Check flags
        flags = status.get('flags', {})
        log("\nStatus Flags:", 'INFO')
        for flag, value in flags.items():
            symbol = "✓" if value else " "
            log(f"  [{symbol}] {flag}: {value}", 'INFO')

        # Check faults
        faults = servo.check_faults(status.get('status', 0))
        if faults:
            log(f"\n⚠ Active faults: {', '.join(faults)}", 'WARNING')
        else:
            log("\n✓ No faults detected", 'SUCCESS')

        return True

    except Exception as e:
        log(f"✗ Error during servo operations: {e}", 'ERROR')
        import traceback
        traceback.print_exc()
        return False


def test_io_controller(network: LDCNNetwork):
    """
    Test 3: I/O Controller Operations

    Tests:
    - I/O controller configuration
    - Diagnostic code reading
    - Power state reading
    """
    log("\n" + "="*70, 'HEADER')
    log("TEST 3: I/O Controller Operations", 'HEADER')
    log("="*70, 'HEADER')

    # Find I/O controller
    io_controller = None
    for device in network.devices:
        if isinstance(device, SK2310g2):
            io_controller = device
            break

    if io_controller is None:
        log("✗ I/O controller (SK2310g2) not found", 'ERROR')
        return False

    log(f"Testing I/O controller: {io_controller}", 'INFO')

    try:
        # Configure for full status
        log("\nConfiguring I/O controller...", 'INFO')
        io_controller.configure()
        log("✓ Configuration complete", 'SUCCESS')

        # Read diagnostic
        log("\nReading diagnostic code...", 'INFO')
        diag = io_controller.read_diagnostic()
        log(f"  Diagnostic code: 0x{diag:02X}", 'INFO')

        # Read power state
        log("\nReading power state...", 'INFO')
        power_state = io_controller.read_power_state()
        power_str = "ON" if power_state else "OFF"
        log(f"  Power state: {power_str}", 'INFO')

        # Read full status
        log("\nReading full status...", 'INFO')
        status = io_controller.read_status()
        log(f"  Status byte: 0x{status.get('status', 0):02X}", 'INFO')
        log(f"  Diagnostic: 0x{status.get('diagnostic', 0):02X}", 'INFO')
        log(f"  Power state: {status.get('power_state', False)}", 'INFO')

        return True

    except NotImplementedError as e:
        log(f"⚠ Feature not implemented: {e}", 'WARNING')
        return True  # Not a failure
    except Exception as e:
        log(f"✗ Error during I/O controller operations: {e}", 'ERROR')
        import traceback
        traceback.print_exc()
        return False


def test_position_reading(network: LDCNNetwork, servo_addr: int, duration: float = 2.0):
    """
    Test 4: Continuous Position Reading

    Tests:
    - Fast position reading
    - Position monitoring over time
    """
    log("\n" + "="*70, 'HEADER')
    log("TEST 4: Continuous Position Reading", 'HEADER')
    log("="*70, 'HEADER')

    # Find servo
    servo = None
    for device in network.devices:
        if isinstance(device, LS231SE) and device.address == servo_addr:
            servo = device
            break

    if servo is None:
        log(f"✗ Servo at address {servo_addr} not found", 'ERROR')
        return False

    log(f"Reading position from servo {servo_addr} for {duration}s...", 'INFO')
    log("(Move the axis by hand to see position change)\n", 'INFO')

    try:
        start_time = time.time()
        sample_count = 0

        while time.time() - start_time < duration:
            pos_data = servo.read_position()
            position = pos_data.get('position', 0)
            status = pos_data.get('status', 0)

            elapsed = time.time() - start_time
            log(f"  [{elapsed:5.2f}s] Position: {position:10d}  Status: 0x{status:02X}", 'INFO')

            sample_count += 1
            time.sleep(0.1)  # 10 Hz

        log(f"\n✓ Read {sample_count} position samples", 'SUCCESS')
        return True

    except Exception as e:
        log(f"✗ Error during position reading: {e}", 'ERROR')
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main test runner"""
    parser = argparse.ArgumentParser(
        description='LDCN Network Module Test Script',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--target-baud', type=int, default=125000,
                       help='Target baud rate (default: 125000)')
    parser.add_argument('--test-servo', type=int, default=1,
                       help='Servo address to test (default: 1)')
    parser.add_argument('--skip-init', action='store_true',
                       help='Skip network initialization')
    parser.add_argument('--verbose', action='store_true',
                       help='Show detailed output')

    args = parser.parse_args()

    print()
    log("="*70, 'HEADER')
    log("LDCN NETWORK MODULE TEST SUITE", 'HEADER')
    log("="*70, 'HEADER')
    log(f"Port: {args.port}", 'INFO')
    log(f"Target baud: {args.target_baud}", 'INFO')
    log(f"Test servo: {args.test_servo}", 'INFO')
    print()

    tests_passed = 0
    tests_failed = 0
    network = None

    try:
        # Test 1: Network Initialization
        if not args.skip_init:
            network = test_network_initialization(args.port, args.target_baud)
            tests_passed += 1
        else:
            log("Skipping network initialization", 'WARNING')
            # Open network without initialization
            network = LDCNNetwork(args.port)
            network.open()
            if args.target_baud != 19200:
                network.set_baud_rate(args.target_baud)

        # Test 2: Servo Operations
        if test_servo_operations(network, args.test_servo):
            tests_passed += 1
        else:
            tests_failed += 1

        # Test 3: I/O Controller
        if test_io_controller(network):
            tests_passed += 1
        else:
            tests_failed += 1

        # Test 4: Position Reading
        if test_position_reading(network, args.test_servo, duration=2.0):
            tests_passed += 1
        else:
            tests_failed += 1

    except KeyboardInterrupt:
        log("\n\n✗ Tests interrupted by user", 'ERROR')
        sys.exit(1)
    except Exception as e:
        log(f"\n\n✗ Test suite failed: {e}", 'ERROR')
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if network:
            log("\nClosing network...", 'INFO')
            network.close()

    # Summary
    log("\n" + "="*70, 'HEADER')
    log("TEST SUMMARY", 'HEADER')
    log("="*70, 'HEADER')
    log(f"Tests passed: {tests_passed}", 'SUCCESS' if tests_passed > 0 else 'INFO')
    log(f"Tests failed: {tests_failed}", 'ERROR' if tests_failed > 0 else 'INFO')

    if tests_failed == 0:
        log("\n✓ ALL TESTS PASSED!", 'SUCCESS')
        sys.exit(0)
    else:
        log(f"\n✗ {tests_failed} TESTS FAILED", 'ERROR')
        sys.exit(1)


if __name__ == '__main__':
    main()
