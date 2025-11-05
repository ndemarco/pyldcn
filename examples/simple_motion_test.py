#!/usr/bin/env python3
"""
simple_motion_test.py - Simple Axis Movement Test

A straightforward script to test basic axis movement on LDCN hardware.

This script:
1. Initializes the LDCN network
2. Configures the SK-2310g2 I/O controller (if present)
3. Checks power state
4. Initializes a servo drive
5. Moves the axis a small distance
6. Monitors position

Usage:
    python3 simple_motion_test.py [--port PORT] [--servo ADDR]

Arguments:
    --port PORT        Serial port (default: /dev/ttyUSB0)
    --servo ADDR       Servo address to test (default: 1)
    --baud BAUD        Baud rate (default: 125000)
    --distance DIST    Move distance in counts (default: 1000)
    --velocity VEL     Velocity in counts/sec (default: 1000)
    --accel ACCEL      Acceleration in counts/sec^2 (default: 5000)

Safety:
    - Start with small movements (default: 1000 counts)
    - Ensure axis has clearance to move
    - E-stop should be easily accessible
    - Power can be cycled if needed

Author: LinuxCNC Community
License: GPL v2 or later
"""

import sys
import argparse
import time
from pyldcn import (
    LDCNNetwork, LS231SE, SK2310g2,
    LDCNError, LDCNInitializationError
)


def print_separator(char='=', width=70):
    """Print a separator line."""
    print(char * width)


def print_section(title):
    """Print a section header."""
    print()
    print_separator()
    print(f"  {title}")
    print_separator()
    print()


def initialize_network(port: str, target_baud: int = 125000):
    """
    Initialize the LDCN network.

    Returns:
        LDCNNetwork: Initialized network object
    """
    print_section("NETWORK INITIALIZATION")

    print(f"Opening {port} at 19200 baud...")
    network = LDCNNetwork(port)
    network.open()
    print("✓ Serial port opened")

    print("\nInitializing network (reset, address, discover)...")
    num_devices, device_info = network.initialize()
    print(f"✓ Found {num_devices} devices")

    # Show discovered devices
    for dev in device_info:
        status = "✓" if dev['responding'] else "✗"
        print(f"  {status} Address {dev['address']}: "
              f"ID=0x{dev['device_id']:02X}, Version=0x{dev['version']:02X}")

    # Upgrade baud rate
    if target_baud != 19200:
        print(f"\nUpgrading to {target_baud} baud...")
        network.set_baud_rate(target_baud)
        print(f"✓ Baud rate set to {target_baud}")

    # Show device objects
    print(f"\n✓ Created {len(network.devices)} device objects:")
    for device in network.devices:
        print(f"  {device}")

    return network


def check_io_controller(network: LDCNNetwork):
    """
    Find and configure SK-2310g2 I/O controller if present.

    Returns:
        SK2310g2 or None
    """
    print_section("I/O CONTROLLER CHECK")

    # Find I/O controller
    io_controller = None
    for device in network.devices:
        if isinstance(device, SK2310g2):
            io_controller = device
            break

    if io_controller is None:
        print("⚠ No SK-2310g2 I/O controller found")
        print("  (This is OK if you don't have one)")
        return None

    print(f"Found I/O controller: {io_controller}")

    # Configure it
    print("\nConfiguring I/O controller...")
    io_controller.configure()
    print("✓ Configuration complete")

    # Check diagnostic
    diag = io_controller.read_diagnostic()
    print(f"  Diagnostic code: 0x{diag:02X}")

    # Check power state
    power_state = io_controller.read_power_state()
    power_str = "ON" if power_state else "OFF"
    print(f"  Power state: {power_str}")

    if not power_state:
        print("\n⚠ WARNING: Power is OFF")
        print("  You may need to press the power button on the controller")
        print("  or run the power_on_workflow.py script first")

        response = input("\nContinue anyway? (y/N): ")
        if response.lower() != 'y':
            print("Exiting...")
            sys.exit(0)

    return io_controller


def find_servo(network: LDCNNetwork, servo_addr: int):
    """
    Find the servo drive at the specified address.

    Returns:
        LS231SE: Servo device object

    Raises:
        LDCNError: If servo not found
    """
    print_section(f"LOCATE SERVO AT ADDRESS {servo_addr}")

    servo = None
    for device in network.devices:
        if isinstance(device, LS231SE) and device.address == servo_addr:
            servo = device
            break

    if servo is None:
        raise LDCNError(f"Servo at address {servo_addr} not found or not an LS231SE")

    print(f"✓ Found servo: {servo}")
    return servo


def initialize_servo(servo: LS231SE):
    """
    Initialize the servo drive with default parameters.

    This runs the 7-step initialization sequence.
    """
    print_section("SERVO INITIALIZATION")

    print("Running 7-step initialization sequence...")
    print("  This sets up default PID gains and prepares the servo")
    print()

    success = servo.initialize()

    if success:
        print("\n✓ Servo initialization complete")
    else:
        print("\n⚠ Servo initialization completed with warnings")
        print("  Check for faults before proceeding")

    # Read status
    print("\nReading servo status...")
    status = servo.read_status()

    print(f"  Status byte: 0x{status.get('status', 0):02X}")
    print(f"  Position: {status.get('position', 'N/A')}")
    print(f"  Servo on: {status.get('servo_on', False)}")

    # Check for faults
    faults = servo.check_faults(status.get('status', 0))
    if faults:
        print(f"\n⚠ WARNING: Active faults detected:")
        for fault in faults:
            print(f"    - {fault}")

        response = input("\nContinue anyway? (y/N): ")
        if response.lower() != 'y':
            print("Exiting...")
            sys.exit(1)
    else:
        print("\n✓ No faults detected")


def move_axis(servo: LS231SE, distance: int, velocity: int, accel: int):
    """
    Move the axis a specified distance.

    Args:
        servo: Servo device object
        distance: Distance in encoder counts
        velocity: Velocity in counts/sec
        accel: Acceleration in counts/sec^2
    """
    print_section("AXIS MOVEMENT TEST")

    print(f"Movement parameters:")
    print(f"  Distance: {distance} counts")
    print(f"  Velocity: {velocity} counts/sec")
    print(f"  Acceleration: {accel} counts/sec^2")
    print()

    # Read starting position
    start_pos_data = servo.read_position()
    start_pos = start_pos_data.get('position', 0)
    print(f"Starting position: {start_pos}")

    # Move relative
    target_pos = start_pos + distance
    print(f"Target position: {target_pos}")
    print()

    # Confirm movement
    print("⚠ SAFETY CHECK ⚠")
    print(f"  About to move {distance} counts")
    print(f"  Ensure axis has clearance to move")
    print(f"  E-stop should be easily accessible")
    print()

    response = input("Proceed with movement? (y/N): ")
    if response.lower() != 'y':
        print("Movement cancelled")
        return False

    print("\nExecuting movement...")

    try:
        # Execute move using encoder counts
        servo.move_to_counts(target_pos, velocity, accel)
        print("✓ Move command sent")

        # Monitor position during movement
        print("\nMonitoring position (Ctrl+C to stop)...")
        print("Time     Position      Delta      Status")
        print("-" * 50)

        start_time = time.time()
        last_pos = start_pos
        move_done = False

        while not move_done:
            current_data = servo.read_position()
            current_pos = current_data.get('position', 0)
            status = current_data.get('status', 0)

            elapsed = time.time() - start_time
            delta = current_pos - last_pos

            # Check move done flag (bit 0)
            move_done = bool(status & 0x01)
            status_str = "DONE" if move_done else "MOVING"

            print(f"{elapsed:6.2f}s  {current_pos:10d}  {delta:+6d}  0x{status:02X} {status_str}")

            last_pos = current_pos
            time.sleep(0.05)  # 20 Hz

        # Read final position
        final_pos_data = servo.read_position()
        final_pos = final_pos_data.get('position', 0)

        print()
        print("✓ Movement complete")
        print(f"  Start position: {start_pos}")
        print(f"  Final position: {final_pos}")
        print(f"  Actual movement: {final_pos - start_pos} counts")
        print(f"  Position error: {final_pos - target_pos} counts")

        return True

    except KeyboardInterrupt:
        print("\n\n⚠ Movement interrupted by user")
        print("Reading current position...")
        current_data = servo.read_position()
        current_pos = current_data.get('position', 0)
        print(f"  Current position: {current_pos}")
        return False


def main():
    """Main test sequence."""
    parser = argparse.ArgumentParser(
        description='Simple LDCN Axis Movement Test',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--servo', type=int, default=1,
                       help='Servo address to test (default: 1)')
    parser.add_argument('--baud', type=int, default=125000,
                       help='Baud rate (default: 125000)')
    parser.add_argument('--distance', type=int, default=1000,
                       help='Move distance in counts (default: 1000)')
    parser.add_argument('--velocity', type=int, default=1000,
                       help='Velocity in counts/sec (default: 1000)')
    parser.add_argument('--accel', type=int, default=5000,
                       help='Acceleration in counts/sec^2 (default: 5000)')

    args = parser.parse_args()

    print()
    print_separator('=')
    print("  LDCN SIMPLE MOTION TEST")
    print_separator('=')
    print(f"  Port: {args.port}")
    print(f"  Servo address: {args.servo}")
    print(f"  Baud rate: {args.baud}")
    print_separator('=')

    network = None

    try:
        # Step 1: Initialize network
        network = initialize_network(args.port, args.baud)

        # Step 2: Check I/O controller (optional)
        io_controller = check_io_controller(network)

        # Step 3: Find servo
        servo = find_servo(network, args.servo)

        # Step 4: Initialize servo
        initialize_servo(servo)

        # Step 5: Move axis
        success = move_axis(servo, args.distance, args.velocity, args.accel)

        # Summary
        print()
        print_separator('=')
        if success:
            print("  ✓ TEST COMPLETE - AXIS MOVED SUCCESSFULLY")
        else:
            print("  ⚠ TEST INCOMPLETE")
        print_separator('=')
        print()

        if success:
            print("Next steps:")
            print("  1. Try different distances, velocities, accelerations")
            print("  2. Test homing functionality")
            print("  3. Test continuous position tracking")
            print("  4. Integrate with LinuxCNC")
            print()

    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if network:
            print("Closing network...")
            network.close()
            print("✓ Network closed")


if __name__ == '__main__':
    main()
