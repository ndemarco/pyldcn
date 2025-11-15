#!/usr/bin/env python3
"""
servo_status_example.py - LS-231SE Status Monitoring and Diagnostics

Demonstrates the three-layer status determination architecture:
- Layer 1 (Primary): Raw status byte and auxiliary status byte bits
- Layer 2 (Derived): Pattern-matched CONDITION from diagnostic tables
- Layer 3 (Metadata): Error classification, brake state, fault detection

Author: NickyDoes
License: GPL v2 or later
Date: 2025-11-14
"""

import time
from pyldcn import LDCNNetwork


def main():
    # Initialize network
    print("Connecting to LDCN network...")
    network = LDCNNetwork('/dev/ttyUSB0')

    try:
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        print(f"Found {len(network.devices)} devices\n")

        # Get servo at address 1
        servo = network.get_device(1, 'LS-231SE')

        # =====================================================================
        # Example 1: Basic Status Reading (Layer 1 - Raw Bits)
        # =====================================================================
        print("=" * 70)
        print("Example 1: Basic Status Reading (Layer 1 - Raw Bits)")
        print("=" * 70)

        # Read current status
        status = servo.read_status()

        print(f"\nRaw Status Bytes:")
        print(f"  Status Byte:    0x{status['status']:02X}")
        print(f"  Aux Byte:       0x{status.get('aux_status', 0):02X}")

        print(f"\nKey Flags:")
        flags = status.get('flags', {})
        print(f"  Move Done:      {flags.get('move_done', False)}")
        print(f"  Powered:        {flags.get('power', False)}")
        print(f"  Servo On:       {flags.get('servo_on', False)}")
        print(f"  At Limit:       {flags.get('limit2', False)}")
        print(f"  Homing:         {flags.get('home_in_progress', False)}")
        print()

        # =====================================================================
        # Example 2: Diagnostic Condition Matching (Layer 2 - Derived)
        # =====================================================================
        print("=" * 70)
        print("Example 2: Diagnostic Condition Matching (Layer 2)")
        print("=" * 70)

        # Get diagnostic condition
        condition = servo.get_condition()

        if condition:
            print(f"\nCondition: {condition.condition}")
            print(f"  Mode:           {condition.mode}")
            print(f"  Faulted:        {'YES' if condition.is_faulted else 'NO'}")
            print(f"  Needs Reset:    {'YES' if condition.requires_reset else 'NO'}")
            print(f"  Error Class:    {condition.error_class or 'None'}")
            print(f"  Brake State:    {condition.brake_state}")
            print(f"  Fault Relay:    {condition.fault_relay}")
        else:
            print("\nNo diagnostic condition matched (unknown state)")
        print()

        # =====================================================================
        # Example 3: Quick Status Checks (Layer 3 - Metadata)
        # =====================================================================
        print("=" * 70)
        print("Example 3: Quick Status Checks (Layer 3)")
        print("=" * 70)

        print(f"\nOperational Status:")
        print(f"  Ready for motion:     {'YES' if servo.is_ready() else 'NO'}")
        print(f"  Faulted:              {'YES' if servo.is_faulted() else 'NO'}")
        print(f"  Moving:               {'YES' if not servo.move_done else 'NO'}")
        print(f"  Servo loop enabled:   {'YES' if servo.servo_on else 'NO'}")
        print()

        # =====================================================================
        # Example 4: Formatted Status Report
        # =====================================================================
        print("=" * 70)
        print("Example 4: Formatted Status Report")
        print("=" * 70)
        print()

        # Print comprehensive formatted status
        servo.print_status(include_leds=False)
        print()

        # =====================================================================
        # Example 5: Continuous Status Monitoring
        # =====================================================================
        print("=" * 70)
        print("Example 5: Continuous Status Monitoring (10 seconds)")
        print("=" * 70)
        print("\nMonitoring status changes...\n")

        last_condition = None
        start_time = time.time()

        while time.time() - start_time < 10.0:
            # Read status
            servo.read_status()

            # Get current condition
            current_condition = servo.get_condition()

            # Detect condition changes
            if current_condition and current_condition != last_condition:
                timestamp = time.time() - start_time
                print(f"[{timestamp:6.2f}s] Condition changed: {current_condition.condition}")

                if current_condition.is_faulted:
                    print(f"         FAULT DETECTED: {current_condition.error_class}")
                    print(f"         Brake: {current_condition.brake_state}")

                last_condition = current_condition

            time.sleep(0.1)  # Monitor at 10Hz

        print("\nMonitoring complete\n")

        # =====================================================================
        # Example 6: Fault Detection and Handling
        # =====================================================================
        print("=" * 70)
        print("Example 6: Fault Detection and Handling")
        print("=" * 70)

        # Read current status
        servo.read_status()

        # Check for faults
        if servo.is_faulted():
            condition = servo.get_condition()
            print(f"\n⚠️  FAULT DETECTED: {condition.condition}")
            print(f"   Error Class:     {condition.error_class}")
            print(f"   Brake State:     {condition.brake_state}")
            print(f"   Needs Reset:     {'YES' if condition.requires_reset else 'NO'}")

            if condition.requires_reset:
                print(f"\n   Action Required: HARD RESET (power cycle or reset command)")
            else:
                print(f"\n   Action: Clear faults and investigate cause")
                # servo.clear_faults()  # Uncomment to clear
        else:
            print(f"\n✓ No faults detected - servo operational")
        print()

        # =====================================================================
        # Example 7: Custom Status Item Reading
        # =====================================================================
        print("=" * 70)
        print("Example 7: Custom Status Item Reading (DEFINE_STATUS)")
        print("=" * 70)

        # Configure status to return position, velocity, aux, and path_count
        status_mask = 0x0001 | 0x0004 | 0x0008 | 0x0080  # pos, vel, aux, path_count
        servo.status.define_status(status_mask)

        # Read with custom configuration
        status = servo.read_status()

        print(f"\nCustom Status Read:")
        print(f"  Position:       {status.get('position', 'N/A')} counts")
        print(f"  Velocity:       {status.get('velocity', 'N/A')} counts/tick")
        print(f"  Aux Status:     0x{status.get('aux_status', 0):02X}")
        print(f"  Path Count:     {status.get('path_count', 'N/A')} points")
        print()

        # Read all available status items
        full_status = servo.status.read_full_status()

        print(f"Full Status Read:")
        print(f"  Position:       {full_status.get('position', 'N/A')}")
        print(f"  Velocity:       {full_status.get('velocity', 'N/A')}")
        print(f"  Pos Error:      {full_status.get('pos_error', 'N/A')}")
        print(f"  Home Position:  {full_status.get('home', 'N/A')}")
        print(f"  Device ID:      {full_status.get('device_id', 'N/A')}")
        print()

        # =====================================================================
        # Example 8: Motion State Monitoring
        # =====================================================================
        print("=" * 70)
        print("Example 8: Motion State Monitoring")
        print("=" * 70)

        # Start a small test move (if operational)
        if servo.is_ready():
            print("\nStarting test move...")

            # Small move: 1000 counts at moderate speed
            servo.move_to_counts(position=1000, velocity=1000, accel=100)

            # Monitor motion progress
            print("Monitoring motion...")
            while not servo.move_done:
                servo.read_status()
                pos = servo.position or 0
                vel = servo.velocity or 0
                print(f"  Position: {pos:6d}  Velocity: {vel:5d}  Moving: {not servo.move_done}", end='\r')
                time.sleep(0.05)

            print(f"\n✓ Motion complete - Final position: {servo.position}")
        else:
            print("\n⚠️  Servo not ready for motion")
        print()

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        network.close()
        print("\nNetwork closed")


if __name__ == '__main__':
    main()
