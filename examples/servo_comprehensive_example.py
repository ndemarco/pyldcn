#!/usr/bin/env python3
"""
servo_comprehensive_example.py - Complete LS-231SE Feature Demonstration

Demonstrates all major servo features:
- Status monitoring and diagnostics
- Motion control (absolute, relative, velocity)
- Homing sequences
- Path mode (coordinated motion)
- Fault handling and recovery
- Brake control
- I/O operations
- Safety features (watchdog, limit stop)

Author: NickyDoes
License: GPL v2 or later
Date: 2025-11-14
"""

import time
from pyldcn import LDCNNetwork


def wait_for_motion(servo, timeout=10.0):
    """Wait for motion to complete with timeout."""
    start = time.time()
    while not servo.move_done and (time.time() - start) < timeout:
        servo.read_status()
        time.sleep(0.02)
    return servo.move_done


def main():
    # Initialize network
    print("=" * 70)
    print("LS-231SE COMPREHENSIVE SERVO CONTROL DEMONSTRATION")
    print("=" * 70)
    print("\nConnecting to LDCN network...")

    network = LDCNNetwork('/dev/ttyUSB0')

    try:
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        print(f"✓ Found {len(network.devices)} devices\n")

        # Get servo at address 1
        servo = network.get_device(1, 'LS-231SE')

        # =====================================================================
        # SECTION 1: Initialization and Status Check
        # =====================================================================
        print("=" * 70)
        print("SECTION 1: Initialization and Status Check")
        print("=" * 70)

        # Initialize with PID gains
        print("\n1.1 Loading PID gains...")
        servo.load_gains(kp=10, kd=1000, ki=0)
        time.sleep(0.1)
        print("✓ Gains loaded")

        # Check initial status
        print("\n1.2 Checking initial status...")
        servo.print_status()

        if not servo.is_ready():
            print("\n⚠️  Servo not operational - clearing faults...")
            servo.clear_faults()
            time.sleep(0.1)
            servo.read_status()

        print(f"\n✓ Servo ready: {servo.is_ready()}\n")

        # =====================================================================
        # SECTION 2: Basic Motion Control
        # =====================================================================
        print("=" * 70)
        print("SECTION 2: Basic Motion Control")
        print("=" * 70)

        # Absolute positioning
        print("\n2.1 Absolute positioning...")
        print("Moving to position 5000...")
        servo.move_to_counts(position=5000, velocity=2000, accel=200)
        wait_for_motion(servo)
        print(f"✓ Reached position: {servo.position}")

        # Relative move (using current position)
        print("\n2.2 Relative move...")
        current = servo.position or 0
        print(f"Moving +2000 from {current}...")
        servo.move_to_counts(position=current + 2000, velocity=2000, accel=200)
        wait_for_motion(servo)
        print(f"✓ Reached position: {servo.position}")

        # Velocity mode (constant velocity move)
        print("\n2.3 Velocity mode (constant velocity)...")
        print("Moving at 1000 counts/tick for 2 seconds...")
        servo.move_to_counts(position=50000, velocity=1000, accel=100)
        time.sleep(2.0)
        servo.stop()
        print(f"✓ Stopped at position: {servo.position}\n")

        # =====================================================================
        # SECTION 3: Homing
        # =====================================================================
        print("=" * 70)
        print("SECTION 3: Homing Sequence")
        print("=" * 70)

        print("\n3.1 Two-stage homing (limit + index)...")
        servo.home_to_limit(
            limit_switch=2,
            velocity=2000,
            accel=200,
            use_index=True,
            index_velocity=500
        )
        status = servo.status.read_full_status()
        print(f"✓ Homed to position: {status.get('home', 0)}")

        print("\n3.2 Resetting position to zero...")
        servo._motion.reset_position(0)
        servo.read_status()
        print(f"✓ Position reset: {servo.position}")

        print("\n3.3 Saving home position...")
        servo._motion.save_home()
        print(f"✓ Home saved\n")

        # =====================================================================
        # SECTION 4: I/O and Brake Control
        # =====================================================================
        print("=" * 70)
        print("SECTION 4: I/O and Brake Control")
        print("=" * 70)

        print("\n4.1 Engaging brake...")
        servo._io.set_brake(released=False)
        print("✓ Brake engaged")
        time.sleep(0.5)

        print("\n4.2 Releasing brake...")
        servo._io.set_brake(released=True)
        print("✓ Brake released")

        print("\n4.3 Reading input states...")
        inputs = servo._io.read_inputs()
        print(f"  Home switch:    {inputs.get('home_source', False)}")
        print(f"  Limit2 switch:  {inputs.get('limit2', False)}")
        print(f"  Index signal:   {inputs.get('index', False)}\n")

        # =====================================================================
        # SECTION 5: Path Mode (Coordinated Motion)
        # =====================================================================
        print("=" * 70)
        print("SECTION 5: Path Mode (Coordinated Motion)")
        print("=" * 70)

        print("\n5.1 Building path profile...")
        print("Creating 5-point linear trajectory...")

        # Clear any existing path points
        servo._motion.clear_path_buffer()

        # Add path points (position deltas in int8.frac8 format)
        # For simplicity, using small integer deltas
        for i in range(5):
            position_delta = (100 << 8)  # 100 counts per point (int8.frac8)
            servo._motion.add_path_point(
                position=position_delta,
                velocity=500,
                accel=50
            )
        print("✓ 5 path points added")

        print("\n5.2 Starting path execution...")
        servo._motion.start_path_mode()

        # Monitor path execution
        print("Executing path...")
        while servo._motion.get_path_count() > 0:
            servo.read_status()
            count = servo._motion.get_path_count()
            print(f"  Points remaining: {count:3d}  Position: {servo.position or 0:6d}", end='\r')
            time.sleep(0.1)

        print(f"\n✓ Path complete - Final position: {servo.position}\n")

        # =====================================================================
        # SECTION 6: Fault Handling
        # =====================================================================
        print("=" * 70)
        print("SECTION 6: Fault Detection and Recovery")
        print("=" * 70)

        print("\n6.1 Monitoring for faults...")
        servo.read_status()

        if servo.is_faulted():
            condition = servo.get_condition()
            print(f"⚠️  FAULT: {condition.condition}")
            print(f"   Error class:    {condition.error_class}")
            print(f"   Needs reset:    {'YES' if condition.requires_reset else 'NO'}")

            if not condition.requires_reset:
                print(f"\n6.2 Clearing recoverable faults...")
                servo.clear_faults()
                time.sleep(0.1)
                servo.read_status()
                print(f"✓ Faults cleared")
        else:
            print("✓ No faults detected")

        print(f"\n6.3 Verifying operational status...")
        print(f"  Ready:          {servo.is_ready()}")
        print(f"  Faulted:        {servo.is_faulted()}")
        print(f"  Operational:    {servo._status.is_operational()}\n")

        # =====================================================================
        # SECTION 7: Advanced Status Features
        # =====================================================================
        print("=" * 70)
        print("SECTION 7: Advanced Status Features")
        print("=" * 70)

        print("\n7.1 Custom status configuration (DEFINE_STATUS)...")
        # Configure to return position, velocity, aux, pos_error, path_count
        status_mask = 0x0001 | 0x0004 | 0x0008 | 0x0040 | 0x0080
        servo.status.define_status(status_mask)
        print("✓ Status configured")

        print("\n7.2 Reading custom status...")
        status = servo.read_status()
        print(f"  Position:       {status.get('position', 'N/A')}")
        print(f"  Velocity:       {status.get('velocity', 'N/A')}")
        print(f"  Aux Status:     0x{status.get('aux_status', 0):02X}")
        print(f"  Pos Error:      {status.get('pos_error', 'N/A')}")
        print(f"  Path Count:     {status.get('path_count', 'N/A')}")

        print("\n7.3 Getting comprehensive state...")
        comp_state = servo._status.get_comprehensive_state()
        print(f"  Condition:      {comp_state['condition'].condition if comp_state['condition'] else 'Unknown'}")
        print(f"  Faulted:        {comp_state['is_faulted']}")
        print(f"  Operational:    {comp_state['operational']}")
        print(f"  Brake released: {comp_state['brake_released']}\n")

        # =====================================================================
        # SECTION 8: Motion Profiling
        # =====================================================================
        print("=" * 70)
        print("SECTION 8: Motion Profiling Demonstration")
        print("=" * 70)

        print("\n8.1 Slow precise move...")
        servo.move_to_counts(position=1000, velocity=500, accel=50)
        wait_for_motion(servo)
        print(f"✓ Precise positioning: {servo.position}")

        print("\n8.2 Fast move...")
        servo.move_to_counts(position=10000, velocity=5000, accel=500)
        wait_for_motion(servo)
        print(f"✓ Fast move complete: {servo.position}")

        print("\n8.3 Gentle move (low acceleration)...")
        servo.move_to_counts(position=5000, velocity=2000, accel=100)
        wait_for_motion(servo)
        print(f"✓ Gentle move complete: {servo.position}\n")

        # =====================================================================
        # SECTION 9: Position Monitoring
        # =====================================================================
        print("=" * 70)
        print("SECTION 9: Real-Time Position Monitoring")
        print("=" * 70)

        print("\n9.1 Monitoring move in progress...")
        servo.move_to_counts(position=0, velocity=1500, accel=150)

        positions = []
        while not servo.move_done:
            servo.read_status()
            pos = servo.position or 0
            vel = servo.velocity or 0
            positions.append(pos)
            print(f"  Pos: {pos:6d}  Vel: {vel:5d}  Done: {servo.move_done}", end='\r')
            time.sleep(0.05)

        print(f"\n✓ Motion complete")
        print(f"  Start position: {positions[0] if positions else 0}")
        print(f"  End position:   {servo.position}")
        print(f"  Samples:        {len(positions)}\n")

        # =====================================================================
        # SECTION 10: Summary
        # =====================================================================
        print("=" * 70)
        print("SECTION 10: Final Status Summary")
        print("=" * 70)
        print()

        servo.print_status()

        print("\n" + "=" * 70)
        print("COMPREHENSIVE DEMONSTRATION COMPLETE")
        print("=" * 70)
        print("\nFeatures Demonstrated:")
        print("  ✓ Initialization and status monitoring")
        print("  ✓ Absolute and relative positioning")
        print("  ✓ Velocity control")
        print("  ✓ Two-stage homing (limit + index)")
        print("  ✓ Home position save/reset")
        print("  ✓ Brake control")
        print("  ✓ I/O monitoring")
        print("  ✓ Path mode (coordinated motion)")
        print("  ✓ Fault detection and recovery")
        print("  ✓ Custom status configuration")
        print("  ✓ Motion profiling")
        print("  ✓ Real-time position monitoring")
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
