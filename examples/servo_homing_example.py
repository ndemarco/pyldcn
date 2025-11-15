#!/usr/bin/env python3
"""
servo_homing_example.py - LS-231SE Homing Sequences

Demonstrates two-stage homing procedure:
1. Stage 1: Home to limit switch
2. Stage 2: Fine positioning to index pulse (optional)
3. Save home position to EEPROM

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

        # Initialize servo with gains
        print("Initializing servo...")
        servo.load_gains(kp=10, kd=1000, ki=0)
        time.sleep(0.1)

        # =====================================================================
        # Example 1: Simple Homing to Limit Switch
        # =====================================================================
        print("=" * 70)
        print("Example 1: Simple Homing to Limit Switch")
        print("=" * 70)

        print("\nHoming to forward limit (Limit2)...")

        # Configure homing mode: Limit 2, stop abruptly
        servo.set_home_mode(limit_switch=2, stop_mode='abrupt')

        # Start homing move
        servo.move_to_counts(
            position=100000,     # Move far enough to hit limit
            velocity=2000,       # Moderate homing speed
            accel=200           # Gentle acceleration
        )

        # Wait for limit switch
        print("Waiting for limit switch...")
        while not servo.home_source:
            servo.read_status()
            print(f"  Position: {servo.position or 0:6d}  At limit: {servo.home_source}", end='\r')
            time.sleep(0.05)

        print(f"\n✓ Limit switch triggered at position: {servo.position}")

        # Read captured home position
        status = servo.status.read_full_status()
        home_pos = status.get('home', 0)
        print(f"✓ Home position captured: {home_pos}\n")

        # =====================================================================
        # Example 2: Two-Stage Homing (Limit + Index)
        # =====================================================================
        print("=" * 70)
        print("Example 2: Two-Stage Homing (Limit + Index Pulse)")
        print("=" * 70)

        print("\nPerforming two-stage homing sequence...")

        # Use built-in two-stage homing method
        servo.home_to_limit(
            limit_switch=2,          # Home to forward limit
            velocity=2000,           # Stage 1 velocity
            accel=200,              # Stage 1 accel
            use_index=True,         # Enable stage 2 (index pulse)
            index_velocity=500      # Stage 2 velocity (slower for precision)
        )

        # Read final home position
        status = servo.status.read_full_status()
        final_home = status.get('home', 0)
        print(f"\n✓ Two-stage homing complete")
        print(f"  Final home position: {final_home}\n")

        # =====================================================================
        # Example 3: Homing to Index Pulse Only
        # =====================================================================
        print("=" * 70)
        print("Example 3: Homing to Index Pulse Only (No Limit)")
        print("=" * 70)

        print("\nHoming to index pulse...")

        # Configure homing: index only, stop smoothly
        servo.set_home_mode(use_index=True, stop_mode='smooth')

        # Start move to search for index
        servo.move_to_counts(
            position=5000,          # Move a short distance
            velocity=1000,          # Slow speed for index capture
            accel=100
        )

        # Wait for index pulse
        print("Searching for index pulse...")
        while servo.state.home_in_progress:
            servo.read_status()
            print(f"  Position: {servo.position or 0:6d}  Homing: {servo.state.home_in_progress}", end='\r')
            time.sleep(0.05)

        status = servo.status.read_full_status()
        index_home = status.get('home', 0)
        print(f"\n✓ Index pulse found at position: {index_home}\n")

        # =====================================================================
        # Example 4: Save Home Position to EEPROM
        # =====================================================================
        print("=" * 70)
        print("Example 4: Save Home Position to EEPROM")
        print("=" * 70)

        print(f"\nCurrent position: {servo.position}")

        # Move to desired home position (e.g., offset from limit)
        print("Moving to final home position...")
        servo.move_to_counts(position=1000, velocity=2000, accel=200)

        # Wait for move to complete
        while not servo.move_done:
            servo.read_status()
            time.sleep(0.05)

        print(f"✓ At home position: {servo.position}")

        # Save current position as home
        print("Saving home position to EEPROM...")
        servo._motion.save_home()
        print("✓ Home position saved\n")

        # =====================================================================
        # Example 5: Reset Position to Zero at Home
        # =====================================================================
        print("=" * 70)
        print("Example 5: Reset Position Counter at Home")
        print("=" * 70)

        print(f"\nCurrent position: {servo.position}")

        # Reset position counter to zero
        print("Resetting position to 0...")
        servo._motion.reset_position(0)

        # Verify
        servo.read_status()
        print(f"✓ Position reset: {servo.position}\n")

        # =====================================================================
        # Example 6: Return to Home
        # =====================================================================
        print("=" * 70)
        print("Example 6: Return to Home Position")
        print("=" * 70)

        # Move away from home
        print("\nMoving away from home...")
        servo.move_to_counts(position=5000, velocity=2000, accel=200)

        while not servo.move_done:
            servo.read_status()
            time.sleep(0.05)

        print(f"✓ Moved to position: {servo.position}")

        # Return to home (position 0)
        print("Returning to home...")
        servo.move_to_counts(position=0, velocity=2000, accel=200)

        while not servo.move_done:
            servo.read_status()
            print(f"  Position: {servo.position or 0:6d}  Target: 0", end='\r')
            time.sleep(0.05)

        print(f"\n✓ Back at home: {servo.position}\n")

        # =====================================================================
        # Example 7: Custom Homing Sequence
        # =====================================================================
        print("=" * 70)
        print("Example 7: Custom Homing Sequence")
        print("=" * 70)

        print("\nCustom sequence: Limit → Back off → Index → Save")

        # Stage 1: Fast approach to limit
        print("\nStage 1: Fast approach to limit...")
        servo.set_home_mode(limit_switch=2, stop_mode='abrupt')
        servo.move_to_counts(position=100000, velocity=3000, accel=300)

        while not servo.home_source:
            servo.read_status()
            time.sleep(0.05)

        print(f"✓ Hit limit at: {servo.position}")

        # Stage 2: Back off from limit
        print("\nStage 2: Backing off from limit...")
        current_pos = servo.position or 0
        servo.move_to_counts(position=current_pos - 2000, velocity=1000, accel=100)

        while not servo.move_done:
            servo.read_status()
            time.sleep(0.05)

        print(f"✓ Backed off to: {servo.position}")

        # Stage 3: Slow approach to index
        print("\nStage 3: Slow approach to index pulse...")
        servo.set_home_mode(use_index=True, stop_mode='smooth')
        current_pos = servo.position or 0
        servo.move_to_counts(position=current_pos + 3000, velocity=500, accel=50)

        while servo.state.home_in_progress:
            servo.read_status()
            time.sleep(0.05)

        status = servo.status.read_full_status()
        final_pos = status.get('home', 0)
        print(f"✓ Index found at: {final_pos}")

        # Stage 4: Save as permanent home
        print("\nStage 4: Saving as permanent home...")
        servo._motion.reset_position(0)
        servo._motion.save_home()
        print(f"✓ Home saved at position 0\n")

        # =====================================================================
        # Example 8: Homing Error Handling
        # =====================================================================
        print("=" * 70)
        print("Example 8: Homing Error Handling")
        print("=" * 70)

        print("\nDemonstrating timeout handling...")

        # Start homing with timeout
        servo.set_home_mode(limit_switch=2, stop_mode='abrupt')
        servo.move_to_counts(position=1000, velocity=500, accel=50)

        # Wait with timeout
        timeout = 5.0  # 5 second timeout
        start_time = time.time()
        homing_complete = False

        while time.time() - start_time < timeout:
            servo.read_status()

            if servo.home_source:
                homing_complete = True
                print(f"✓ Homing succeeded")
                break

            time.sleep(0.05)

        if not homing_complete:
            print(f"⚠️  Homing timeout - limit not reached")
            print(f"   Stopping motion...")
            servo.stop()
            print(f"   Final position: {servo.position}")
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
