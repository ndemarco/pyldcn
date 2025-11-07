#!/usr/bin/env python3
"""
axis_control_example.py - Example of high-level axis control

Demonstrates using the AxisController API to control axes by name
using configuration from JSON files.

Author: NickyDoes
License: GPL v2 or later
Date: 2025-11-06
"""

from pyldcn import LDCNNetwork
from pyldcn.command import AxisController


def main():
    # Initialize network
    print("Connecting to LDCN network...")
    network = LDCNNetwork('/dev/ttyUSB0')

    try:
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        print(f"Found {len(network.devices)} devices\n")

        # Create axis controller with configuration
        controller = AxisController(network, 'fiveaxis_full_config.json')

        print(f"Loaded configuration for axes: {list(controller.axes.keys())}\n")

        # =====================================================================
        # Example 1: Initialize axis (set gains)
        # =====================================================================
        print("="*60)
        print("Example 1: Initialize X axis with config defaults")
        print("="*60)
        controller.initialize_axis('X')
        print()

        # =====================================================================
        # Example 2: Initialize with custom gains
        # =====================================================================
        print("="*60)
        print("Example 2: Initialize Y axis with custom gains")
        print("="*60)
        controller.initialize_axis('Y', kp=15, kd=1200)
        print()

        # =====================================================================
        # Example 3: Move to absolute position
        # =====================================================================
        print("="*60)
        print("Example 3: Move X axis to absolute position")
        print("="*60)

        # Move using config max velocity/accel
        controller.move_absolute('X', position=100.0)

        # Wait for motion to complete
        print("Waiting for motion to complete...")
        if controller.wait_for_motion_complete('X', timeout=10.0):
            final_pos = controller.get_position('X')
            print(f"Motion complete. Final position: {final_pos:.3f} mm\n")
        else:
            print("Motion timeout!\n")

        # =====================================================================
        # Example 4: Move to absolute position with custom velocity/accel
        # =====================================================================
        print("="*60)
        print("Example 4: Move Y axis with custom velocity and acceleration")
        print("="*60)

        controller.move_absolute('Y', position=50.0, velocity=30.0, accel=5.0)
        controller.wait_for_motion_complete('Y', timeout=10.0)
        print()

        # =====================================================================
        # Example 5: Relative moves
        # =====================================================================
        print("="*60)
        print("Example 5: Relative moves")
        print("="*60)

        # Move 25mm forward
        controller.move_relative('X', distance=25.0)
        controller.wait_for_motion_complete('X', timeout=5.0)
        print(f"X position after +25mm: {controller.get_position('X'):.3f} mm")

        # Move 10mm backward
        controller.move_relative('X', distance=-10.0)
        controller.wait_for_motion_complete('X', timeout=5.0)
        print(f"X position after -10mm: {controller.get_position('X'):.3f} mm\n")

        # =====================================================================
        # Example 6: Stop motion (emergency stop)
        # =====================================================================
        print("="*60)
        print("Example 6: Stop motion")
        print("="*60)

        # Start a long move
        controller.move_absolute('X', position=200.0, velocity=20.0)

        # Stop after short delay
        import time
        time.sleep(0.5)
        controller.stop('X')

        print(f"X position after stop: {controller.get_position('X'):.3f} mm\n")

        # =====================================================================
        # Example 7: Query current positions
        # =====================================================================
        print("="*60)
        print("Example 7: Query all axis positions")
        print("="*60)

        for axis_name in ['X', 'Y', 'Z']:
            try:
                pos = controller.get_position(axis_name)
                print(f"  {axis_name}: {pos:.3f} mm")
            except KeyError:
                print(f"  {axis_name}: Not configured")
        print()

        # =====================================================================
        # Example 8: Coordinated multi-axis move
        # =====================================================================
        print("="*60)
        print("Example 8: Coordinated multi-axis move")
        print("="*60)

        # Start multiple axes moving
        print("Starting coordinated move...")
        controller.move_absolute('X', position=150.0, velocity=50.0, accel=10.0)
        controller.move_absolute('Y', position=100.0, velocity=50.0, accel=10.0)

        # Wait for all to complete
        print("Waiting for X axis...")
        controller.wait_for_motion_complete('X', timeout=10.0)
        print("Waiting for Y axis...")
        controller.wait_for_motion_complete('Y', timeout=10.0)

        print(f"Final positions:")
        print(f"  X: {controller.get_position('X'):.3f} mm")
        print(f"  Y: {controller.get_position('Y'):.3f} mm\n")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        network.close()
        print("Network closed")


if __name__ == '__main__':
    main()
