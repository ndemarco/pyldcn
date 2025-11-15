#!/usr/bin/env python3
"""
servo_coordinated_motion.py - Multi-Axis Coordinated Motion

Demonstrates coordinated motion using path mode:
- Synchronized multi-axis moves
- Linear interpolation
- Circular interpolation
- Path planning and execution
- Motion synchronization

Author: NickyDoes
License: GPL v2 or later
Date: 2025-11-14
"""

import time
import math
from typing import List, Tuple
from pyldcn import LDCNNetwork


def wait_for_all_axes(servos: List, timeout: float = 10.0) -> bool:
    """Wait for all axes to complete motion."""
    start = time.time()
    while time.time() - start < timeout:
        all_done = True
        for servo in servos:
            servo.read_status()
            if not servo.move_done:
                all_done = False

        if all_done:
            return True

        time.sleep(0.02)

    return False


def linear_interpolation(
    start: Tuple[float, float],
    end: Tuple[float, float],
    points: int
) -> List[Tuple[float, float]]:
    """
    Generate linear interpolation between two points.

    Args:
        start: (x, y) starting position
        end: (x, y) ending position
        points: Number of intermediate points

    Returns:
        List of (x, y) waypoints
    """
    waypoints = []
    for i in range(points + 1):
        t = i / points
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        waypoints.append((x, y))

    return waypoints


def circular_interpolation(
    center: Tuple[float, float],
    radius: float,
    start_angle: float,
    end_angle: float,
    points: int
) -> List[Tuple[float, float]]:
    """
    Generate circular interpolation.

    Args:
        center: (x, y) circle center
        radius: Circle radius
        start_angle: Starting angle in radians
        end_angle: Ending angle in radians
        points: Number of points along arc

    Returns:
        List of (x, y) waypoints
    """
    waypoints = []
    angle_step = (end_angle - start_angle) / points

    for i in range(points + 1):
        angle = start_angle + i * angle_step
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        waypoints.append((x, y))

    return waypoints


def main():
    # Initialize network
    print("=" * 70)
    print("MULTI-AXIS COORDINATED MOTION DEMONSTRATION")
    print("=" * 70)
    print("\nConnecting to LDCN network...")

    network = LDCNNetwork('/dev/ttyUSB0')

    try:
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        print(f"✓ Found {len(network.devices)} devices\n")

        # Get X and Y axis servos
        servo_x = network.get_device(1, 'LS-231SE')
        servo_y = network.get_device(2, 'LS-231SE')

        # Initialize both axes
        print("Initializing axes...")
        for servo in [servo_x, servo_y]:
            servo.load_gains(kp=10, kd=1000, ki=0)
            time.sleep(0.1)
        print("✓ Axes initialized\n")

        # =====================================================================
        # Example 1: Synchronized Point-to-Point Move
        # =====================================================================
        print("=" * 70)
        print("Example 1: Synchronized Point-to-Point Move")
        print("=" * 70)

        print("\nMoving to (5000, 3000)...")

        # Start both axes simultaneously
        servo_x.move_to_counts(position=5000, velocity=2000, accel=200)
        servo_y.move_to_counts(position=3000, velocity=2000, accel=200)

        # Wait for both to complete
        wait_for_all_axes([servo_x, servo_y])

        print(f"✓ Reached position: X={servo_x.position}, Y={servo_y.position}\n")

        # =====================================================================
        # Example 2: Linear Interpolation
        # =====================================================================
        print("=" * 70)
        print("Example 2: Linear Interpolation")
        print("=" * 70)

        print("\nMoving along line from (0, 0) to (10000, 5000)...")

        # Generate waypoints
        waypoints = linear_interpolation(
            start=(0.0, 0.0),
            end=(10000.0, 5000.0),
            points=10
        )

        print(f"Generated {len(waypoints)} waypoints")

        # Execute waypoints sequentially
        for i, (x, y) in enumerate(waypoints):
            print(f"  Moving to waypoint {i+1}/{len(waypoints)}: ({x:.0f}, {y:.0f})", end='\r')

            servo_x.move_to_counts(position=int(x), velocity=2000, accel=200)
            servo_y.move_to_counts(position=int(y), velocity=2000, accel=200)

            wait_for_all_axes([servo_x, servo_y], timeout=2.0)

        print(f"\n✓ Linear move complete: X={servo_x.position}, Y={servo_y.position}\n")

        # =====================================================================
        # Example 3: Circular Motion Using Path Mode
        # =====================================================================
        print("=" * 70)
        print("Example 3: Circular Motion Using Path Mode")
        print("=" * 70)

        print("\nGenerating circular path...")

        # Generate circular waypoints (quarter circle)
        center = (5000.0, 5000.0)
        radius = 3000.0
        waypoints = circular_interpolation(
            center=center,
            radius=radius,
            start_angle=0,
            end_angle=math.pi / 2,  # Quarter circle (0 to 90 degrees)
            points=20
        )

        print(f"Generated {len(waypoints)} points for circular arc")

        # Clear path buffers
        servo_x._motion.clear_path_buffer()
        servo_y._motion.clear_path_buffer()

        # Add path points (deltas) for both axes
        print("Building path buffers...")
        prev_x, prev_y = waypoints[0]

        for x, y in waypoints[1:]:
            # Calculate deltas
            delta_x = int(x - prev_x)
            delta_y = int(y - prev_y)

            # Convert to int8.frac8 format (simplified - integer only)
            delta_x_encoded = (delta_x & 0xFF) << 8
            delta_y_encoded = (delta_y & 0xFF) << 8

            # Add to path buffers
            servo_x._motion.add_path_point(
                position=delta_x_encoded,
                velocity=500,
                accel=50
            )
            servo_y._motion.add_path_point(
                position=delta_y_encoded,
                velocity=500,
                accel=50
            )

            prev_x, prev_y = x, y

        print(f"✓ Path buffers loaded")

        # Start synchronized path execution
        print("\nExecuting circular path...")
        servo_x._motion.start_path_mode()
        servo_y._motion.start_path_mode()

        # Monitor execution
        while servo_x._motion.get_path_count() > 0 or servo_y._motion.get_path_count() > 0:
            servo_x.read_status()
            servo_y.read_status()

            x_count = servo_x._motion.get_path_count()
            y_count = servo_y._motion.get_path_count()

            print(f"  Path points remaining: X={x_count:3d}, Y={y_count:3d}  Pos: X={servo_x.position or 0:6d}, Y={servo_y.position or 0:6d}", end='\r')
            time.sleep(0.1)

        print(f"\n✓ Circular motion complete: X={servo_x.position}, Y={servo_y.position}\n")

        # =====================================================================
        # Example 4: Return to Origin
        # =====================================================================
        print("=" * 70)
        print("Example 4: Return to Origin")
        print("=" * 70)

        print("\nReturning to (0, 0)...")

        servo_x.move_to_counts(position=0, velocity=2000, accel=200)
        servo_y.move_to_counts(position=0, velocity=2000, accel=200)

        # Monitor return
        while not (servo_x.move_done and servo_y.move_done):
            servo_x.read_status()
            servo_y.read_status()
            print(f"  X: {servo_x.position or 0:6d}  Y: {servo_y.position or 0:6d}", end='\r')
            time.sleep(0.05)

        print(f"\n✓ Back at origin: X={servo_x.position}, Y={servo_y.position}\n")

        # =====================================================================
        # Example 5: Complex Pattern - Square
        # =====================================================================
        print("=" * 70)
        print("Example 5: Square Pattern")
        print("=" * 70)

        print("\nExecuting square pattern (4000 x 4000)...")

        # Define square corners
        corners = [
            (4000, 0),     # Bottom right
            (4000, 4000),  # Top right
            (0, 4000),     # Top left
            (0, 0)         # Bottom left (origin)
        ]

        for i, (x, y) in enumerate(corners):
            print(f"  Moving to corner {i+1}/4: ({x}, {y})")

            servo_x.move_to_counts(position=x, velocity=2000, accel=200)
            servo_y.move_to_counts(position=y, velocity=2000, accel=200)

            wait_for_all_axes([servo_x, servo_y])

        print(f"✓ Square pattern complete\n")

        # =====================================================================
        # Example 6: Velocity Profiling
        # =====================================================================
        print("=" * 70)
        print("Example 6: Coordinated Velocity Profiling")
        print("=" * 70)

        print("\nDemonstrating different velocity profiles...")

        # Slow move
        print("  Slow move (500 counts/tick)...")
        servo_x.move_to_counts(position=2000, velocity=500, accel=50)
        servo_y.move_to_counts(position=2000, velocity=500, accel=50)
        wait_for_all_axes([servo_x, servo_y])
        print(f"  ✓ Position: X={servo_x.position}, Y={servo_y.position}")

        # Medium move
        print("  Medium move (2000 counts/tick)...")
        servo_x.move_to_counts(position=4000, velocity=2000, accel=200)
        servo_y.move_to_counts(position=4000, velocity=2000, accel=200)
        wait_for_all_axes([servo_x, servo_y])
        print(f"  ✓ Position: X={servo_x.position}, Y={servo_y.position}")

        # Fast move
        print("  Fast move (4000 counts/tick)...")
        servo_x.move_to_counts(position=0, velocity=4000, accel=400)
        servo_y.move_to_counts(position=0, velocity=4000, accel=400)
        wait_for_all_axes([servo_x, servo_y])
        print(f"  ✓ Position: X={servo_x.position}, Y={servo_y.position}\n")

        # =====================================================================
        # Example 7: Status Monitoring During Motion
        # =====================================================================
        print("=" * 70)
        print("Example 7: Real-Time Status Monitoring")
        print("=" * 70)

        print("\nMonitoring coordinated move...")

        servo_x.move_to_counts(position=8000, velocity=1500, accel=150)
        servo_y.move_to_counts(position=6000, velocity=1500, accel=150)

        samples = []
        while not (servo_x.move_done and servo_y.move_done):
            servo_x.read_status()
            servo_y.read_status()

            sample = {
                'x_pos': servo_x.position or 0,
                'y_pos': servo_y.position or 0,
                'x_vel': servo_x.velocity or 0,
                'y_vel': servo_y.velocity or 0
            }
            samples.append(sample)

            print(f"  X: {sample['x_pos']:6d} ({sample['x_vel']:5d})  Y: {sample['y_pos']:6d} ({sample['y_vel']:5d})", end='\r')
            time.sleep(0.05)

        print(f"\n✓ Motion complete - Captured {len(samples)} samples")
        print(f"  Final position: X={servo_x.position}, Y={servo_y.position}\n")

        # =====================================================================
        # Summary
        # =====================================================================
        print("=" * 70)
        print("COORDINATED MOTION DEMONSTRATION COMPLETE")
        print("=" * 70)
        print("\nFeatures Demonstrated:")
        print("  ✓ Synchronized point-to-point moves")
        print("  ✓ Linear interpolation")
        print("  ✓ Circular interpolation with path mode")
        print("  ✓ Complex patterns (square)")
        print("  ✓ Coordinated velocity profiling")
        print("  ✓ Real-time motion monitoring")
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
