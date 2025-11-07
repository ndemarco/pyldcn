#!/usr/bin/env python3
"""
drive - LDCN Servo Drive Control CLI

Command-line interface for controlling individual LDCN servo drives.

Commands:
    home           Home a single axis
    moveabs        Move to absolute position
    move           Move relative distance
    reset-position Reset position counter

Examples:
    # Home X axis (do this before other axes)
    drive home X --config fiveaxis_full_config.json

    # Move to absolute position
    drive moveabs X 100.0 --velocity 50.0 --accel 10.0 --config config.json

    # Move relative distance
    drive move Y 25.0 --velocity 30.0 --config config.json

    # Reset position counter
    drive reset-position Z --position 0.0 --config config.json

Author: LinuxCNC Community
License: GPL v2 or later
Date: 2025-11-06
"""

import sys
import json
import argparse
from pathlib import Path
from typing import Dict, List

from pyldcn import LDCNNetwork, LDCNError
from pyldcn.command import AxisController


def cmd_home(args):
    """Home a single axis"""
    network = None

    try:
        print(f"Connecting to {args.port} at {args.baud} baud...")
        network = LDCNNetwork(args.port)
        network.open()
        network.initialize()
        network.set_baud_rate(args.baud)

        # Load configuration and create controller
        controller = AxisController(network, args.config)

        # Verify axis exists
        if args.axis not in controller.axes:
            print(
                f"Error: Axis '{args.axis}' not found in configuration", file=sys.stderr
            )
            print(f"Available axes: {list(controller.axes.keys())}", file=sys.stderr)
            return 1

        # Home the axis
        print(f"\nHoming axis '{args.axis}'...")
        try:
            controller.home_axis(args.axis)
            print(f"✓ Axis '{args.axis}' homed successfully")
            return 0
        except NotImplementedError as e:
            print(f"Error: {e}", file=sys.stderr)
            print(
                "\nNote: Homing requires implementation in network.py", file=sys.stderr
            )
            return 1

    except FileNotFoundError:
        print(f"Error: Config file not found: {args.config}", file=sys.stderr)
        return 1
    except KeyError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except LDCNError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1
    finally:
        if network:
            network.close()


def cmd_move_absolute(args):
    """Move axis to absolute position"""
    network = None

    try:
        print(f"Connecting to {args.port} at {args.baud} baud...")
        network = LDCNNetwork(args.port)
        network.open()
        network.initialize()
        network.set_baud_rate(args.baud)

        # Load configuration and create controller
        controller = AxisController(network, args.config)

        # Get current position for display
        current_pos = controller.get_position(args.axis)

        # Execute move
        print(f"\nMoving axis '{args.axis}' to position {args.position}:")
        print(f"  Current position: {current_pos:.3f}")
        print(f"  Target position: {args.position:.3f}")
        print(f"  Distance: {args.position - current_pos:.3f}")
        if args.velocity:
            print(f"  Velocity: {args.velocity}")
        if args.accel:
            print(f"  Acceleration: {args.accel}")

        controller.move_absolute(
            args.axis, position=args.position, velocity=args.velocity, accel=args.accel
        )

        # Wait for completion if requested
        if not args.no_wait:
            print("\nWaiting for motion to complete...")
            timeout = args.timeout if args.timeout else None
            if controller.wait_for_motion_complete(args.axis, timeout=timeout):
                final_pos = controller.get_position(args.axis)
                print(f"✓ Motion complete. Final position: {final_pos:.3f}")
                return 0
            else:
                print("⚠ Motion timeout", file=sys.stderr)
                return 1
        else:
            print("✓ Move command sent (not waiting for completion)")
            return 0

    except FileNotFoundError:
        print(f"Error: Config file not found: {args.config}", file=sys.stderr)
        return 1
    except (KeyError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except LDCNError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1
    finally:
        if network:
            network.close()


def cmd_move_relative(args):
    """Move axis relative distance"""
    network = None

    try:
        print(f"Connecting to {args.port} at {args.baud} baud...")
        network = LDCNNetwork(args.port)
        network.open()
        network.initialize()
        network.set_baud_rate(args.baud)

        # Load configuration and create controller
        controller = AxisController(network, args.config)

        # Get current position for display
        current_pos = controller.get_position(args.axis)
        target_pos = current_pos + args.distance

        # Execute move
        print(f"\nMoving axis '{args.axis}' relative {args.distance:+.3f}:")
        print(f"  Current position: {current_pos:.3f}")
        print(f"  Target position: {target_pos:.3f}")
        if args.velocity:
            print(f"  Velocity: {args.velocity}")
        if args.accel:
            print(f"  Acceleration: {args.accel}")

        controller.move_relative(
            args.axis, distance=args.distance, velocity=args.velocity, accel=args.accel
        )

        # Wait for completion if requested
        if not args.no_wait:
            print("\nWaiting for motion to complete...")
            timeout = args.timeout if args.timeout else None
            if controller.wait_for_motion_complete(args.axis, timeout=timeout):
                final_pos = controller.get_position(args.axis)
                print(f"✓ Motion complete. Final position: {final_pos:.3f}")
                return 0
            else:
                print("⚠ Motion timeout", file=sys.stderr)
                return 1
        else:
            print("✓ Move command sent (not waiting for completion)")
            return 0

    except FileNotFoundError:
        print(f"Error: Config file not found: {args.config}", file=sys.stderr)
        return 1
    except (KeyError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except LDCNError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1
    finally:
        if network:
            network.close()


def cmd_move_reset_position(args):
    """Reset position counter to zero or specified value"""
    network = None

    try:
        print(f"Connecting to {args.port} at {args.baud} baud...")
        network = LDCNNetwork(args.port)
        network.open()
        network.initialize()
        network.set_baud_rate(args.baud)

        # Load configuration and create controller
        controller = AxisController(network, args.config)

        # Get axis and device
        axis = controller.get_axis(args.axis)
        device = controller.get_device(args.axis)

        # Get current position
        old_position = controller.get_position(args.axis)

        # Reset position counter
        new_position = args.position if args.position is not None else 0.0
        new_position_counts = axis.position_to_counts(new_position)

        print(f"\nResetting position counter for axis '{args.axis}':")
        print(
            f"  Old position: {old_position:.3f} ({axis.position_to_counts(old_position)} counts)"
        )
        print(f"  New position: {new_position:.3f} ({new_position_counts} counts)")

        device.reset_position(new_position_counts)

        # Verify
        verify_position = controller.get_position(args.axis)
        print(f"✓ Position reset. Current position: {verify_position:.3f}")
        return 0

    except FileNotFoundError:
        print(f"Error: Config file not found: {args.config}", file=sys.stderr)
        return 1
    except (KeyError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except LDCNError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1
    finally:
        if network:
            network.close()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="LDCN Servo Drive Control CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    # Common arguments
    parent_parser = argparse.ArgumentParser(add_help=False)
    parent_parser.add_argument(
        "--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)"
    )
    parent_parser.add_argument(
        "--baud", type=int, default=125000, help="Baud rate (default: 125000)"
    )
    parent_parser.add_argument(
        "--config", required=True, help="Path to axis configuration JSON file"
    )

    # Create subparsers
    subparsers = parser.add_subparsers(dest="command", help="Command to execute")
    subparsers.required = True

    # home command
    home_parser = subparsers.add_parser(
        "home", parents=[parent_parser], help="Home a single axis"
    )
    home_parser.add_argument("axis", help="Axis name (e.g., X, Y, Z)")
    home_parser.set_defaults(func=cmd_home)

    # moveabs command (absolute move)
    abs_parser = subparsers.add_parser(
        "moveabs", parents=[parent_parser], help="Move to absolute position"
    )
    abs_parser.add_argument("axis", help="Axis name (e.g., X, Y, Z)")
    abs_parser.add_argument("position", type=float, help="Target position (mm or deg)")
    abs_parser.add_argument(
        "--velocity", type=float, help="Velocity (uses config default if not specified)"
    )
    abs_parser.add_argument(
        "--accel",
        type=float,
        help="Acceleration (uses config default if not specified)",
    )
    abs_parser.add_argument(
        "--no-wait", action="store_true", help="Do not wait for motion to complete"
    )
    abs_parser.add_argument(
        "--timeout", type=float, help="Motion timeout in seconds (default: infinite)"
    )
    abs_parser.set_defaults(func=cmd_move_absolute)

    # move command (relative move)
    rel_parser = subparsers.add_parser(
        "move", parents=[parent_parser], help="Move relative distance"
    )
    rel_parser.add_argument("axis", help="Axis name (e.g., X, Y, Z)")
    rel_parser.add_argument(
        "distance",
        type=float,
        help="Relative distance (mm or deg, positive or negative)",
    )
    rel_parser.add_argument(
        "--velocity", type=float, help="Velocity (uses config default if not specified)"
    )
    rel_parser.add_argument(
        "--accel",
        type=float,
        help="Acceleration (uses config default if not specified)",
    )
    rel_parser.add_argument(
        "--no-wait", action="store_true", help="Do not wait for motion to complete"
    )
    rel_parser.add_argument(
        "--timeout", type=float, help="Motion timeout in seconds (default: infinite)"
    )
    rel_parser.set_defaults(func=cmd_move_relative)

    # reset-position command
    reset_parser = subparsers.add_parser(
        "reset-position", parents=[parent_parser], help="Reset position counter"
    )
    reset_parser.add_argument("axis", help="Axis name (e.g., X, Y, Z)")
    reset_parser.add_argument(
        "--position", type=float, default=0.0, help="New position value (default: 0.0)"
    )
    reset_parser.set_defaults(func=cmd_move_reset_position)

    # Parse and execute
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
