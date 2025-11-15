#!/usr/bin/env python3
"""
read_sk2310g2_status.py - SK-2310g2 I/O Status Reader

Reads and displays complete I/O status from SK-2310g2 supervisory controller
including digital inputs/outputs with functional labels and analog values.

Usage:
    python3 read_sk2310g2_status.py [options]

Options:
    --port PORT         Serial port (default: /dev/ttyUSB0)
    --baud BAUD         Baud rate (default: 125000)
    --address ADDR      Device address (default: auto-detect)
    --continuous        Continuous monitoring mode
    --interval SEC      Polling interval in continuous mode (default: 1.0)
    --outputs           Also read and display output states

Author: NickyDoes
License: GPL v2 or later
Date: 2025-11-12
"""

import sys
import argparse
import time
from pyldcn import LDCNNetwork, SK2310g2
from pyldcn.devices.sk2310g2 import format_status


def find_sk2310g2(network: LDCNNetwork) -> SK2310g2:
    """
    Find SK-2310g2 device on network.

    Args:
        network: Initialized LDCNNetwork

    Returns:
        SK2310g2 device instance

    Raises:
        RuntimeError: If no SK-2310g2 found
    """
    device = network.find_supervisor()
    if device is None:
        raise RuntimeError("No SK-2310g2 device found on network")
    return device


def read_and_display(device: SK2310g2, show_outputs: bool = False):
    """
    Read status and display I/O report.

    Args:
        device: SK2310g2 device instance
        show_outputs: If True, also read and display outputs
    """
    # Read status
    status = device.read_status()

    # Read outputs if requested (requires additional command)
    if show_outputs:
        try:
            # Read current output states using CMD_READ_OUTPUT
            # This is device-specific and may not be implemented yet
            outputs = device.read_digital_outputs()
            status['digital_outputs'] = outputs
        except Exception as e:
            print(f"Warning: Could not read outputs: {e}")

    # Display formatted report
    print(format_status(status))


def continuous_monitor(device: SK2310g2, interval: float, show_outputs: bool):
    """
    Continuously monitor and display status.

    Args:
        device: SK2310g2 device instance
        interval: Polling interval in seconds
        show_outputs: If True, also read and display outputs
    """
    print(f"Starting continuous monitoring (interval: {interval}s)")
    print("Press Ctrl+C to stop\n")
    time.sleep(1)  # Give user time to read initial message

    try:
        # Clear screen before entering loop
        print('\033[2J\033[H', end='', flush=True)

        while True:
            # Move cursor to home position (row 1, col 1)
            print('\033[H', end='', flush=True)

            # Display timestamp
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            print(f"Timestamp: {timestamp}\n")

            # Read and display
            read_and_display(device, show_outputs)

            # Display update info at bottom
            print(f"\nUpdate interval: {interval}s | Press Ctrl+C to stop")

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user")


def main():
    """Command-line interface"""
    parser = argparse.ArgumentParser(
        description='SK-2310g2 I/O Status Reader',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Read status once
  python3 read_sk2310g2_status.py --port /dev/ttyUSB0

  # Continuous monitoring at 1Hz
  python3 read_sk2310g2_status.py --continuous --interval 1.0

  # Read status including outputs
  python3 read_sk2310g2_status.py --outputs

  # Custom baud rate and address
  python3 read_sk2310g2_status.py --baud 19200 --address 2
        """
    )

    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=125000,
                       help='Baud rate (default: 125000)')
    parser.add_argument('--address', type=int,
                       help='Device address (default: auto-detect)')
    parser.add_argument('--continuous', action='store_true',
                       help='Enable continuous monitoring mode')
    parser.add_argument('--interval', type=float, default=1.0,
                       help='Polling interval in seconds (default: 1.0)')
    parser.add_argument('--outputs', action='store_true',
                       help='Also read and display output states')

    args = parser.parse_args()

    # Initialize network
    print(f"Connecting to {args.port} at {args.baud} baud...")
    network = LDCNNetwork(args.port)

    try:
        network.open()
        network.initialize()

        # Set baud rate if different from default
        if args.baud != 19200:
            network.set_baud_rate(args.baud)

        print(f"Found {len(network.devices)} devices")

        # Find SK-2310g2
        if args.address is not None:
            # Use specific address
            device = None
            for dev in network.devices:
                if dev.address == args.address:
                    if not isinstance(dev, SK2310g2):
                        print(f"Error: Device at address {args.address} is not SK-2310g2")
                        sys.exit(1)
                    device = dev
                    break
            if device is None:
                print(f"Error: No device found at address {args.address}")
                sys.exit(1)
        else:
            # Auto-detect
            device = find_sk2310g2(network)

        print(f"Using SK-2310g2 at address {device.address}\n")

        # Read and display
        if args.continuous:
            continuous_monitor(device, args.interval, args.outputs)
        else:
            read_and_display(device, args.outputs)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        network.close()


if __name__ == '__main__':
    main()
