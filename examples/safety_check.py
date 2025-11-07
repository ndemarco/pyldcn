#!/usr/bin/env python3
"""
safety_check.py - Convenience wrapper for LDCN Safety System Monitor

Quick safety system diagnostic tool. Identifies which device(s) triggered
safety shutdown using routine polling and heuristic analysis.

Usage:
    # Quick check (one-shot analysis)
    ./examples/safety_check.py

    # Monitor for 30 seconds
    ./examples/safety_check.py --monitor --duration 30

    # Continuous monitoring with custom interval
    ./examples/safety_check.py --monitor --interval 1.0

    # Use different port
    ./examples/safety_check.py --port /dev/ttyUSB1

Author: NickyDoes
License: GPL v2 or later
Date: 2025-11-05
"""

import sys
import argparse
from pyldcn import LDCNNetwork

# Import SafetyMonitor from local safety_monitor.py
from safety_monitor import SafetyMonitor


def main():
    parser = argparse.ArgumentParser(
        description='LDCN Safety System Monitor - Quick Diagnostic Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Quick safety check (default)
  ./examples/safety_check.py

  # Monitor for changes over 30 seconds
  ./examples/safety_check.py --monitor --duration 30

  # Continuous monitoring (press Ctrl+C to stop)
  ./examples/safety_check.py --monitor

  # Monitor with slower polling (reduces bus traffic)
  ./examples/safety_check.py --monitor --interval 1.0

The monitor uses CMD_NOP (0x00) for low-overhead status polling and tracks:
  • Servo faults (LS-231SE drives)
  • ServoFAULT input (SK-2310g2)
  • Safety Link chain integrity
  • Diagnostic codes (SK-2310g2)
  • Communication failures

Heuristic analysis determines which device triggered the shutdown first by
tracking fault timestamps and state transitions.
        """
    )

    parser.add_argument(
        '--port',
        default='/dev/ttyUSB0',
        help='Serial port (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=125000,
        help='Baud rate (default: 125000)'
    )
    parser.add_argument(
        '--monitor',
        action='store_true',
        help='Enable continuous monitoring mode'
    )
    parser.add_argument(
        '--interval',
        type=float,
        default=0.5,
        help='Polling interval in seconds (default: 0.5, range: 0.1-5.0)'
    )
    parser.add_argument(
        '--duration',
        type=float,
        help='Monitoring duration in seconds (default: infinite)'
    )

    args = parser.parse_args()

    # Validate interval
    if args.interval < 0.1 or args.interval > 5.0:
        print("Error: Polling interval must be between 0.1 and 5.0 seconds")
        sys.exit(1)

    # Initialize network
    print(f"Connecting to {args.port} at {args.baud} baud...")
    network = LDCNNetwork(args.port)

    try:
        network.open()
        network.initialize()

        if args.baud != 125000:
            print(f"Setting baud rate to {args.baud}...")
            network.set_baud_rate(args.baud)

        print(f"Found {len(network.devices)} devices")

        # Show device summary
        for device in network.devices:
            print(f"  • Address {device.address}: {device.device_type}")
        print()

        # Create monitor
        monitor = SafetyMonitor(network)

        if args.monitor:
            # Continuous monitoring
            monitor.start_monitoring(interval=args.interval, duration=args.duration)
        else:
            # One-shot analysis
            print(monitor.analyze_safety_state())

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)
    finally:
        network.close()


if __name__ == '__main__':
    main()
