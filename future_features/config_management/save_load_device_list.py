#!/usr/bin/env python3
"""
save_load_device_list.py - Device List Save/Load Example

Demonstrates saving and loading device discovery results:
1. Initialize LDCN network and discover devices
2. Save device list to JSON file
3. Load device list from file
4. Display both sets of information

This is the foundation of the three-file configuration system.

Usage:
    python3 save_load_device_list.py [--port PORT] [--baud BAUD] [--output FILE]

Author: LinuxCNC Community
License: GPL v2 or later
"""

import sys
import argparse
import traceback
from pyldcn import LDCNNetwork, LDCNError


def main():
    """Main execution."""
    parser = argparse.ArgumentParser(
        description='LDCN Device List Save/Load Example',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=125000,
                       help='Target baud rate (default: 125000)')
    parser.add_argument('--output', default='device_list.json',
                       help='Output filename (default: device_list.json)')

    args = parser.parse_args()

    print("=" * 70)
    print("LDCN Device List Save/Load Example")
    print("=" * 70)
    print(f"\nPort: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Output: {args.output}")

    network = None

    try:
        # Step 1: Initialize network and discover devices
        print("\n[Step 1] Initialize Network")
        print("-" * 70)

        network = LDCNNetwork(args.port)
        network.open()
        print("✓ Serial port opened")

        print("\nInitializing network (reset, address, discover)...")
        num_devices, _ = network.initialize()
        print(f"✓ Found {num_devices} devices")

        if args.baud != 19200:
            print(f"\nUpgrading to {args.baud} baud...")
            network.set_baud_rate(args.baud)
            print(f"✓ Baud rate upgraded to {args.baud}")

        print(f"\n✓ Created {len(network.devices)} device objects:")
        for device in network.devices:
            print(f"  {device}")

        # Step 2: Save device list
        print("\n[Step 2] Save Device List")
        print("-" * 70)

        print(f"Saving device list to {args.output}...")
        network.save_device_list(args.output)
        print(f"✓ Device list saved to {args.output}")

        # Step 3: Load device list
        print("\n[Step 3] Load Device List")
        print("-" * 70)

        print(f"Loading device list from {args.output}...")
        loaded_devices = network.load_device_list(args.output)
        print("✓ Device list loaded successfully")

        # Step 4: Display loaded information
        print("\n[Step 4] Display Loaded Information")
        print("-" * 70)

        print(f"\nLoaded {len(loaded_devices)} devices:")
        for device in loaded_devices:
            print(f"  Address {device['address']}: {device['device_type']} "
                  f"(ID=0x{device['device_id']:02X}, Version=0x{device['version']:02X})")

        # Success
        print("\n" + "=" * 70)
        print("SUCCESS")
        print("=" * 70)
        print(f"\n✓ Device list saved to {args.output}")
        print("\nNext steps:")
        print("  1. Create axis_config.json with tuning parameters")
        print("  2. Use pyldcn-merge-config to create configured_devices.json")
        print("  3. Apply configuration to hardware during initialization")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except LDCNError as e:
        print(f"\n✗ LDCN Error: {e}")
        sys.exit(1)
    except Exception as e:  # pylint: disable=broad-exception-caught
        print(f"\n✗ Error: {e}")
        traceback.print_exc()
        sys.exit(1)
    finally:
        if network:
            print("\nClosing network...")
            network.close()
            print("✓ Network closed")


if __name__ == '__main__':
    main()
