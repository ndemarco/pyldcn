#!/usr/bin/env python3
"""
pyldcn-device - Unified Device Configuration CLI

Pipeline-friendly Unix-style tool for LDCN device management.

Commands:
    discover    Scan network and output device list to stdout/file
    validate    Validate device_list or axis_config format
    diff        Compare two configuration files
    merge       Merge device_list + axis_config into configured_devices

Examples:
    # Discover devices
    pyldcn-device discover --port /dev/ttyUSB0 > device_list.json

    # Validate configuration
    pyldcn-device validate device_list.json
    cat axis_config.json | pyldcn-device validate

    # Compare configurations
    pyldcn-device discover | pyldcn-device diff - device_list.json

    # Merge configurations
    pyldcn-device merge --axis-config axis.json --device-list device_list.json > configured.json

    # Full pipeline
    pyldcn-device discover | tee device_list.json | \\
        pyldcn-device merge --axis-config axis.json > configured.json

Author: LinuxCNC Community
License: GPL v2 or later
"""

import sys
import json
import argparse
from typing import Optional, Dict, List
from pyldcn import LDCNNetwork, LDCNError
from pyldcn.config import AxisConfig, ConfigError, ConfigValidationError


def cmd_discover(args):
    """Discover devices on LDCN network."""
    network = None

    try:
        # Initialize network
        network = LDCNNetwork(args.port)
        network.open()

        # Discover devices
        num_devices, _ = network.initialize()

        if num_devices == 0:
            print("Warning: No devices discovered", file=sys.stderr)

        # Set baud rate if requested
        if args.baud != 19200:
            network.set_baud_rate(args.baud)

        # Generate device list output
        device_list_data = []
        for device in network.devices:
            device_data = {
                "address": device.address,
                "device_id": device.model_id if device.model_id is not None else 0,
                "device_type": device.device_type,
                "version": device.version if device.version is not None else 0
            }
            device_list_data.append(device_data)

        from datetime import datetime
        output = {
            "file_version": "1.0",
            "discovered_at": datetime.now().isoformat(),
            "port": args.port,
            "baud_rate": network.baud_rate,
            "num_devices": len(network.devices),
            "devices": device_list_data
        }

        # Write to stdout or file
        if args.output:
            with open(args.output, 'w') as f:
                json.dump(output, f, indent=2)
        else:
            json.dump(output, sys.stdout, indent=2)
            print()  # Newline for cleaner pipeline output

        return 0

    except LDCNError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    finally:
        if network:
            network.close()


def cmd_validate(args):
    """Validate device_list.json or axis_config.json format."""
    try:
        # Read input from file or stdin
        if args.file and args.file != '-':
            with open(args.file, 'r') as f:
                data = json.load(f)
        else:
            data = json.load(sys.stdin)

        # Detect file type and validate
        if 'file_version' in data and 'devices' in data and 'discovered_at' in data:
            # Device list format
            _validate_device_list(data)
            file_type = "device_list"
        elif 'file_version' in data and 'axes' in data:
            # Axis config format
            axis_config = AxisConfig(data)
            axis_config.validate()
            file_type = "axis_config"
        elif 'file_version' in data and 'num_configured_axes' in data:
            # Configured devices format
            _validate_configured_devices(data)
            file_type = "configured_devices"
        else:
            print("Error: Unknown file format", file=sys.stderr)
            return 1

        if not args.quiet:
            print(f"✓ Valid {file_type} format", file=sys.stderr)

        return 0

    except FileNotFoundError:
        print(f"Error: File not found: {args.file}", file=sys.stderr)
        return 1
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON: {e}", file=sys.stderr)
        return 1
    except (ConfigError, ConfigValidationError) as e:
        print(f"Error: Validation failed: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


def cmd_diff(args):
    """Compare two configuration files."""
    try:
        # Load both files
        if args.file1 == '-':
            data1 = json.load(sys.stdin)
            file1_name = '<stdin>'
        else:
            with open(args.file1, 'r') as f:
                data1 = json.load(f)
            file1_name = args.file1

        with open(args.file2, 'r') as f:
            data2 = json.load(f)
        file2_name = args.file2

        # Perform diff based on file type
        if 'devices' in data1 and 'devices' in data2:
            _diff_device_lists(data1, data2, file1_name, file2_name)
        elif 'axes' in data1 and 'axes' in data2:
            _diff_axis_configs(data1, data2, file1_name, file2_name)
        else:
            print("Error: Cannot compare files of different types", file=sys.stderr)
            return 1

        return 0

    except FileNotFoundError as e:
        print(f"Error: File not found: {e}", file=sys.stderr)
        return 1
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


def cmd_merge(args):
    """Merge device_list + axis_config into configured_devices."""
    try:
        # Load device list
        if args.device_list == '-':
            device_list = json.load(sys.stdin)
        else:
            with open(args.device_list, 'r') as f:
                device_list = json.load(f)

        # Load axis config
        if args.axis_config == '-':
            if args.device_list == '-':
                print("Error: Only one input can be stdin", file=sys.stderr)
                return 1
            axis_config_data = json.load(sys.stdin)
        else:
            with open(args.axis_config, 'r') as f:
                axis_config_data = json.load(f)

        # Validate inputs
        _validate_device_list(device_list)
        axis_config = AxisConfig(axis_config_data)
        axis_config.validate()

        # Perform merge
        merged = _merge_configurations(device_list, axis_config_data)

        # Output to stdout or file
        if args.output:
            with open(args.output, 'w') as f:
                json.dump(merged, f, indent=2)
        else:
            json.dump(merged, sys.stdout, indent=2)
            print()  # Newline for cleaner pipeline output

        return 0

    except FileNotFoundError as e:
        print(f"Error: File not found: {e}", file=sys.stderr)
        return 1
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON: {e}", file=sys.stderr)
        return 1
    except (ConfigError, ConfigValidationError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


# Helper functions

def _validate_device_list(data: Dict) -> None:
    """Validate device_list.json format."""
    from pyldcn.network import LDCNError

    # Validate file version
    if data.get('file_version') != "1.0":
        raise LDCNError(f"Unsupported file version: {data.get('file_version')}")

    # Validate required fields
    required = ['file_version', 'discovered_at', 'port', 'baud_rate', 'num_devices', 'devices']
    for field in required:
        if field not in data:
            raise LDCNError(f"Missing required field: {field}")

    # Validate device entries
    devices = data['devices']
    if len(devices) != data['num_devices']:
        raise LDCNError(f"Device count mismatch: expected {data['num_devices']}, got {len(devices)}")

    for i, device in enumerate(devices):
        required_device_fields = ['address', 'device_id', 'device_type', 'version']
        for field in required_device_fields:
            if field not in device:
                raise LDCNError(f"Device {i} missing field: {field}")


def _validate_configured_devices(data: Dict) -> None:
    """Validate configured_devices.json format."""
    from pyldcn.network import LDCNError

    if data.get('file_version') != "1.0":
        raise LDCNError(f"Unsupported file version: {data.get('file_version')}")

    required = ['file_version', 'port', 'baud_rate', 'num_devices', 'num_configured_axes', 'devices']
    for field in required:
        if field not in data:
            raise LDCNError(f"Missing required field: {field}")


def _diff_device_lists(data1: Dict, data2: Dict, name1: str, name2: str) -> None:
    """Compare two device lists."""
    devices1 = {d['address']: d for d in data1['devices']}
    devices2 = {d['address']: d for d in data2['devices']}

    all_addresses = sorted(set(devices1.keys()) | set(devices2.keys()))

    differences_found = False

    for addr in all_addresses:
        if addr not in devices1:
            print(f"+ Address {addr}: {devices2[addr]['device_type']} (only in {name2})")
            differences_found = True
        elif addr not in devices2:
            print(f"- Address {addr}: {devices1[addr]['device_type']} (only in {name1})")
            differences_found = True
        elif devices1[addr] != devices2[addr]:
            print(f"~ Address {addr}: differs")
            d1, d2 = devices1[addr], devices2[addr]
            for key in set(d1.keys()) | set(d2.keys()):
                if d1.get(key) != d2.get(key):
                    print(f"    {key}: {d1.get(key)} -> {d2.get(key)}")
            differences_found = True

    if not differences_found:
        print("No differences found")


def _diff_axis_configs(data1: Dict, data2: Dict, name1: str, name2: str) -> None:
    """Compare two axis configurations."""
    axes1 = {a['name']: a for a in data1['axes']}
    axes2 = {a['name']: a for a in data2['axes']}

    all_names = sorted(set(axes1.keys()) | set(axes2.keys()))

    differences_found = False

    for name in all_names:
        if name not in axes1:
            print(f"+ Axis {name}: (only in {name2})")
            differences_found = True
        elif name not in axes2:
            print(f"- Axis {name}: (only in {name1})")
            differences_found = True
        elif axes1[name] != axes2[name]:
            print(f"~ Axis {name}: differs")
            differences_found = True

    if not differences_found:
        print("No differences found")


def _merge_configurations(device_list: Dict, axis_config_data: Dict) -> Dict:
    """Merge device list with axis configuration."""
    from datetime import datetime

    # Create address to axis mapping
    axis_by_address = {}
    for axis in axis_config_data['axes']:
        addr = axis['address']
        if addr in axis_by_address:
            raise ConfigError(f"Duplicate axis address: {addr}")
        axis_by_address[addr] = axis

    # Merge devices with axis config
    merged_devices = []
    num_configured = 0

    for device in device_list['devices']:
        addr = device['address']
        merged_device = device.copy()

        if addr in axis_by_address:
            # Device has axis configuration
            axis = axis_by_address[addr]
            merged_device.update({
                'configured': True,
                'axis_name': axis['name'],
                'axis_type': axis['axis_type'],
                'pitch': axis['pitch'],
                'encoder_resolution': axis['encoder_resolution'],
                'gear_ratio': axis['gear_ratio'],
                'gains': axis['gains'],
                'homing': axis['homing'],
                'limits': axis['limits'],
                'motion': axis['motion']
            })
            num_configured += 1
        else:
            # Device has no axis configuration
            merged_device['configured'] = False

        merged_devices.append(merged_device)

    # Check for axes without matching devices
    device_addresses = {d['address'] for d in device_list['devices']}
    for addr, axis in axis_by_address.items():
        if addr not in device_addresses:
            raise ConfigError(
                f"Axis '{axis['name']}' at address {addr} has no matching device"
            )

    # Build merged configuration
    merged = {
        'file_version': '1.0',
        'merged_at': datetime.now().isoformat(),
        'port': device_list['port'],
        'baud_rate': device_list['baud_rate'],
        'num_devices': len(merged_devices),
        'num_configured_axes': num_configured,
        'devices': merged_devices
    }

    return merged


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='LDCN Device Configuration Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    subparsers.required = True

    # discover command
    discover_parser = subparsers.add_parser('discover',
        help='Discover devices on network')
    discover_parser.add_argument('--port', default='/dev/ttyUSB0',
        help='Serial port (default: /dev/ttyUSB0)')
    discover_parser.add_argument('--baud', type=int, default=125000,
        help='Target baud rate (default: 125000)')
    discover_parser.add_argument('--output', '-o',
        help='Output file (default: stdout)')
    discover_parser.set_defaults(func=cmd_discover)

    # validate command
    validate_parser = subparsers.add_parser('validate',
        help='Validate configuration file')
    validate_parser.add_argument('file', nargs='?', default='-',
        help='File to validate (default: stdin)')
    validate_parser.add_argument('--quiet', '-q', action='store_true',
        help='Suppress success messages')
    validate_parser.set_defaults(func=cmd_validate)

    # diff command
    diff_parser = subparsers.add_parser('diff',
        help='Compare two configuration files')
    diff_parser.add_argument('file1',
        help='First file (or - for stdin)')
    diff_parser.add_argument('file2',
        help='Second file')
    diff_parser.set_defaults(func=cmd_diff)

    # merge command
    merge_parser = subparsers.add_parser('merge',
        help='Merge device_list + axis_config')
    merge_parser.add_argument('--device-list', required=True,
        help='Device list file (or - for stdin)')
    merge_parser.add_argument('--axis-config', required=True,
        help='Axis config file (or - for stdin)')
    merge_parser.add_argument('--output', '-o',
        help='Output file (default: stdout)')
    merge_parser.set_defaults(func=cmd_merge)

    args = parser.parse_args()

    # Execute command
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
