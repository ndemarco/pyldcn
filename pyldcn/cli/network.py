#!/usr/bin/env python3
"""
ldcn - LDCN Network Configuration CLI

Pipeline-friendly Unix-style tool for LDCN network management.

Commands:
    discover    Scan network and output device list to stdout/file
    validate    Validate device_list or axis_config format
    diff        Compare two configuration files
    merge       Merge device_list + axis_config into configured_devices
    init        Initialize network, verify devices, set gains

Examples:
    # Discover devices
    ldcn discover --port /dev/ttyUSB0 > device_list.json

    # Validate configuration
    ldcn validate device_list.json
    cat axis_config.json | ldcn validate

    # Compare discovered devices with axis configuration
    ldcn discover | ldcn diff - examples/fiveaxis_full_config.json

    # Compare two device lists
    ldcn discover | ldcn diff - device_list.json

    # Merge configurations
    ldcn merge --axis-config axis.json --device-list device_list.json > configured.json

    # Initialize network and set gains
    ldcn init --config fiveaxis_full_config.json --port /dev/ttyUSB0

    # Full pipeline
    ldcn discover | tee device_list.json | \\
        ldcn merge --axis-config axis.json > configured.json

Author: LinuxCNC Community
License: GPL v2 or later
"""

import sys
import json
import argparse
from typing import Optional, Dict, List
from pyldcn import LDCNNetwork, LDCNError
from pyldcn.config import AxisConfig, ConfigError, ConfigValidationError
from pyldcn.command import AxisController


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

        if args.verbose:
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

        # Detect file types
        is_device_list1 = 'devices' in data1 and 'discovered_at' in data1
        is_device_list2 = 'devices' in data2 and 'discovered_at' in data2
        is_axis_config1 = 'axes' in data1 and not is_device_list1
        is_axis_config2 = 'axes' in data2 and not is_device_list2

        # Perform diff based on file types
        differences_found = False
        if is_device_list1 and is_device_list2:
            # Both are device lists
            differences_found = _diff_device_lists(data1, data2, file1_name, file2_name, args.verbose)
        elif is_axis_config1 and is_axis_config2:
            # Both are axis configs
            differences_found = _diff_axis_configs(data1, data2, file1_name, file2_name, args.verbose)
        elif (is_device_list1 and is_axis_config2) or (is_axis_config1 and is_device_list2):
            # One is device_list, other is axis_config
            # Convert axis config to pseudo device list format and compare
            if is_device_list1:
                device_list_data = data1
                axis_config_data = data2
                device_list_name = file1_name
                axis_config_name = file2_name
            else:
                device_list_data = data2
                axis_config_data = data1
                device_list_name = file2_name
                axis_config_name = file1_name

            differences_found = _diff_discovered_vs_config(device_list_data, axis_config_data,
                                                          device_list_name, axis_config_name, args.verbose)
        else:
            print("Error: Cannot compare files of different types", file=sys.stderr)
            return 1

        # Return non-zero exit code if differences found
        return 1 if differences_found else 0

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


def cmd_init(args):
    """Initialize LDCN network, verify devices, and set gains"""
    network = None

    try:
        if args.verbose:
            print(f"Connecting to {args.port} at {args.baud} baud...")
        network = LDCNNetwork(args.port)
        network.open()

        # Initialize network (discover devices)
        if args.verbose:
            print("Discovering devices...")
        num_devices, _ = network.initialize()
        if args.verbose:
            print(f"Found {num_devices} devices on network")

        # Set baud rate if not default
        if args.baud != 19200:
            if args.verbose:
                print(f"Setting baud rate to {args.baud}...")
            network.set_baud_rate(args.baud)

        # Load configuration
        if args.verbose:
            print(f"\nLoading configuration from {args.config}...")
        with open(args.config, 'r') as f:
            config = json.load(f)

        # Verify devices match configuration
        if args.verbose:
            print("\nVerifying devices...")
        axes = config.get('axes', [])
        io_devices = config.get('io_devices', [])
        verified = []
        errors = []

        # Verify axis devices
        for axis_config in axes:
            name = axis_config['name']
            address = axis_config['address']

            # Find device at address
            device = None
            for dev in network.devices:
                if dev.address == address:
                    device = dev
                    break

            if device is None:
                errors.append(f"  ✗ Axis '{name}' (address {address}): No device found")
                continue

            # Verify device type (should be LS-231SE for servo axes)
            expected_type = 'LS-231SE'  # Could be in config
            if device.device_type != expected_type:
                errors.append(
                    f"  ✗ Axis '{name}' (address {address}): "
                    f"Expected {expected_type}, found {device.device_type}"
                )
                continue

            # Device verified
            verified.append((name, address, device.device_type, device.version, 'axis'))
            if args.verbose:
                print(f"  ✓ Axis '{name}' (address {address}): {device.device_type} v{device.version}")

        # Verify IO devices
        for io_config in io_devices:
            name = io_config.get('name', 'IO')
            address = io_config['address']
            expected_type = io_config.get('device_type', 'SK-2310g2')

            # Find device at address
            device = None
            for dev in network.devices:
                if dev.address == address:
                    device = dev
                    break

            if device is None:
                errors.append(f"  ✗ IO device '{name}' (address {address}): No device found")
                continue

            # Verify device type
            if device.device_type != expected_type:
                errors.append(
                    f"  ✗ IO device '{name}' (address {address}): "
                    f"Expected {expected_type}, found {device.device_type}"
                )
                continue

            # Device verified
            verified.append((name, address, device.device_type, device.version, 'io'))
            if args.verbose:
                print(f"  ✓ IO device '{name}' (address {address}): {device.device_type} v{device.version}")

        if errors:
            print("\nErrors found:", file=sys.stderr)
            for error in errors:
                print(error, file=sys.stderr)
            return 1

        if args.verbose:
            print(f"\nAll {len(verified)} devices verified")

        # Initialize gains for all axes
        if args.verbose:
            print("\nSetting gains for all axes...")
        controller = AxisController(network, args.config)

        for axis_name in controller.axes.keys():
            if args.verbose:
                print(f"\nInitializing axis '{axis_name}':")
            controller.initialize_axis(axis_name, set_gains=True, home=False)

        if args.verbose:
            print("\n✓ Network initialization complete")
        return 0

    except FileNotFoundError as e:
        print(f"Error: Config file not found: {args.config}", file=sys.stderr)
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


# Helper functions

def _diff_discovered_vs_config(device_list_data: Dict, axis_config_data: Dict,
                                device_list_name: str, axis_config_name: str, verbose: bool = True) -> bool:
    """Compare discovered devices with axis configuration.

    Verifies that devices exist at all addresses specified in the axis config
    and io_devices sections.

    Returns:
        True if differences were found, False otherwise
    """
    # Build discovered devices lookup by address
    discovered = {}
    for device in device_list_data['devices']:
        addr = device['address']
        discovered[addr] = {
            'device_type': device.get('device_type', 'Unknown'),
            'version': device.get('version', 'Unknown')
        }

    # Build expected addresses from axes
    expected_addresses = set()
    device_by_address = {}  # Maps address to (name, type) tuple

    for axis in axis_config_data.get('axes', []):
        addr = axis['address']
        expected_addresses.add(addr)
        device_by_address[addr] = (axis['name'], 'axis')

    # Add IO devices
    for io_device in axis_config_data.get('io_devices', []):
        addr = io_device['address']
        expected_addresses.add(addr)
        device_by_address[addr] = (io_device.get('name', 'IO'), 'io')

    discovered_addresses = set(discovered.keys())

    # Find differences
    missing_devices = expected_addresses - discovered_addresses
    extra_devices = discovered_addresses - expected_addresses
    matching_devices = expected_addresses & discovered_addresses

    differences_found = False

    # Report missing devices
    if missing_devices:
        differences_found = True
        if verbose:
            print(f"\nMissing devices (in {axis_config_name} but not discovered):")
            for addr in sorted(missing_devices):
                device_name, device_kind = device_by_address.get(addr, ('Unknown', 'unknown'))
                print(f"  - Address {addr}: Expected for {device_kind} '{device_name}'")

    # Report extra devices
    if extra_devices:
        differences_found = True
        if verbose:
            print(f"\nExtra devices (discovered but not in {axis_config_name}):")
            for addr in sorted(extra_devices):
                device_type = discovered[addr]['device_type']
                version = discovered[addr]['version']
                print(f"  + Address {addr}: {device_type} v{version}")

    # Report matching devices
    if verbose:
        if matching_devices and not differences_found:
            print(f"\nAll {len(matching_devices)} expected devices found:")
            for addr in sorted(matching_devices):
                device_name, device_kind = device_by_address.get(addr, ('Unknown', 'unknown'))
                device_type = discovered[addr]['device_type']
                version = discovered[addr]['version']
                print(f"  ✓ Address {addr} ({device_kind} '{device_name}'): {device_type} v{version}")

        if not differences_found:
            print("\nNo differences found (all expected devices present)")
        elif matching_devices:
            print(f"\nMatching devices ({len(matching_devices)} found):")
            for addr in sorted(matching_devices):
                device_name, device_kind = device_by_address.get(addr, ('Unknown', 'unknown'))
                device_type = discovered[addr]['device_type']
                version = discovered[addr]['version']
                print(f"  ✓ Address {addr} ({device_kind} '{device_name}'): {device_type} v{version}")

    return differences_found


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


def _diff_device_lists(data1: Dict, data2: Dict, name1: str, name2: str, verbose: bool = True) -> bool:
    """Compare two device lists.

    Only compares core device identification fields:
    - address
    - device_type (model)
    - version (firmware)

    Ignores extra configuration data like axis parameters, gains, etc.

    Returns:
        True if differences were found, False otherwise
    """
    devices1 = {d['address']: d for d in data1['devices']}
    devices2 = {d['address']: d for d in data2['devices']}

    all_addresses = sorted(set(devices1.keys()) | set(devices2.keys()))

    differences_found = False

    # Core fields to compare (ignore device_id as it's redundant with device_type)
    core_fields = ['device_type', 'version']

    for addr in all_addresses:
        if addr not in devices1:
            differences_found = True
            if verbose:
                device_type = devices2[addr].get('device_type', 'Unknown')
                version = devices2[addr].get('version', 'Unknown')
                print(f"+ Address {addr}: {device_type} v{version} (only in {name2})")
        elif addr not in devices2:
            differences_found = True
            if verbose:
                device_type = devices1[addr].get('device_type', 'Unknown')
                version = devices1[addr].get('version', 'Unknown')
                print(f"- Address {addr}: {device_type} v{version} (only in {name1})")
        else:
            # Compare only core identification fields
            d1, d2 = devices1[addr], devices2[addr]
            device_differs = False
            differences = []

            for field in core_fields:
                val1 = d1.get(field)
                val2 = d2.get(field)
                if val1 != val2:
                    device_differs = True
                    differences.append(f"    {field}: {val1} -> {val2}")

            if device_differs:
                differences_found = True
                if verbose:
                    device_type1 = d1.get('device_type', 'Unknown')
                    device_type2 = d2.get('device_type', 'Unknown')
                    print(f"~ Address {addr}: {device_type1} vs {device_type2}")
                    for diff in differences:
                        print(diff)

    if not differences_found and verbose:
        print("No differences found (devices, addresses, models, and firmware match)")

    return differences_found


def _diff_axis_configs(data1: Dict, data2: Dict, name1: str, name2: str, verbose: bool = True) -> bool:
    """Compare two axis configurations.

    Returns:
        True if differences were found, False otherwise
    """
    axes1 = {a['name']: a for a in data1['axes']}
    axes2 = {a['name']: a for a in data2['axes']}

    all_names = sorted(set(axes1.keys()) | set(axes2.keys()))

    differences_found = False

    for name in all_names:
        if name not in axes1:
            differences_found = True
            if verbose:
                print(f"+ Axis {name}: (only in {name2})")
        elif name not in axes2:
            differences_found = True
            if verbose:
                print(f"- Axis {name}: (only in {name1})")
        elif axes1[name] != axes2[name]:
            differences_found = True
            if verbose:
                print(f"~ Axis {name}: differs")

    if not differences_found and verbose:
        print("No differences found")

    return differences_found


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
        description='LDCN Network Configuration Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    # Global verbose flag
    parser.add_argument('--verbose', '-v', action='store_true',
        help='Enable verbose output (default: silent except errors)')

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

    # init command
    init_parser = subparsers.add_parser('init',
        help='Initialize network, verify devices, set gains')
    init_parser.add_argument('--port', default='/dev/ttyUSB0',
        help='Serial port (default: /dev/ttyUSB0)')
    init_parser.add_argument('--baud', type=int, default=125000,
        help='Baud rate (default: 125000)')
    init_parser.add_argument('--config', required=True,
        help='Path to axis configuration JSON file')
    init_parser.set_defaults(func=cmd_init)

    args = parser.parse_args()

    # Execute command
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
