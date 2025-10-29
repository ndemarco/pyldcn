#!/usr/bin/env python3
"""
merger.py - Configuration merge utilities

Merges device_list.json with axis_config.json to create configured_devices.json

Author: LinuxCNC Community
License: GPL v2 or later
"""

import json
from typing import Dict
from datetime import datetime

from .exceptions import ConfigError, ConfigValidationError, ConfigMergeError
from .axis_config import AxisConfig


class ConfigMerger:
    """
    Merges device_list.json with axis_config.json to create configured_devices.json.

    The merge process:
    1. Load device_list.json (hardware facts from discovery)
    2. Load axis_config.json (user-defined configuration)
    3. Match devices by address
    4. Merge configuration with device info
    5. Validate that all configured axes have corresponding devices
    6. Save to configured_devices.json
    """

    def __init__(self, device_list_file: str, axis_config_file: str):
        """
        Initialize config merger.

        Args:
            device_list_file: Path to device_list.json
            axis_config_file: Path to axis_config.json

        Raises:
            ConfigError: If files cannot be loaded
            ConfigValidationError: If validation fails
        """
        # Load device list
        try:
            with open(device_list_file, 'r') as f:
                self.device_list = json.load(f)
        except FileNotFoundError:
            raise ConfigError(f"Device list file not found: {device_list_file}")
        except json.JSONDecodeError as e:
            raise ConfigError(f"Invalid JSON in device list: {e}")

        # Validate device list
        self._validate_device_list()

        # Load axis config
        self.axis_config = AxisConfig.from_file(axis_config_file)

    def _validate_device_list(self) -> None:
        """
        Validate device list structure.

        Raises:
            ConfigValidationError: If validation fails
        """
        required_fields = ['file_version', 'port', 'baud_rate', 'num_devices', 'devices']
        for field in required_fields:
            if field not in self.device_list:
                raise ConfigValidationError(f"Device list missing required field: {field}")

        if self.device_list['file_version'] != "1.0":
            raise ConfigValidationError(f"Unsupported device list version: {self.device_list['file_version']}")

    def merge(self) -> Dict:
        """
        Merge device list with axis configuration.

        Returns:
            Merged configuration dictionary

        Raises:
            ConfigMergeError: If merge fails
        """
        merged_devices = []

        # Create address-to-device mapping
        device_map = {dev['address']: dev for dev in self.device_list['devices']}

        # Process each axis configuration
        for axis in self.axis_config.get_axes():
            address = axis['address']

            # Check if device exists at this address
            if address not in device_map:
                raise ConfigMergeError(f"Axis '{axis['name']}' configured for address {address}, "
                                      f"but no device found at that address")

            device = device_map[address]

            # Verify device is a servo drive (LS-231SE)
            if device['device_type'] != 'LS-231SE':
                raise ConfigMergeError(f"Axis '{axis['name']}' at address {address} is configured as servo, "
                                      f"but device is {device['device_type']}")

            # Create merged device entry
            merged_device = {
                # Device hardware facts
                "address": address,
                "device_id": device['device_id'],
                "device_type": device['device_type'],
                "version": device['version'],

                # Axis configuration
                "axis_name": axis['name'],
                "axis_type": axis['axis_type'],

                # Physical parameters
                "pitch": axis['pitch'],
                "encoder_resolution": axis['encoder_resolution'],
                "gear_ratio": axis['gear_ratio'],
                "invert_direction": axis['invert_direction'],

                # PID tuning
                "gains": axis['gains'],

                # Homing
                "homing": axis['homing'],

                # Limits
                "limits": axis['limits'],

                # Motion
                "motion": axis['motion']
            }

            merged_devices.append(merged_device)

        # Include unconfigured devices (like SK-2310g2 supervisor)
        for device in self.device_list['devices']:
            address = device['address']

            # Check if device was already included in axis config
            if any(d['address'] == address for d in merged_devices):
                continue

            # Add unconfigured device
            merged_device = {
                "address": address,
                "device_id": device['device_id'],
                "device_type": device['device_type'],
                "version": device['version'],
                "configured": False  # Mark as unconfigured
            }

            merged_devices.append(merged_device)

        # Sort by address
        merged_devices.sort(key=lambda d: d['address'])

        # Create final configuration
        merged_config = {
            "file_version": "1.0",
            "merged_at": datetime.now().isoformat(),
            "source_device_list": self.device_list.get('discovered_at', 'unknown'),
            "port": self.device_list['port'],
            "baud_rate": self.device_list['baud_rate'],
            "num_devices": len(merged_devices),
            "num_configured_axes": len(self.axis_config.get_axes()),
            "devices": merged_devices
        }

        return merged_config

    def save(self, output_file: str) -> None:
        """
        Merge and save to output file.

        Args:
            output_file: Path to save merged configuration (configured_devices.json)

        Raises:
            ConfigMergeError: If merge fails
        """
        merged_config = self.merge()

        with open(output_file, 'w') as f:
            json.dump(merged_config, f, indent=2)
