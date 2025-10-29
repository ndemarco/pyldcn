#!/usr/bin/env python3
"""
mctl_importer.py - Import configuration from legacy Mctl_*.ini files

Author: LinuxCNC Community
License: GPL v2 or later
"""

from typing import Dict, Optional

from .exceptions import ConfigError
from .axis_config import AxisConfig


class MCTLImporter:
    """
    Import axis configuration from legacy Mctl_*.ini files.

    Parses MCTL INI format and converts to axis_config.json format.
    """

    def __init__(self, mctl_file: str):
        """
        Initialize MCTL importer.

        Args:
            mctl_file: Path to Mctl_*.ini file

        Raises:
            ConfigError: If file cannot be read
        """
        self.mctl_file = mctl_file
        self.config = self._parse_mctl_file()

    def _parse_mctl_file(self) -> Dict:
        """
        Parse MCTL INI file.

        Returns:
            Dictionary of parsed configuration

        Raises:
            ConfigError: If parsing fails
        """
        try:
            with open(self.mctl_file, 'r') as f:
                lines = f.readlines()
        except FileNotFoundError:
            raise ConfigError(f"MCTL file not found: {self.mctl_file}")

        # Parse INI file into sections
        sections = {}
        current_section = None

        for line in lines:
            line = line.strip()

            # Skip empty lines and comments
            if not line or line.startswith(';') or line.startswith('#'):
                continue

            # Section header
            if line.startswith('[') and line.endswith(']'):
                current_section = line[1:-1]
                sections[current_section] = {}
                continue

            # Key-value pair
            if '=' in line and current_section:
                key, value = line.split('=', 1)
                sections[current_section][key.strip()] = value.strip()

        return sections

    def _parse_axis_section(self, section_name: str, axis_data: Dict) -> Optional[Dict]:
        """
        Parse single axis section from MCTL INI.

        Args:
            section_name: Section name (e.g., "AXIS_X")
            axis_data: Dictionary of axis key-value pairs

        Returns:
            Axis configuration dictionary or None if invalid
        """
        # Extract axis name from section name (AXIS_X -> X)
        if not section_name.startswith('AXIS_'):
            return None

        axis_name = section_name[5:]  # Remove "AXIS_" prefix

        # Parse address
        address = int(axis_data.get('Address', 0))
        if address == 0:
            return None  # Invalid address

        # Parse axis type
        axis_type_val = int(axis_data.get('AxisType', 0))
        axis_type = "rotary" if axis_type_val == 1 else "linear"

        # Parse physical parameters
        pitch = float(axis_data.get('Pitch', 5.0))
        encoder_resolution = int(axis_data.get('EncoderResolution', 10000))
        gear1 = int(axis_data.get('Gear1', 1))
        gear2 = int(axis_data.get('Gear2', 1))
        invert_direction = bool(int(axis_data.get('InvertDirection', 0)))

        # Parse PID gains
        gains = {
            "kp": int(axis_data.get('KP', 10)),
            "kd": int(axis_data.get('KD', 1000)),
            "ki": int(axis_data.get('KI', 20)),
            "il": int(axis_data.get('IL', 40)),
            "ol": int(axis_data.get('OL', 255)),
            "cl": int(axis_data.get('CL', 0)),
            "el": int(axis_data.get('EL', 2000)),
            "sr": int(axis_data.get('SR', 1)),
            "dbc": int(axis_data.get('DBC', 0))
        }

        # Parse homing
        homing = {
            "enabled": True,  # Assume enabled if section exists
            "home_switch": int(axis_data.get('HomeSwitch', 0)),
            "invert_direction": bool(int(axis_data.get('InvertHomeDirection', 0))),
            "use_index_pulse": bool(int(axis_data.get('UseIndexPulse', 1))),
            "home_distance": float(axis_data.get('HomeDistance', 5.0)),
            "start_velocity": float(axis_data.get('HomeStartVel', 20.0)),
            "end_velocity": float(axis_data.get('HomeEndVel', 5.0)),
            "acceleration": float(axis_data.get('HomeAcc', 100.0))
        }

        # Parse limits
        limits = {
            "hard_limit_negative": {
                "enabled": bool(int(axis_data.get('EnableNegativeHardLimit', 1))),
                "active_low": bool(int(axis_data.get('NegativeHardLimitActive', 0)))
            },
            "hard_limit_positive": {
                "enabled": bool(int(axis_data.get('EnablePositiveHardLimit', 1))),
                "active_low": bool(int(axis_data.get('PositiveHardLimitActive', 0)))
            },
            "soft_limit_negative": {
                "enabled": bool(int(axis_data.get('EnableNegativeSoftLimit', 1))),
                "position": float(axis_data.get('NegativeSoftLimit', -100.0))
            },
            "soft_limit_positive": {
                "enabled": bool(int(axis_data.get('EnablePositiveSoftLimit', 1))),
                "position": float(axis_data.get('PositiveSoftLimit', 100.0))
            }
        }

        # Parse motion parameters
        motion = {
            "max_velocity": float(axis_data.get('Velocity', 100.0)),
            "max_acceleration": float(axis_data.get('Acceleration', 100.0)),
            "max_deceleration": float(axis_data.get('Deceleration', 10000.0)),
            "acceleration_jerk": float(axis_data.get('AccelerationJerk', 10000.0)),
            "deceleration_jerk": float(axis_data.get('DecelerationJerk', 10000.0))
        }

        # Build axis configuration
        axis_config = {
            "name": axis_name,
            "address": address,
            "axis_type": axis_type,
            "pitch": pitch,
            "encoder_resolution": encoder_resolution,
            "gear_ratio": [gear1, gear2],
            "invert_direction": invert_direction,
            "gains": gains,
            "homing": homing,
            "limits": limits,
            "motion": motion
        }

        return axis_config

    def convert(self) -> AxisConfig:
        """
        Convert MCTL INI to axis_config.json format.

        Returns:
            AxisConfig object

        Raises:
            ConfigError: If conversion fails
        """
        axes = []

        # Parse each axis section
        for section_name in ['AXIS_X', 'AXIS_Y', 'AXIS_Z', 'AXIS_A', 'AXIS_B', 'AXIS_C']:
            if section_name in self.config:
                axis_config = self._parse_axis_section(section_name, self.config[section_name])
                if axis_config:
                    axes.append(axis_config)

        if not axes:
            raise ConfigError("No valid axis configurations found in MCTL file")

        # Create AxisConfig object
        config_data = {
            "file_version": "1.0",
            "axes": axes
        }

        return AxisConfig(config_data)

    def save(self, output_file: str) -> None:
        """
        Convert and save to axis_config.json.

        Args:
            output_file: Path to save axis configuration

        Raises:
            ConfigError: If conversion or save fails
        """
        axis_config = self.convert()
        axis_config.save(output_file)
