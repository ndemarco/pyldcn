#!/usr/bin/env python3
"""
axis_config.py - Axis configuration validation and management

Author: LinuxCNC Community
License: GPL v2 or later
"""

import json
from typing import Dict, List, Optional

from .exceptions import ConfigError, ConfigValidationError
from .schema import VALID_AXIS_NAMES, AXIS_CONFIG_SCHEMA


class AxisConfig:
    """
    Axis configuration validator and manager.

    Validates axis_config.json format and provides safe access to configuration data.
    """

    def __init__(self, config_data: Optional[Dict] = None):
        """
        Initialize axis configuration.

        Args:
            config_data: Configuration dictionary (if None, creates empty config)
        """
        if config_data is None:
            config_data = {
                "file_version": "1.0",
                "axes": []
            }

        self.config = config_data
        self.validate()

    @classmethod
    def from_file(cls, filename: str) -> 'AxisConfig':
        """
        Load axis configuration from JSON file.

        Args:
            filename: Path to axis_config.json

        Returns:
            AxisConfig object

        Raises:
            ConfigError: If file cannot be read or parsed
            ConfigValidationError: If validation fails
        """
        try:
            with open(filename, 'r') as f:
                config_data = json.load(f)
        except FileNotFoundError:
            raise ConfigError(f"Axis config file not found: {filename}")
        except json.JSONDecodeError as e:
            raise ConfigError(f"Invalid JSON in axis config file: {e}")

        return cls(config_data)

    def save(self, filename: str) -> None:
        """
        Save axis configuration to JSON file.

        Args:
            filename: Path to save file
        """
        with open(filename, 'w') as f:
            json.dump(self.config, f, indent=2)

    def validate(self) -> None:
        """
        Validate axis configuration structure and values.

        Raises:
            ConfigValidationError: If validation fails
        """
        # Check file version
        if 'file_version' not in self.config:
            raise ConfigValidationError("Missing file_version field")

        if self.config['file_version'] != "1.0":
            raise ConfigValidationError(f"Unsupported file version: {self.config['file_version']}")

        # Check axes list
        if 'axes' not in self.config:
            raise ConfigValidationError("Missing axes field")

        if not isinstance(self.config['axes'], list):
            raise ConfigValidationError("axes field must be a list")

        # Validate each axis
        for i, axis in enumerate(self.config['axes']):
            self._validate_axis(axis, i)

        # Check for duplicate axis names
        axis_names = [axis['name'] for axis in self.config['axes']]
        if len(axis_names) != len(set(axis_names)):
            raise ConfigValidationError("Duplicate axis names found")

        # Check for duplicate addresses
        addresses = [axis['address'] for axis in self.config['axes']]
        if len(addresses) != len(set(addresses)):
            raise ConfigValidationError("Duplicate LDCN addresses found")

    def _validate_axis(self, axis: Dict, index: int) -> None:
        """
        Validate a single axis configuration.

        Args:
            axis: Axis configuration dictionary
            index: Axis index (for error messages)

        Raises:
            ConfigValidationError: If validation fails
        """
        prefix = f"Axis {index}"

        # Required fields
        required_fields = ['name', 'address', 'axis_type', 'pitch', 'encoder_resolution',
                          'gear_ratio', 'invert_direction', 'gains', 'homing', 'limits', 'motion']

        for field in required_fields:
            if field not in axis:
                raise ConfigValidationError(f"{prefix}: Missing required field '{field}'")

        # Validate name
        if axis['name'] not in VALID_AXIS_NAMES:
            raise ConfigValidationError(f"{prefix}: Invalid axis name '{axis['name']}'. "
                                       f"Must be one of {VALID_AXIS_NAMES}")

        # Validate address
        if not isinstance(axis['address'], int) or axis['address'] < 1 or axis['address'] > 127:
            raise ConfigValidationError(f"{prefix}: Address must be 1-127")

        # Validate axis type
        if axis['axis_type'] not in ['linear', 'rotary']:
            raise ConfigValidationError(f"{prefix}: axis_type must be 'linear' or 'rotary'")

        # Validate physical parameters
        if axis['pitch'] <= 0:
            raise ConfigValidationError(f"{prefix}: pitch must be positive")

        if axis['encoder_resolution'] <= 0:
            raise ConfigValidationError(f"{prefix}: encoder_resolution must be positive")

        if not isinstance(axis['gear_ratio'], list) or len(axis['gear_ratio']) != 2:
            raise ConfigValidationError(f"{prefix}: gear_ratio must be [numerator, denominator]")

        if axis['gear_ratio'][0] <= 0 or axis['gear_ratio'][1] <= 0:
            raise ConfigValidationError(f"{prefix}: gear_ratio values must be positive")

        # Validate gains
        self._validate_gains(axis['gains'], prefix)

        # Validate homing
        self._validate_homing(axis['homing'], prefix)

        # Validate limits
        self._validate_limits(axis['limits'], prefix)

        # Validate motion
        self._validate_motion(axis['motion'], prefix)

    def _validate_gains(self, gains: Dict, prefix: str) -> None:
        """Validate PID gains section."""
        required_fields = ['kp', 'kd', 'ki', 'il', 'ol', 'cl', 'el', 'sr', 'dbc']

        for field in required_fields:
            if field not in gains:
                raise ConfigValidationError(f"{prefix}: Missing gain field '{field}'")

        # Validate ranges
        if gains['kp'] < 0 or gains['kp'] > 65535:
            raise ConfigValidationError(f"{prefix}: kp must be 0-65535")

        if gains['kd'] < 0 or gains['kd'] > 65535:
            raise ConfigValidationError(f"{prefix}: kd must be 0-65535")

        if gains['ki'] < 0 or gains['ki'] > 65535:
            raise ConfigValidationError(f"{prefix}: ki must be 0-65535")

        if gains['il'] < 0 or gains['il'] > 255:
            raise ConfigValidationError(f"{prefix}: il must be 0-255")

        if gains['ol'] < 0 or gains['ol'] > 255:
            raise ConfigValidationError(f"{prefix}: ol must be 0-255")

        if gains['cl'] < 0 or gains['cl'] > 255:
            raise ConfigValidationError(f"{prefix}: cl must be 0-255")

        if gains['el'] < 0 or gains['el'] > 65535:
            raise ConfigValidationError(f"{prefix}: el must be 0-65535")

        if gains['sr'] < 0 or gains['sr'] > 255:
            raise ConfigValidationError(f"{prefix}: sr must be 0-255")

        if gains['dbc'] < 0 or gains['dbc'] > 255:
            raise ConfigValidationError(f"{prefix}: dbc must be 0-255")

    def _validate_homing(self, homing: Dict, prefix: str) -> None:
        """Validate homing section."""
        required_fields = ['enabled', 'home_switch', 'invert_direction', 'use_index_pulse',
                          'home_distance', 'start_velocity', 'end_velocity', 'acceleration']

        for field in required_fields:
            if field not in homing:
                raise ConfigValidationError(f"{prefix}: Missing homing field '{field}'")

        if homing['home_switch'] < 0 or homing['home_switch'] > 15:
            raise ConfigValidationError(f"{prefix}: home_switch must be 0-15")

        if homing['start_velocity'] <= 0:
            raise ConfigValidationError(f"{prefix}: start_velocity must be positive")

        if homing['end_velocity'] <= 0:
            raise ConfigValidationError(f"{prefix}: end_velocity must be positive")

        if homing['acceleration'] <= 0:
            raise ConfigValidationError(f"{prefix}: acceleration must be positive")

    def _validate_limits(self, limits: Dict, prefix: str) -> None:
        """Validate limits section."""
        required_sections = ['hard_limit_negative', 'hard_limit_positive',
                           'soft_limit_negative', 'soft_limit_positive']

        for section in required_sections:
            if section not in limits:
                raise ConfigValidationError(f"{prefix}: Missing limits section '{section}'")

        # Validate hard limits
        for hl in ['hard_limit_negative', 'hard_limit_positive']:
            if 'enabled' not in limits[hl]:
                raise ConfigValidationError(f"{prefix}: Missing {hl}.enabled")
            if 'active_low' not in limits[hl]:
                raise ConfigValidationError(f"{prefix}: Missing {hl}.active_low")

        # Validate soft limits
        for sl in ['soft_limit_negative', 'soft_limit_positive']:
            if 'enabled' not in limits[sl]:
                raise ConfigValidationError(f"{prefix}: Missing {sl}.enabled")
            if 'position' not in limits[sl]:
                raise ConfigValidationError(f"{prefix}: Missing {sl}.position")

        # Check that soft limits are ordered correctly
        if limits['soft_limit_negative']['position'] >= limits['soft_limit_positive']['position']:
            raise ConfigValidationError(f"{prefix}: soft_limit_negative must be less than soft_limit_positive")

    def _validate_motion(self, motion: Dict, prefix: str) -> None:
        """Validate motion parameters section."""
        required_fields = ['max_velocity', 'max_acceleration', 'max_deceleration',
                          'acceleration_jerk', 'deceleration_jerk']

        for field in required_fields:
            if field not in motion:
                raise ConfigValidationError(f"{prefix}: Missing motion field '{field}'")

        # All motion parameters must be positive
        for field in required_fields:
            if motion[field] <= 0:
                raise ConfigValidationError(f"{prefix}: {field} must be positive")

    def get_axis_by_name(self, name: str) -> Optional[Dict]:
        """
        Get axis configuration by name.

        Args:
            name: Axis name (e.g., 'X', 'Y', 'Z')

        Returns:
            Axis configuration dictionary or None if not found
        """
        for axis in self.config['axes']:
            if axis['name'] == name:
                return axis
        return None

    def get_axis_by_address(self, address: int) -> Optional[Dict]:
        """
        Get axis configuration by LDCN address.

        Args:
            address: LDCN device address (1-127)

        Returns:
            Axis configuration dictionary or None if not found
        """
        for axis in self.config['axes']:
            if axis['address'] == address:
                return axis
        return None

    def get_axes(self) -> List[Dict]:
        """
        Get all axis configurations.

        Returns:
            List of axis configuration dictionaries
        """
        return self.config['axes']


def create_example_axis_config(filename: str) -> None:
    """
    Create an example axis_config.json file.

    Args:
        filename: Path to save example file
    """
    example_config = {
        "file_version": "1.0",
        "axes": []
    }

    # Add example X axis (linear)
    example_config['axes'].append(AXIS_CONFIG_SCHEMA['axes'][0])

    with open(filename, 'w') as f:
        json.dump(example_config, f, indent=2)
