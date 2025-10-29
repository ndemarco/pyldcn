#!/usr/bin/env python3
"""
pyldcn.config - LDCN Configuration Management Package

This package provides configuration file handling for the three-file configuration system:
1. device_list.json - Hardware discovery facts
2. axis_config.json - User-defined axis configuration
3. configured_devices.json - Merged result

Public API:
    - AxisConfig: Validates and manages axis configuration data
    - ConfigMerger: Merges device list with axis config
    - MCTLImporter: Imports from legacy Mctl_*.ini files
    - Exceptions: ConfigError, ConfigValidationError, ConfigMergeError
    - Schema constants: AxisType, VALID_AXIS_NAMES, AXIS_CONFIG_SCHEMA

Author: LinuxCNC Community
License: GPL v2 or later
"""

# Import exceptions
from .exceptions import (
    ConfigError,
    ConfigValidationError,
    ConfigMergeError
)

# Import schema constants
from .schema import (
    AxisType,
    VALID_AXIS_NAMES,
    AXIS_CONFIG_SCHEMA
)

# Import main classes
from .axis_config import AxisConfig, create_example_axis_config
from .merger import ConfigMerger
from .mctl_importer import MCTLImporter

# Define public API
__all__ = [
    # Exceptions
    'ConfigError',
    'ConfigValidationError',
    'ConfigMergeError',

    # Schema
    'AxisType',
    'VALID_AXIS_NAMES',
    'AXIS_CONFIG_SCHEMA',

    # Classes
    'AxisConfig',
    'ConfigMerger',
    'MCTLImporter',

    # Utilities
    'create_example_axis_config',
]
