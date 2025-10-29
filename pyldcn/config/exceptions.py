#!/usr/bin/env python3
"""
exceptions.py - Configuration-related exceptions

Author: LinuxCNC Community
License: GPL v2 or later
"""


class ConfigError(Exception):
    """Base exception for configuration errors."""
    pass


class ConfigValidationError(ConfigError):
    """Configuration validation failed."""
    pass


class ConfigMergeError(ConfigError):
    """Configuration merge failed."""
    pass
