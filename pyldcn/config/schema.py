#!/usr/bin/env python3
"""
schema.py - Configuration schema definitions and constants

Author: LinuxCNC Community
License: GPL v2 or later
"""

from enum import IntEnum


# =============================================================================
# Constants
# =============================================================================

# Axis types
class AxisType(IntEnum):
    """Axis type enumeration."""
    LINEAR = 0
    ROTARY = 1


# Valid axis names
VALID_AXIS_NAMES = ['X', 'Y', 'Z', 'A', 'B', 'C', 'U', 'V', 'W']


# =============================================================================
# Axis Configuration Schema
# =============================================================================

AXIS_CONFIG_SCHEMA = {
    "file_version": "1.0",
    "axes": [
        {
            # Identity
            "name": "X",                    # Axis name (X, Y, Z, A, B, C, U, V, W)
            "address": 2,                   # LDCN device address
            "axis_type": "linear",          # "linear" or "rotary"

            # Physical parameters
            "pitch": 5.0,                   # Ball screw pitch (mm/rev for linear, deg/rev for rotary)
            "encoder_resolution": 10000,    # Encoder counts per revolution
            "gear_ratio": [1, 1],          # [numerator, denominator] gear ratio
            "invert_direction": False,      # Reverse motor direction

            # PID tuning
            "gains": {
                "kp": 10,                   # Proportional gain
                "kd": 1000,                 # Derivative gain
                "ki": 20,                   # Integral gain
                "il": 40,                   # Integration limit
                "ol": 255,                  # Output limit
                "cl": 129,                  # Current limit (0 = disabled)
                "el": 2000,                 # Position error limit (counts)
                "sr": 1,                    # Servo rate divisor
                "dbc": 0                    # Deadband compensation
            },

            # Homing
            "homing": {
                "enabled": True,            # Homing enabled
                "home_switch": 0,           # Home switch input number
                "invert_direction": False,  # Search in negative direction
                "use_index_pulse": True,    # Use encoder index for precision
                "home_distance": 5.0,       # Distance to back off from switch (mm or deg)
                "start_velocity": 20.0,     # Initial search velocity (mm/s or deg/s)
                "end_velocity": 5.0,        # Final approach velocity (mm/s or deg/s)
                "acceleration": 100.0       # Homing acceleration (mm/s² or deg/s²)
            },

            # Limits
            "limits": {
                "hard_limit_negative": {
                    "enabled": True,        # Enable negative hard limit
                    "active_low": False     # Switch is active low
                },
                "hard_limit_positive": {
                    "enabled": True,        # Enable positive hard limit
                    "active_low": False     # Switch is active low
                },
                "soft_limit_negative": {
                    "enabled": True,        # Enable negative soft limit
                    "position": -2.0        # Soft limit position (mm or deg)
                },
                "soft_limit_positive": {
                    "enabled": True,        # Enable positive soft limit
                    "position": 310.0       # Soft limit position (mm or deg)
                }
            },

            # Motion parameters
            "motion": {
                "max_velocity": 500.0,      # Maximum velocity (mm/s or deg/s)
                "max_acceleration": 100.0,  # Maximum acceleration (mm/s² or deg/s²)
                "max_deceleration": 10000.0,# Maximum deceleration (mm/s² or deg/s²)
                "acceleration_jerk": 10000.0,# Acceleration jerk limit (mm/s³ or deg/s³)
                "deceleration_jerk": 10000.0 # Deceleration jerk limit (mm/s³ or deg/s³)
            }
        }
    ]
}
