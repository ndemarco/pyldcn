"""
pyldcn.command - High-level axis control commands

Provides convenient commands for controlling LDCN axes by name using
configuration from JSON files.

Example:
    from pyldcn import LDCNNetwork
    from pyldcn.command import AxisController

    network = LDCNNetwork('/dev/ttyUSB0')
    network.open()
    network.initialize()
    network.set_baud_rate(125000)

    # Load configuration
    controller = AxisController(network, 'fiveaxis_full_config.json')

    # Initialize axis (set gains, home)
    controller.initialize_axis('X')

    # Move to absolute position
    controller.move_absolute('X', position=100.0, velocity=50.0, accel=10.0)

    # Move relative
    controller.move_relative('Y', distance=25.0, velocity=30.0, accel=5.0)

    # Stop motion
    controller.stop('Z')

Author: LinuxCNC Community
License: GPL v2 or later
Date: 2025-11-06
"""

from .axis import AxisController

__all__ = ['AxisController']
