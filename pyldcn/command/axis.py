#!/usr/bin/env python3
"""
axis.py - High-level axis control for LS-231SE servo drives

Provides named axis control using configuration from JSON files.
Commands use physical units (mm, deg, mm/s) and configuration defaults.

This module is designed to be imported and used as a Python library:
    from pyldcn.command import AxisController
    controller = AxisController(network, config_file)
    controller.home_axis('X')

Author: NickyDoes
License: GPL v2 or later
Date: 2025-11-07
"""

import json
import time
import logging
from typing import Dict, Optional, Any
from pathlib import Path

# Set up logging
logger = logging.getLogger(__name__)


class AxisConfig:
    """Configuration for a single axis"""

    def __init__(self, config_dict: Dict[str, Any]):
        """Initialize axis configuration from dict"""
        self.name = config_dict['name']
        self.address = config_dict['address']
        self.axis_type = config_dict['axis_type']
        self.pitch = config_dict['pitch']
        self.encoder_resolution = config_dict['encoder_resolution']
        self.gear_ratio = tuple(config_dict['gear_ratio'])
        self.invert_direction = config_dict.get('invert_direction', False)

        # Gains
        gains = config_dict['gains']
        self.gains = {
            'kp': gains['kp'],
            'kd': gains['kd'],
            'ki': gains['ki'],
            'il': gains['il'],
            'ol': gains['ol'],
            'cl': gains['cl'],
            'el': gains['el'],
            'sr': gains['sr'],
            'dbc': gains['dbc']
        }

        # Homing
        homing = config_dict.get('homing', {})
        self.homing = {
            'enabled': homing.get('enabled', False),
            'home_switch': homing.get('home_switch', 0),
            'invert_direction': homing.get('invert_direction', False),
            'use_index_pulse': homing.get('use_index_pulse', True),
            'home_distance': homing.get('home_distance', 5.0),
            'start_velocity': homing.get('start_velocity', 20.0),
            'end_velocity': homing.get('end_velocity', 5.0),
            'acceleration': homing.get('acceleration', 100.0)
        }

        # Motion limits
        motion = config_dict.get('motion', {})
        self.motion = {
            'max_velocity': motion.get('max_velocity', 100.0),
            'max_acceleration': motion.get('max_acceleration', 100.0),
            'max_deceleration': motion.get('max_deceleration', 100.0),
            'acceleration_jerk': motion.get('acceleration_jerk', 10000.0),
            'deceleration_jerk': motion.get('deceleration_jerk', 10000.0)
        }

        # Soft limits
        limits = config_dict.get('limits', {})
        self.soft_limit_positive = limits.get('soft_limit_positive', {}).get('position')
        self.soft_limit_negative = limits.get('soft_limit_negative', {}).get('position')

    @property
    def counts_per_unit(self) -> float:
        """Calculate encoder counts per physical unit (mm or deg)"""
        # counts/rev * gear_ratio / pitch(mm/rev or deg/rev)
        return (self.encoder_resolution * self.gear_ratio[0]) / (self.pitch * self.gear_ratio[1])

    def position_to_counts(self, position: float) -> int:
        """Convert physical position to encoder counts"""
        return int(position * self.counts_per_unit)

    def velocity_to_counts(self, velocity: float) -> int:
        """Convert physical velocity to encoder counts/sec"""
        return int(velocity * self.counts_per_unit)

    def accel_to_counts(self, accel: float) -> int:
        """Convert physical acceleration to encoder counts/sec²"""
        return int(accel * self.counts_per_unit)


class AxisController:
    """
    High-level axis controller using named axes from configuration.

    Provides commands to initialize, move, and stop axes by name,
    using physical units and configuration defaults.

    Example:
        from pyldcn import LDCNNetwork
        from pyldcn.command import AxisController

        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        controller = AxisController(network, 'config.json')

        # Initialize axis (set gains, home)
        controller.initialize_axis('X')

        # Move to position
        controller.move_absolute('X', position=100.0, velocity=50.0, accel=10.0)

        # Stop
        controller.stop('X')
    """

    def __init__(self, network, config_path: Optional[str] = None):
        """
        Initialize axis controller.

        Args:
            network: LDCNNetwork instance (must be initialized)
            config_path: Path to axis configuration JSON file (optional)
        """
        self.network = network
        self.axes: Dict[str, AxisConfig] = {}

        if config_path:
            self.load_config(config_path)

    def load_config(self, config_path: str):
        """
        Load axis configuration from JSON file.

        Args:
            config_path: Path to axis configuration JSON file
        """
        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(path, 'r') as f:
            config = json.load(f)

        # Load each axis configuration
        for axis_dict in config.get('axes', []):
            axis_config = AxisConfig(axis_dict)
            self.axes[axis_config.name] = axis_config

    def get_axis(self, name: str) -> AxisConfig:
        """
        Get axis configuration by name.

        Args:
            name: Axis name (e.g., 'X', 'Y', 'Z')

        Returns:
            AxisConfig object

        Raises:
            KeyError: If axis name not found in configuration
        """
        if name not in self.axes:
            raise KeyError(f"Axis '{name}' not found in configuration. Available: {list(self.axes.keys())}")
        return self.axes[name]

    def get_device(self, name: str):
        """
        Get LDCN device for axis by name.

        Args:
            name: Axis name

        Returns:
            LDCNDevice object

        Raises:
            KeyError: If axis not found
            ValueError: If device at axis address not found
        """
        axis = self.get_axis(name)

        for device in self.network.devices:
            if device.address == axis.address:
                return device

        raise ValueError(f"No device found at address {axis.address} for axis '{name}'")

    def initialize_axis(self, name: str,
                       set_gains: bool = True,
                       home: bool = False,
                       **gain_overrides):
        """
        Initialize axis by setting gains and optionally homing.

        Args:
            name: Axis name
            set_gains: Set PID gains from config (default: True)
            home: Perform homing sequence (default: False)
            **gain_overrides: Override specific gain values (kp, kd, ki, etc.)

        Example:
            # Use config defaults
            controller.initialize_axis('X')

            # Override specific gains
            controller.initialize_axis('Y', kp=15, kd=1200)

            # Set gains and home
            controller.initialize_axis('Z', home=True)
        """
        axis = self.get_axis(name)
        device = self.get_device(name)

        if set_gains:
            # Merge config gains with overrides
            gains = axis.gains.copy()
            gains.update(gain_overrides)

            logger.info(f"Setting gains for axis '{name}' (address {axis.address}):")
            logger.info(f"  KP={gains['kp']}, KD={gains['kd']}, KI={gains['ki']}")
            logger.info(f"  IL={gains['il']}, OL={gains['ol']}, CL={gains['cl']}")
            logger.info(f"  EL={gains['el']}, SR={gains['sr']}, DBC={gains['dbc']}")

            device.set_gains(
                kp=gains['kp'], kd=gains['kd'], ki=gains['ki'],
                il=gains['il'], ol=gains['ol'], cl=gains['cl'],
                el=gains['el'], sr=gains['sr'], db=gains['dbc']
            )

        if home and axis.homing['enabled']:
            logger.info(f"Homing axis '{name}'...")
            self.home_axis(name)

    def home_axis(self, name: str,
                  start_velocity: Optional[float] = None,
                  end_velocity: Optional[float] = None,
                  acceleration: Optional[float] = None,
                  invert_direction: Optional[bool] = None):
        """
        Perform homing sequence for axis.

        The homing direction is determined by homing config:
        - invert_direction=False: Home to Limit 2 (positive/forward direction)
        - invert_direction=True:  Home to Limit 1 (negative/reverse direction)

        Args:
            name: Axis name
            start_velocity: Initial homing velocity (uses config default if None)
            end_velocity: Final homing velocity (uses config default if None)
            acceleration: Homing acceleration (uses config default if None)
            invert_direction: Override homing direction from config

        Example:
            # Use config defaults
            controller.home_axis('X')

            # Override velocities
            controller.home_axis('Y', start_velocity=10.0, end_velocity=2.0)
        """
        axis = self.get_axis(name)
        device = self.get_device(name)

        if not axis.homing['enabled']:
            raise ValueError(f"Homing not enabled for axis '{name}'")

        # Step 1: Enable main power via SK-2310g2
        logger.info(f"\n{'='*60}")
        logger.info("ENABLING MAIN POWER")
        logger.info("=" *60)

        io_controller = None
        for dev in self.network.devices:
            if hasattr(dev, 'device_type') and dev.device_type == 'SK-2310g2':
                io_controller = dev
                break

        if not io_controller:
            raise RuntimeError("SK-2310g2 I/O controller not found on network")

        # Configure SK-2310g2 for full status reporting (required before setting outputs)
        logger.info("Configuring SK-2310g2 I/O controller...")
        io_controller.configure()

        # Enable power relay (Output 15 = System Lock / Power ON/OFF)
        # This will make the power button flash
        logger.info("Enabling power relay (Output 15)...")
        io_controller.set_outputs(0x8000)

        # Step 2: Prompt user and wait for power button press
        io_controller.request_power_on()

        # Wait for power to be enabled (check SK-2310g2 diagnostic code change)
        # Following linuxcnc-logosol approach: read baseline, wait for change to >= 0x08
        logger.info("  Waiting for power on (monitoring SK-2310g2 diagnostic)...")
        logger.info("  Logging diagnostic transitions to: power_on_diagnostic.log")

        # Open log file
        log_file = open('power_on_diagnostic.log', 'w')
        log_file.write("Time(s),Diagnostic,Binary,Description\n")
        log_file.flush()

        # Read baseline diagnostic (power OFF state)
        baseline_diag = io_controller.read_diagnostic()
        logger.info(f"  Baseline diagnostic: 0x{baseline_diag:02X}")
        log_file.write(f"0.000,0x{baseline_diag:02X},{baseline_diag:08b},Baseline\n")
        log_file.flush()

        timeout = 30
        start_time = time.time()
        last_message_time = start_time
        last_diag = baseline_diag
        poll_count = 0

        try:
            while True:
                poll_count += 1
                diag = io_controller.read_diagnostic()
                current_time = time.time()
                elapsed = current_time - start_time

                # Log every diagnostic read (including duplicates for timing analysis)
                if diag != last_diag or poll_count % 10 == 0:  # Log changes and every 10th poll
                    diag_desc = {
                        0x00: "Power OFF delay",
                        0x04: "Control voltage LOW",
                        0x08: "System LOCKED",
                        0x0C: "Cover Open Stop",
                        0x10: "Emergency Stop",
                        0x14: "Ready to power",
                        0x1C: "Power ON, Covers Open",
                        0x1F: "Power ON, Covers Closed"
                    }.get(diag, "Unknown")

                    log_file.write(f"{elapsed:.3f},0x{diag:02X},{diag:08b},{diag_desc}\n")
                    log_file.flush()

                    if diag != last_diag:
                        logger.info(f"  [{elapsed:.2f}s] Diagnostic changed: 0x{diag:02X} ({diag_desc})")

                # Power button detection (following linuxcnc-logosol logic):
                # Detect when diagnostic CHANGES from baseline AND is >= 0x08
                # Baseline is typically 0x00 or 0x04 (power OFF)
                # Button press changes to 0x08+ (operational states)
                if diag != baseline_diag and diag >= 0x08:
                    logger.info(f"\n✓ Power button pressed (diagnostic: 0x{diag:02X})")
                    break

                last_diag = diag

                # Print periodic updates
                if current_time - last_message_time >= 2.0:
                    logger.info(f"  Waiting... ({elapsed:.1f}s, diagnostic: 0x{diag:02X}, polls: {poll_count})")
                    last_message_time = current_time

                if elapsed > timeout:
                    raise TimeoutError(
                        f"Timeout waiting for power button press. Last diagnostic: 0x{diag:02X}"
                    )

                time.sleep(0.05)  # Poll rapidly (50ms)

        finally:
            log_file.close()
            logger.info(f"  Diagnostic log saved to: power_on_diagnostic.log")

        logger.info(f"{'='*60}\n")

        # Step 3: Enable amplifier before homing (required for motion)
        logger.info(f"\nEnabling amplifier for axis '{name}'...")
        device.enable()

        # Use config defaults or overrides
        start_vel = start_velocity if start_velocity is not None else axis.homing['start_velocity']
        end_vel = end_velocity if end_velocity is not None else axis.homing['end_velocity']
        accel = acceleration if acceleration is not None else axis.homing['acceleration']
        invert = invert_direction if invert_direction is not None else axis.homing['invert_direction']

        # Convert to counts
        start_vel_counts = axis.velocity_to_counts(start_vel)
        end_vel_counts = axis.velocity_to_counts(end_vel)
        accel_counts = axis.accel_to_counts(accel)

        # Determine home switch (Limit 1 or Limit 2)
        # invert_direction=False → home to Limit 2 (forward)
        # invert_direction=True  → home to Limit 1 (reverse)
        home_switch = 1 if invert else 2

        logger.info(f"Homing axis '{name}' to Limit {home_switch}:")
        logger.info(f"  Start velocity: {start_vel:.1f} ({start_vel_counts} counts/s)")
        logger.info(f"  End velocity: {end_vel:.1f} ({end_vel_counts} counts/s)")
        logger.info(f"  Acceleration: {accel:.1f} ({accel_counts} counts/s²)")
        logger.info(f"  Use index pulse: {axis.homing['use_index_pulse']}")

        # Perform homing sequence
        device.home_to_limit(
            limit_switch=home_switch,
            velocity=start_vel_counts,
            accel=accel_counts,
            use_index=axis.homing['use_index_pulse'],
            index_velocity=end_vel_counts
        )

    def move_absolute(self, name: str,
                      position: float,
                      velocity: Optional[float] = None,
                      accel: Optional[float] = None):
        """
        Move axis to absolute position.

        Args:
            name: Axis name
            position: Target position in physical units (mm or deg)
            velocity: Motion velocity (uses config max if None)
            accel: Motion acceleration (uses config max if None)

        Example:
            # Move to 100mm at config max velocity/accel
            controller.move_absolute('X', 100.0)

            # Move with specific velocity and accel
            controller.move_absolute('Y', 50.0, velocity=30.0, accel=5.0)
        """
        axis = self.get_axis(name)
        device = self.get_device(name)

        # Use config defaults if not specified
        vel = velocity if velocity is not None else axis.motion['max_velocity']
        acc = accel if accel is not None else axis.motion['max_acceleration']

        # Check soft limits if configured
        if axis.soft_limit_positive is not None and position > axis.soft_limit_positive:
            raise ValueError(
                f"Position {position} exceeds positive soft limit {axis.soft_limit_positive} "
                f"for axis '{name}'"
            )
        if axis.soft_limit_negative is not None and position < axis.soft_limit_negative:
            raise ValueError(
                f"Position {position} below negative soft limit {axis.soft_limit_negative} "
                f"for axis '{name}'"
            )

        # Convert to counts
        position_counts = axis.position_to_counts(position)
        velocity_counts = axis.velocity_to_counts(vel)
        accel_counts = axis.accel_to_counts(acc)

        logger.info(f"Moving axis '{name}' to position {position:.3f}:")
        logger.info(f"  Position: {position_counts} counts")
        logger.info(f"  Velocity: {vel:.1f} ({velocity_counts} counts/s)")
        logger.info(f"  Acceleration: {acc:.1f} ({accel_counts} counts/s²)")

        # Execute move
        device.move_to_counts(position_counts, velocity_counts, accel_counts)

    def move_relative(self, name: str,
                      distance: float,
                      velocity: Optional[float] = None,
                      accel: Optional[float] = None):
        """
        Move axis relative to current position.

        Args:
            name: Axis name
            distance: Relative distance in physical units (mm or deg)
            velocity: Motion velocity (uses config max if None)
            accel: Motion acceleration (uses config max if None)

        Example:
            # Move 25mm forward at config velocity
            controller.move_relative('X', 25.0)

            # Move 10mm backward at 20mm/s
            controller.move_relative('Y', -10.0, velocity=20.0)
        """
        axis = self.get_axis(name)
        device = self.get_device(name)

        # Get current position
        status = device.read_status()
        current_counts = status.get('position', 0)
        current_position = current_counts / axis.counts_per_unit

        # Calculate target position
        target_position = current_position + distance

        logger.info(f"Moving axis '{name}' relative {distance:.3f}:")
        logger.info(f"  Current position: {current_position:.3f}")
        logger.info(f"  Target position: {target_position:.3f}")

        # Use absolute move with calculated target
        self.move_absolute(name, target_position, velocity, accel)

    def stop(self, name: str):
        """
        Stop axis motion immediately.

        Args:
            name: Axis name

        Example:
            controller.stop('X')
        """
        axis = self.get_axis(name)
        device = self.get_device(name)

        logger.info(f"Stopping axis '{name}' (address {axis.address})")
        device.stop_motor()

    def wait_for_motion_complete(self, name: str,
                                 timeout: Optional[float] = None,
                                 poll_interval: float = 0.1):
        """
        Wait for axis motion to complete.

        Args:
            name: Axis name
            timeout: Maximum wait time in seconds (None = infinite)
            poll_interval: Status polling interval in seconds

        Returns:
            True if motion completed, False if timeout

        Example:
            controller.move_absolute('X', 100.0)
            if controller.wait_for_motion_complete('X', timeout=10.0):
                logger.info("Motion complete")
            else:
                logger.info("Motion timeout")
        """
        axis = self.get_axis(name)
        device = self.get_device(name)

        start_time = time.time()

        while True:
            status = device.read_status()

            # Check if motion is done (bit 0 of status_byte)
            if status.get('move_done', False):
                return True

            # Check timeout
            if timeout is not None and (time.time() - start_time) > timeout:
                return False

            time.sleep(poll_interval)

    def get_position(self, name: str) -> float:
        """
        Get current axis position in physical units.

        Args:
            name: Axis name

        Returns:
            Current position in physical units (mm or deg)

        Example:
            pos = controller.get_position('X')
            logger.info(f"X position: {pos:.3f} mm")
        """
        axis = self.get_axis(name)
        device = self.get_device(name)

        status = device.read_status()
        position_counts = status.get('position', 0)
        position = position_counts / axis.counts_per_unit

        return position
