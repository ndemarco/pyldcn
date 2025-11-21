"""
LS-231SE Servo Drive Motion Subsystem

Position control, trajectory commands, and path point profiles.

Author: NickyDoes
License: GPL v2 or later
"""

import time
import struct
from typing import Optional, List, TYPE_CHECKING

from .servo_state import ServoState

if TYPE_CHECKING:
    from .servo import LS231SE


# =============================================================================
# Servo Drive Commands
# =============================================================================

CMD_LOAD_TRAJECTORY = 0x04
CMD_START_MOTION = 0x05
CMD_SET_HOME_MODE = 0x09

# =============================================================================
# Motor Control Flags (STOP_MOTOR command)
# =============================================================================

AMP_ENABLE = 0x01  # Bit 0: Pic_ae (Power Driver enable)
MOTOR_OFF = 0x02  # Bit 1: Turn motor off
STOP_ABRUPT = 0x04  # Bit 2: Stop abruptly
STOP_SMOOTH = 0x08  # Bit 3: Stop smoothly
STOP_HERE = 0x10  # Bit 4: Stop here


class Motion:
    """
    Position control operations.

    This subsystem handles:
    - Point-to-point moves (trapezoidal profiles)
    - Homing sequences
    - Amplifier enable/disable
    - Position reset
    """

    def __init__(self, state: ServoState, device: "LS231SE"):
        """
        Initialize Motion subsystem.

        Args:
            state: Shared ServoState object
            device: Parent LS231SE device instance
        """
        self._state = state
        self._device = device

    # -------------------------------------------------------------------------
    # Motion Commands
    # -------------------------------------------------------------------------

    def move_to(
        self, position: float, velocity: float, accel: float, scale: float = 1.0
    ) -> None:
        """
        Command motion to absolute position.

        Args:
            position: Target position (physical units, e.g., mm)
            velocity: Velocity (physical units/sec)
            accel: Acceleration (physical units/sec²)
            scale: Counts per physical unit (e.g., 2000 counts/mm)
        """
        position_counts = int(position * scale)
        velocity_counts = int(velocity * scale)
        accel_counts = int(accel * scale)

        self.move_to_counts(position_counts, velocity_counts, accel_counts)

    def move_to_counts(self, position: int, velocity: int, accel: int) -> None:
        """
        Command motion to absolute position (raw counts).

        Args:
            position: Target position (encoder counts)
            velocity: Velocity (counts per servo tick)
            accel: Acceleration (counts per tick²)
        """
        traj_ctrl = 0x80 | 0x10  # start_now=1, servo_mode=1
        traj_data = struct.pack("<Biii", traj_ctrl, position, velocity, accel)
        self._device.send_command(CMD_LOAD_TRAJECTORY, list(traj_data))

        # Cache last commanded motion parameters
        self._state.position = position
        self._state.velocity = velocity
        self._state.pos_error = None  # will be updated on read
        setattr(self._state, "accel", accel)

    # -------------------------------------------------------------------------
    # Homing
    # -------------------------------------------------------------------------

    def set_home_mode(
        self,
        limit_switch: Optional[int] = None,
        use_index: bool = False,
        stop_mode: str = "abrupt",
    ) -> None:
        """
        Configure homing mode to capture home position.

        Args:
            limit_switch: Which limit switch to home to (1=reverse, 2=forward, None=no limit)
            use_index: If True, capture on encoder index pulse
            stop_mode: How to stop when home is found: 'abrupt', 'smooth', or 'off'

        Example:
            # Home to Limit 2 (forward), stop abruptly
            motion.set_home_mode(limit_switch=2, stop_mode='abrupt')

            # Home to index pulse, stop smoothly
            motion.set_home_mode(use_index=True, stop_mode='smooth')
        """
        control_byte = 0

        # Limit switch selection
        if limit_switch == 1:
            control_byte |= 0x01  # Bit 0: Limit 1 (reverse)
        elif limit_switch == 2:
            control_byte |= 0x02  # Bit 1: Limit 2 (forward)

        # Index pulse
        if use_index:
            control_byte |= 0x08  # Bit 3: Index

        # Stop mode (only one should be set)
        if stop_mode == "off":
            control_byte |= 0x04  # Bit 2: Turn motor off
        elif stop_mode == "abrupt":
            control_byte |= 0x10  # Bit 4: Stop abruptly
        elif stop_mode == "smooth":
            control_byte |= 0x20  # Bit 5: Stop smoothly
        else:
            raise ValueError(
                f"Invalid stop_mode: {stop_mode}. Must be 'abrupt', 'smooth', or 'off'"
            )

        self._device.send_command(CMD_SET_HOME_MODE, [control_byte])

    def home_to_limit(
        self,
        limit_switch: int,
        velocity: int,
        accel: int,
        use_index: bool = False,
        index_velocity: Optional[int] = None,
    ) -> None:
        """
        Perform complete homing sequence to a limit switch.

        This executes the homing procedure:
        1. Set home mode for limit switch
        2. Load velocity trajectory
        3. Start motion
        4. Wait for homing to complete
        5. Optionally: Fine-tune with index pulse

        Args:
            limit_switch: Which limit switch (1=reverse, 2=forward)
            velocity: Homing velocity in counts per servo tick
            accel: Homing acceleration in counts per tick²
            use_index: If True, perform second stage homing to index pulse
            index_velocity: Velocity for index homing (default: velocity/4)

        Example:
            # Home to Limit 2, then index pulse
            motion.home_to_limit(limit_switch=2, velocity=40000, accel=10000, use_index=True)
        """
        if limit_switch not in (1, 2):
            raise ValueError("limit_switch must be 1 (reverse) or 2 (forward)")

        # Stage 1: Home to limit switch
        print(f"Homing to Limit {limit_switch}...")

        # Set home mode
        self.set_home_mode(limit_switch=limit_switch, stop_mode="abrupt")

        # Load velocity trajectory
        # Direction: forward for Limit 2, reverse for Limit 1
        direction_bit = 0 if limit_switch == 2 else 0x40
        traj_ctrl = 0x36 | direction_bit  # Bits: 1,2,4,5 + optional direction bit
        traj_data = struct.pack("<Biii", traj_ctrl, 0, velocity, accel)
        self._device.send_command(CMD_LOAD_TRAJECTORY, list(traj_data))

        # Start motion
        self._device.send_command(CMD_START_MOTION, [])

        # Wait for homing to complete
        print("Waiting for limit switch...")
        while True:
            status = self._device.read_status()
            if not status.get("flags", {}).get("home_in_progress", False):
                break
            time.sleep(0.05)

        print(f"Reached Limit {limit_switch}")

        # Stage 2: Optional index pulse homing
        if use_index:
            if index_velocity is None:
                index_velocity = velocity // 4  # Use 25% of homing velocity

            print("Fine-tuning to index pulse...")

            # Set home mode for index
            self.set_home_mode(use_index=True, stop_mode="abrupt")

            # Load velocity trajectory in opposite direction
            reverse_direction_bit = 0x40 if limit_switch == 2 else 0
            traj_ctrl = 0x36 | reverse_direction_bit
            traj_data = struct.pack("<Biii", traj_ctrl, 0, index_velocity, accel)
            self._device.send_command(CMD_LOAD_TRAJECTORY, list(traj_data))

            # Start motion
            self._device.send_command(CMD_START_MOTION, [])

            # Wait for index capture
            print("Waiting for index pulse...")
            while True:
                status = self._device.read_status()
                if not status.get("flags", {}).get("home_in_progress", False):
                    break
                time.sleep(0.05)

            print("Homed to index pulse")

        print("Homing complete")

    # -------------------------------------------------------------------------
    # Amplifier Control
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """
        Enable amplifier (STOP_MOTOR with AMP_ENABLE flag).
        """
        from .servo import CMD_STOP_MOTOR

        stop_ctrl = STOP_ABRUPT | AMP_ENABLE
        self._device.send_command(CMD_STOP_MOTOR, [stop_ctrl])

    def disable(self) -> None:
        """
        Disable amplifier and position servo.
        """
        from .servo import CMD_STOP_MOTOR

        self._device.send_command(CMD_STOP_MOTOR, [0x00])

    def stop(self, smooth: bool = False) -> None:
        """
        Stop motor motion immediately without disabling amplifier.

        Args:
            smooth: If True, use smooth stop; if False, use abrupt stop (default)
        """
        from .servo import CMD_STOP_MOTOR

        if smooth:
            stop_ctrl = STOP_SMOOTH | AMP_ENABLE
        else:
            stop_ctrl = STOP_ABRUPT | AMP_ENABLE
        self._device.send_command(CMD_STOP_MOTOR, [stop_ctrl])

    # -------------------------------------------------------------------------
    # Position Management
    # -------------------------------------------------------------------------

    def reset_position(self, position: int = 0) -> None:
        """
        Reset current position counter (CMD 0x00).

        Args:
            position: New position value in encoder counts (default 0)

        Note:
            Does not move motor, only resets position counter.
            Use this to set a new coordinate system origin.
        """
        from .servo import CMD_RESET_POS

        # CMD_RESET_POS format: [pos_low, pos_mid, pos_high, pos_highest]
        pos_bytes = [
            position & 0xFF,
            (position >> 8) & 0xFF,
            (position >> 16) & 0xFF,
            (position >> 24) & 0xFF,
        ]
        self._device.send_command(CMD_RESET_POS, pos_bytes)

        # Update cached position
        self._state.position = position

    def save_home(self) -> None:
        """
        Save current position as home position (CMD 0x0C).

        Stores current position in non-volatile memory as the
        home reference point. This position can be recalled later
        for return-to-home operations.

        Note:
            This command saves to EEPROM. Avoid calling frequently
            as EEPROM has limited write cycles (~100,000).
        """
        from .servo import CMD_SAVE_AS_HOME

        self._device.send_command(CMD_SAVE_AS_HOME, [])

        # Cache the home position
        if self._state.position is not None:
            self._state.home_position = self._state.position

    # -------------------------------------------------------------------------
    # Path Mode (Coordinated Motion)
    # -------------------------------------------------------------------------

    def add_path_point(self, position: int, velocity: int, accel: int) -> None:
        """
        Add path point to motion buffer (CMD 0x0D).

        Args:
            position: Target position delta (encoder counts, int8.frac8 format)
            velocity: Velocity for this segment (counts per servo tick)
            accel: Acceleration for this segment (counts per tick²)

        The LS-231SE path buffer holds up to 256 points for coordinated motion.
        Points are executed in FIFO order when path mode is active.

        Path Point Format (int8.frac8):
            - Bits 15-8: Integer part (signed, -128 to 127)
            - Bits 7-0: Fractional part (unsigned, 0.0 to 0.99609375)
        """
        from .servo import CMD_ADD_PATHPOINT

        # Pack path point: position_delta (2 bytes), velocity (2 bytes), accel (2 bytes)
        point_data = struct.pack("<hhh", position, velocity, accel)
        self._device.send_command(CMD_ADD_PATHPOINT, list(point_data))

    def start_path_mode(self) -> None:
        """
        Start executing queued path points.

        The servo will execute path points in FIFO order from the internal buffer.
        Monitor path_count status to see remaining points.
        """
        # Use LOAD_TRAJECTORY with path mode flag
        traj_ctrl = 0x80 | 0x20  # start_now=1, path_mode=1
        traj_data = struct.pack(
            "<Biii", traj_ctrl, 0, 0, 0
        )  # Position/vel/accel unused in path mode
        self._device.send_command(CMD_LOAD_TRAJECTORY, list(traj_data))

    def clear_path_buffer(self) -> None:
        """
        Clear all queued path points.

        Stops path mode execution and empties the path buffer.
        """
        from .servo import CMD_STOP_MOTOR

        # Stop with path mode clear flag
        stop_ctrl = 0x80  # Clear path buffer
        self._device.send_command(CMD_STOP_MOTOR, [stop_ctrl])

    def get_path_count(self) -> int:
        """
        Get number of path points remaining in buffer.

        Returns:
            Number of points in path buffer (0-256)
        """
        return self._state.path_count if self._state.path_count is not None else 0


class Trajectory:
    """
    Path point trajectory/profile builder.

    The LS-231SE supports streaming motion via a 256-entry path point buffer.
    Each point uses int8.frac8 format (16-bit fixed-point position deltas).

    This class provides convenient methods to build common trajectory shapes
    (linear, circular, spline) and convert to path point format.
    """

    def __init__(self, buffer_size: int = 256):
        """
        Initialize trajectory builder.

        Args:
            buffer_size: Maximum path points (default 256)
        """
        self._buffer: List[float] = []
        self._timing: int = 100  # Path point timing (5.12ms default)
        self._buffer_size = buffer_size

    # -------------------------------------------------------------------------
    # Trajectory Building
    # -------------------------------------------------------------------------

    def add_linear(self, start: float, end: float, points: int) -> "Trajectory":
        """
        Add linear trajectory segment.

        Args:
            start: Starting position (encoder counts)
            end: Ending position (encoder counts)
            points: Number of path points

        Returns:
            Self for method chaining

        Example:
            traj = Trajectory().add_linear(0, 10000, 50)
        """
        if len(self._buffer) + points > self._buffer_size:
            raise ValueError(f"Trajectory exceeds buffer size ({self._buffer_size})")

        delta = (end - start) / points
        for i in range(points):
            self._buffer.append(start + delta * i)

        return self

    def add_circular(
        self,
        center_x: float,
        center_y: float,
        radius: float,
        start_angle: float,
        end_angle: float,
        points: int,
    ) -> "Trajectory":
        """
        Add circular trajectory segment (2D interpolation).

        Args:
            center_x: Circle center X coordinate
            center_y: Circle center Y coordinate
            radius: Circle radius
            start_angle: Starting angle (radians)
            end_angle: Ending angle (radians)
            points: Number of path points

        Returns:
            Self for method chaining

        Note:
            This generates X-Y coordinates. For single-axis servos, only
            one coordinate will be used. For dual-axis systems, call this
            twice (once per axis) or use coordinated motion.
        """
        if len(self._buffer) + points > self._buffer_size:
            raise ValueError(f"Trajectory exceeds buffer size ({self._buffer_size})")

        import math

        angle_delta = (end_angle - start_angle) / points

        for i in range(points):
            angle = start_angle + angle_delta * i
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            # For now, just store X (single axis)
            # Multi-axis coordination would be handled by caller
            self._buffer.append(x)

        return self

    def clear(self) -> "Trajectory":
        """
        Clear trajectory buffer.

        Returns:
            Self for method chaining
        """
        self._buffer.clear()
        return self

    # -------------------------------------------------------------------------
    # Path Point Conversion
    # -------------------------------------------------------------------------

    def to_path_points(self) -> List[int]:
        """
        Convert trajectory to int8.frac8 format path points.

        Returns:
            List of 16-bit path point deltas

        The LS-231SE path point format is int8.frac8:
        - Bits 15-8: Integer part (signed)
        - Bits 7-0: Fractional part (unsigned)

        Example:
            Position delta of 10.5 counts = 0x0A80
            (10 << 8) | int(0.5 * 256) = 2560 + 128 = 2688 = 0x0A80
        """
        if not self._buffer:
            return []

        path_points = []

        # Calculate deltas between consecutive positions
        for i in range(1, len(self._buffer)):
            delta = self._buffer[i] - self._buffer[i - 1]

            # Split into integer and fractional parts
            int_part = int(delta)
            frac_part = delta - int_part

            # Convert to int8.frac8 format
            # Integer part: signed 8-bit (-128 to 127)
            # Fractional part: unsigned 8-bit (0 to 255)
            if int_part < -128 or int_part > 127:
                raise ValueError(f"Path point delta {delta} out of range (-128 to 127)")

            frac_byte = int(abs(frac_part) * 256) & 0xFF

            # Pack as 16-bit value
            if int_part >= 0:
                point = (int_part << 8) | frac_byte
            else:
                # Two's complement for negative values
                point = ((int_part & 0xFF) << 8) | frac_byte

            path_points.append(point)

        return path_points

    def get_timing(self) -> int:
        """Get path point timing value."""
        return self._timing

    def set_timing(self, timing: int) -> "Trajectory":
        """
        Set path point timing.

        Args:
            timing: Timing value (units depend on servo configuration)

        Returns:
            Self for method chaining
        """
        self._timing = timing
        return self

    def __len__(self) -> int:
        """Return number of points in trajectory."""
        return len(self._buffer)
