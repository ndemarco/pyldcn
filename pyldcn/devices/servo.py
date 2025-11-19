"""
LS-231SE Servo Drive Device

Servo-specific operations including motion control, status reading,
and PID gain configuration.

Author: NickyDoes
License: GPL v2 or later
"""

import time
import struct
from typing import Optional, Dict, List, Any

# Import from parent package (modular architecture)
from pyldcn.device import LDCNDevice
from pyldcn.network import LDCNNetwork

# Import subsystems
from .servo_state import ServoState
from .status import ServoStatus
from .servo_motion import Motion, Trajectory
from .servo_io import IO
from .servo_mappings import LS231SE_STATUS_PROFILE


# =============================================================================
# LS-231SE Servo Drive Commands (Motor/Servo Specific)
# =============================================================================

CMD_RESET_POS = 0x00        # Reset position counter
CMD_LOAD_TRAJECTORY = 0x04  # Load trajectory parameters
CMD_START_MOTION = 0x05     # Start motion
CMD_LOAD_GAINS = 0x06       # Set PID gains
CMD_STOP_MOTOR = 0x07       # Stop motor
CMD_IO_CTRL = 0x08          # I/O control
CMD_SET_HOME_MODE = 0x09    # Configure homing mode
CMD_CLEAR_BITS = 0x0B       # Clear sticky status bits
CMD_SAVE_AS_HOME = 0x0C     # Save current position as home
CMD_ADD_PATHPOINT = 0x0D    # Add path point to buffer
CMD_EXT = 0x0E              # Extended commands (with subcommand byte)


class LS231SE(LDCNDevice):
    """
    LS-231SE Servo Drive

    Implements servo-specific operations including motion control, status
    reading, and PID gain configuration.

    Architecture:
        This class uses a functional subsystem architecture with three
        subsystems sharing a single state object:
        - ServoStatus: Status reading, parsing, diagnostic detection
        - Motion: Position control, homing, amplifier control
        - IO: Brake control, limit/home switch monitoring

    Accessing State:
        Common properties are exposed directly on this class:
            servo.position, servo.velocity, servo.servo_on, etc.

        Bulk state access available via:
            servo.state (ServoState object)

    Using Subsystems:
        Most operations are delegated to subsystems but exposed via
        convenience methods on this class for clean API:
            servo.move_to() -> servo._motion.move_to()
            servo.read_status() -> servo._status.read_status()

    Additional Attributes:
        state: Shared ServoState object containing all servo state

        Status byte flags:
            move_done, cksum_error, current_limit, power,
            pos_error_flag, home_source, limit2, home_in_progress

        Auxiliary status byte flags:
            index, pos_wrap, servo_on, accel_done, slew_done,
            servo_overrun, path_mode

        Control signals:
            stop_cmd, pic_ae

        PID gains:
            kp, kd, ki
    """

    def __init__(self, network: LDCNNetwork, address: int):
        """
        Initialize servo drive.

        Args:
            network: Parent LDCNNetwork object
            address: Device address (1-127)

        Default property values reflect the "No Motor Power after LDCN Init"
        condition from the diagnostic table (power-up/reset state).
        """
        super().__init__(network, address)
        self.device_type = "LS-231SE"

        # Shared state - single source of truth
        self.state = ServoState()
        self.state.current_mode = LS231SE_STATUS_PROFILE.default_mode

        # Subsystems share state reference
        self._status = ServoStatus(self.state, self)
        self._motion = Motion(self.state, self)
        self._io = IO(self.state, self)

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def initialize(self,
                   kp: int = 2, kd: int = 50, ki: int = 0,
                   il: int = 40, ol: int = 255, cl: int = 0,
                   el: int = 2000, sr: int = 20, db: int = 0) -> bool:
        """
        Complete 7-step servo initialization.

        Steps:
        1. Define status reporting (pos, vel, aux, pos_err)
        2. Set PID gains
        3. Load initial trajectory (position 0)
        4. Enable amplifier
        5. Reset position counter
        6. Clear sticky status bits
        7. Read and verify status

        Args:
            kp, kd, ki: PID gains
            il: Integration limit
            ol: Output limit
            cl: Current limit (0 = disabled)
            el: Position error limit (encoder counts)
            sr: Servo rate divisor
            db: Deadband

        Returns:
            True if initialization successful
        """
        try:
            # Step 1: Define status reporting
            # Request: position, velocity, aux, pos_error
            status_bits = 0x0001 | 0x0004 | 0x0008 | 0x0040
            self.define_status(status_bits)
            time.sleep(0.1)

            # Step 2: Set PID gains
            self.set_gains(kp, kd, ki, il, ol, cl, el, sr, db)
            time.sleep(0.1)

            # Step 3: Load initial trajectory (position 0)
            traj_ctrl = 0x10  # servo_mode=1
            traj_data = [traj_ctrl, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0]  # pos=0, vel=0, accel=100
            self.send_command(CMD_LOAD_TRAJECTORY, traj_data)
            time.sleep(0.1)

            # Step 4: Enable amplifier
            self.enable()
            time.sleep(0.1)

            # Step 5: Reset position counter
            self.reset_position()
            time.sleep(0.1)

            # Step 6: Clear sticky status bits
            self.clear_faults()
            time.sleep(0.1)

            # Step 7: Read and verify status
            status = self.read_status()

            # Check for faults
            faults = self._status.check_faults(status['status'])
            if faults:
                return False

            return True

        except Exception:
            return False

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------

    def load_gains(self,
                   kp: int,
                   kd: int,
                   ki: int,
                   il: int = 255,
                   ol: int = 255,
                   cl: int = 255,
                   el: int = 16000,
                   sr: int = 1,
                   db: int = 0) -> None:
        """
        Load PID gains with sensible defaults (LOAD_GAINS command).

        Simplified interface to set_gains() with default values for testing
        and basic operation.

        Args:
            kp: Proportional gain
            kd: Derivative gain
            ki: Integral gain
            il: Integration limit (default: 255 = maximum)
            ol: Output limit (default: 255 = maximum)
            cl: Current limit (default: 255 = maximum)
            el: Position error limit (default: 16000)
            sr: Servo rate divisor (default: 1 = fastest)
            db: Deadband (default: 0 = no deadband)
        """
        self.set_gains(kp, kd, ki, il, ol, cl, el, sr, db)

    def set_gains(self,
                  kp: int, kd: int, ki: int,
                  il: int, ol: int, cl: int,
                  el: int, sr: int, db: int) -> None:
        """
        Set PID gains (LOAD_GAINS command).

        Low-level interface requiring all parameters.

        Args:
            kp: Proportional gain
            kd: Derivative gain
            ki: Integral gain
            il: Integration limit
            ol: Output limit
            cl: Current limit
            el: Position error limit
            sr: Servo rate divisor
            db: Deadband
        """
        gain_data = struct.pack('<HHHBBBHB', kp, kd, ki, il, ol, cl, el, sr)
        self.send_command(CMD_LOAD_GAINS, list(gain_data) + [db])

        # Cache gains in shared state
        self.state.kp = kp
        self.state.kd = kd
        self.state.ki = ki

    def configure_status(self, status_bits: int) -> None:
        """
        Configure status reporting.

        Args:
            status_bits: 16-bit status configuration
        """
        self.define_status(status_bits)

    # -------------------------------------------------------------------------
    # Fault Management
    # -------------------------------------------------------------------------

    def clear_faults(self) -> None:
        """
        Clear sticky status bits (CLEAR_BITS command).
        """
        self.send_command(CMD_CLEAR_BITS)

    # -------------------------------------------------------------------------
    # Delegated Methods - Status Subsystem
    # -------------------------------------------------------------------------

    def read_status(self) -> Dict:
        """
        Read complete servo status.

        Delegates to Status subsystem.

        Returns:
            {
                'status': status_byte,
                'position': position_counts,
                'velocity': velocity,
                'aux_status': aux_byte,
                'pos_error': following_error,
                'flags': {...}
            }
        """
        return self._status.read_status()

    def read_position(self) -> Dict:
        """
        Read position only (fast status read).

        Delegates to Status subsystem.

        Returns:
            {'position': position_counts, 'status': status_byte}
        """
        return self._status.read_position()

    def decode_status_flags(self, status_byte: int) -> Dict[str, bool]:
        """
        Decode status byte into flag dictionary.

        Delegates to Status subsystem.

        Args:
            status_byte: Status byte from response

        Returns:
            Dictionary of flag names to boolean values
        """
        return self._status.decode_status_flags(status_byte)

    # -------------------------------------------------------------------------
    # Mode Management
    # -------------------------------------------------------------------------

    def set_mode(self, mode_name: str) -> None:
        """
        Record the current servo operating mode.

        Note: Setting the software state does not automatically send the
        hardware command to switch modes. Callers must issue the appropriate
        device command separately (e.g., via the LDCN mode change sequence),
        then invoke this helper so status/IO decoding reflects the new mode.
        """
        if mode_name not in self._status.profile.mode_lookup:
            valid = ", ".join(self._status.profile.mode_lookup.keys())
            raise ValueError(f"Unknown mode '{mode_name}'. Valid modes: {valid}")
        self.state.current_mode = mode_name

    def get_mode(self) -> str:
        """Return the cached operating mode key."""
        return self.state.current_mode

    def get_mapped_signals(self) -> Dict[str, str]:
        """
        Return physical signal names for status bits 5/6 based on HomeSEL.

        Status bits 5 and 6 are dynamically mapped to different physical inputs
        depending on the HomeSEL output bit configuration. This method returns
        the current mapping.

        Returns:
            Dictionary with keys:
                - 'home_source_signal': Physical signal mapped to status bit 5
                - 'limit2_signal': Physical signal mapped to status bit 6

        Example:
            >>> servo.io.set_home_selection_state(homesel2=0, homesel1=1)
            >>> signals = servo.get_mapped_signals()
            >>> print(signals)
            {'home_source_signal': 'HomeIN', 'limit2_signal': 'Input10'}
        """
        from .servo_mappings import LS231SE_HOME_SOURCE_TABLE

        lookup = {(e.homesel2, e.homesel1): e for e in LS231SE_HOME_SOURCE_TABLE}
        entry = lookup.get(self.state.home_selection)

        return {
            'home_source_signal': entry.status_bit5 if entry else 'Unknown',
            'limit2_signal': entry.status_bit6 if entry else 'Unknown'
        }

    def get_state(self) -> Dict[str, Any]:
        """
        Get comprehensive servo state with full context.

        Reads current status and applies HomeSEL mapping, mode, and diagnostic
        context automatically. This is the recommended high-level method for
        status monitoring.

        Returns:
            Dictionary containing:
                - 'status_byte': Raw status byte value
                - 'aux_byte': Raw auxiliary status byte value
                - 'flags': Decoded flag dictionary with physical signal names
                - 'condition': Diagnostic condition (brake, LED, relay states)
                - 'position': Current position (if configured)
                - 'velocity': Current velocity (if configured)
                - 'pos_error': Position error (if configured)

        Example:
            >>> state = servo.get_state()
            >>> print(f"Servo on: {state['flags']['servo_on']}")
            >>> print(f"Brake: {state['condition'].brake_state}")
        """
        from . import servo_diagnostics as diag

        status = self.read_status()
        status_byte = status.get("status", 0)
        aux_byte = status.get("aux_status", self.state.aux_status or 0)

        # Get diagnostic state with full context
        state = diag.get_servo_state(
            status_byte,
            aux_byte,
            stop_cmd=self.state.stop_cmd,
            pic_ae=self.state.pic_ae,
            mode=self.state.current_mode,
        )

        # Merge motion data from status
        if "position" in status:
            state["position"] = status["position"]
        if "velocity" in status:
            state["velocity"] = status["velocity"]
        if "pos_error" in status:
            state["pos_error"] = status["pos_error"]

        return state

    def check_faults(self, status_byte: int) -> List[str]:
        """
        Check status byte for fault conditions.

        Delegates to Status subsystem.

        Args:
            status_byte: Status byte to check

        Returns:
            List of active fault names
        """
        return self._status.check_faults(status_byte)

    # -------------------------------------------------------------------------
    # Delegated Methods - Motion Subsystem
    # -------------------------------------------------------------------------

    def move_to(self, position: float, velocity: float, accel: float, scale: float = 1.0) -> None:
        """
        Command motion to absolute position.

        Delegates to Motion subsystem.

        Args:
            position: Target position (physical units, e.g., mm)
            velocity: Velocity (physical units/sec)
            accel: Acceleration (physical units/sec²)
            scale: Counts per physical unit (e.g., 2000 counts/mm)
        """
        self._motion.move_to(position, velocity, accel, scale)

    def move_to_counts(self, position: int, velocity: int, accel: int) -> None:
        """
        Command motion to absolute position (raw counts).

        Delegates to Motion subsystem.

        Args:
            position: Target position (encoder counts)
            velocity: Velocity (counts per servo tick)
            accel: Acceleration (counts per tick²)
        """
        self._motion.move_to_counts(position, velocity, accel)

    def set_home_mode(self, limit_switch: Optional[int] = None,
                      use_index: bool = False,
                      stop_mode: str = 'abrupt') -> None:
        """
        Configure homing mode to capture home position.

        Delegates to Motion subsystem.

        Args:
            limit_switch: Which limit switch to home to (1=reverse, 2=forward, None=no limit)
            use_index: If True, capture on encoder index pulse
            stop_mode: How to stop when home is found: 'abrupt', 'smooth', or 'off'
        """
        self._motion.set_home_mode(limit_switch, use_index, stop_mode)

    def home_to_limit(self, limit_switch: int, velocity: int, accel: int,
                      use_index: bool = False, index_velocity: Optional[int] = None) -> None:
        """
        Perform complete homing sequence to a limit switch.

        Delegates to Motion subsystem.

        Args:
            limit_switch: Which limit switch (1=reverse, 2=forward)
            velocity: Homing velocity in counts per servo tick
            accel: Homing acceleration in counts per tick²
            use_index: If True, perform second stage homing to index pulse
            index_velocity: Velocity for index homing (default: velocity/4)
        """
        self._motion.home_to_limit(limit_switch, velocity, accel, use_index, index_velocity)

    def enable(self) -> None:
        """
        Enable amplifier.

        Delegates to Motion subsystem.
        """
        self._motion.enable()

    def disable(self) -> None:
        """
        Disable amplifier and position servo.

        Delegates to Motion subsystem.
        """
        self._motion.disable()

    def stop(self) -> None:
        """
        Stop motor motion immediately.

        Delegates to Motion subsystem.
        """
        self._motion.stop()

    # -------------------------------------------------------------------------
    # Diagnostic Convenience Methods
    # -------------------------------------------------------------------------

    def is_ready(self, mode: str = "LDCN") -> bool:
        """
        Check if servo is ready for motion commands.

        Args:
            mode: "LDCN" or "Amplifier" (default: "LDCN")

        Returns:
            True if operational (ServoON, ServoOFF, AxisOFF), False if faulted
        """
        return self._status.is_operational(mode)

    def get_condition(self, mode: str = "LDCN"):
        """
        Get current diagnostic condition.

        Args:
            mode: "LDCN" or "Amplifier" (default: "LDCN")

        Returns:
            DiagnosticCondition object or None
        """
        return self._status.get_condition(mode)

    def print_status(self, include_leds: bool = False, mode: str = "LDCN") -> None:
        """
        Print formatted status report to console.

        Args:
            include_leds: If True, include LED states in output
            mode: "LDCN" or "Amplifier" (default: "LDCN")
        """
        print(self._status.format_status(include_leds, mode))

    def is_faulted(self, mode: str = "LDCN") -> bool:
        """
        Check if servo is in faulted state.

        Args:
            mode: "LDCN" or "Amplifier" (default: "LDCN")

        Returns:
            True if faulted, False otherwise
        """
        return self._status.is_faulted(mode)

    # -------------------------------------------------------------------------
    # Property Access - Subsystems
    # -------------------------------------------------------------------------

    @property
    def status(self):
        """Access to Status subsystem for advanced operations."""
        return self._status

    @property
    def motion(self):
        """Access to Motion subsystem for advanced operations."""
        return self._motion

    @property
    def io(self):
        """Access to IO subsystem for advanced operations."""
        return self._io

    # -------------------------------------------------------------------------
    # Property Access - Common State
    # -------------------------------------------------------------------------

    @property
    def position(self) -> Optional[int]:
        """Current position in encoder counts."""
        return self.state.position

    @property
    def velocity(self) -> Optional[int]:
        """Current velocity in counts per servo tick."""
        return self.state.velocity

    @property
    def pos_error(self) -> Optional[int]:
        """Position following error in encoder counts."""
        return self.state.pos_error

    @property
    def servo_on(self) -> bool:
        """Position servo loop enabled."""
        return self.state.servo_on

    @property
    def move_done(self) -> bool:
        """Trapezoidal move complete or velocity stable."""
        return self.state.move_done

    @property
    def current_limit(self) -> bool:
        """Current limiting exceeded (sticky)."""
        return self.state.current_limit

    @property
    def power(self) -> bool:
        """Amplifier power enabled."""
        return self.state.power

    @property
    def home_in_progress(self) -> bool:
        """Searching for home position."""
        return self.state.home_in_progress

    @property
    def kp(self) -> Optional[int]:
        """Proportional gain."""
        return self.state.kp

    @property
    def kd(self) -> Optional[int]:
        """Derivative gain."""
        return self.state.kd

    @property
    def ki(self) -> Optional[int]:
        """Integral gain."""
        return self.state.ki


# Export Trajectory class for external use
__all__ = ['LS231SE', 'Trajectory']
