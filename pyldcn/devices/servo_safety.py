"""
LS-231SE Servo Drive Safety Subsystem

Safety and synchronization features including watchdog timer,
hardware synchronization, and limit stop behavior.

Author: NickyDoes
License: GPL v2 or later
"""

from typing import TYPE_CHECKING

from .servo_state import ServoState

if TYPE_CHECKING:
    from .servo import LS231SE


# Extended command subcodes
EXT_LIMIT_STOP = 0x00      # Configure limit switch stop behavior
EXT_HW_SYNC = 0x04         # Configure hardware synchronization
EXT_WATCHDOG = 0x05        # Configure watchdog timer


class Safety:
    """
    Safety and synchronization operations.

    This subsystem handles:
    - Watchdog timer configuration and reset
    - Hardware synchronization between multiple drives
    - Limit switch stop behavior configuration
    """

    def __init__(self, state: ServoState, device: 'LS231SE'):
        """
        Initialize Safety subsystem.

        Args:
            state: Shared ServoState object
            device: Parent LS231SE device instance
        """
        self._state = state
        self._device = device

    # -------------------------------------------------------------------------
    # Watchdog Timer
    # -------------------------------------------------------------------------

    def set_watchdog(self, timeout_ms: int) -> None:
        """
        Configure watchdog timer (CMD 0x0E, subcommand 0x05).

        The watchdog timer monitors communication health. If no command
        is received within the timeout period, the servo will automatically
        stop and enter a fault state.

        Args:
            timeout_ms: Timeout in milliseconds (0 to disable, max ~65535)

        Note:
            If watchdog is enabled, you must periodically send commands
            (or call reset_watchdog()) to keep the servo operational.
            Typical timeout values: 100-1000ms.

        Example:
            # Enable 500ms watchdog
            safety.set_watchdog(500)

            # Disable watchdog
            safety.set_watchdog(0)
        """
        from .servo import CMD_EXT

        if timeout_ms < 0 or timeout_ms > 65535:
            raise ValueError(f"Watchdog timeout {timeout_ms}ms out of range (0-65535)")

        # Extended command format: [subcommand, data_low, data_high]
        timeout_low = timeout_ms & 0xFF
        timeout_high = (timeout_ms >> 8) & 0xFF

        self._device.send_command(CMD_EXT, [EXT_WATCHDOG, timeout_low, timeout_high])

    def reset_watchdog(self) -> None:
        """
        Reset watchdog timer (keep-alive ping).

        Sends a command to reset the watchdog timer. Use this in long-running
        operations where you're not sending motion commands frequently enough
        to satisfy the watchdog timeout.

        Note:
            Any command sent to the servo will reset the watchdog.
            This is a convenience method using NOP command.
        """
        from pyldcn.protocol import CMD_NOP  # NOP is generic LDCN

        # Send NOP command to reset watchdog
        self._device.send_command(CMD_NOP, [])

    # -------------------------------------------------------------------------
    # Hardware Synchronization
    # -------------------------------------------------------------------------

    def configure_hw_sync(self, enable: bool, sync_mode: str = "start") -> None:
        """
        Configure hardware synchronization (CMD 0x0E, subcommand 0x04).

        Hardware sync allows multiple servo drives to coordinate motion
        via dedicated hardware sync signals (typically connected between drives).

        Args:
            enable: True to enable hardware sync, False to disable
            sync_mode: Synchronization mode:
                      "start" - Sync on motion start
                      "trigger" - Sync on external trigger
                      "master" - Act as sync master
                      "slave" - Act as sync slave

        Example:
            # Enable synchronized start (wait for sync signal before moving)
            safety.configure_hw_sync(True, "start")

            # Disable sync
            safety.configure_hw_sync(False)

        Note:
            Hardware connections required between drives for sync operation.
            See LS-231SE datasheet for sync pin connections.
        """
        from .servo import CMD_EXT

        # Encode sync mode
        mode_map = {
            "start": 0x01,
            "trigger": 0x02,
            "master": 0x04,
            "slave": 0x08
        }

        if sync_mode not in mode_map:
            raise ValueError(f"Invalid sync_mode: {sync_mode}. Must be 'start', 'trigger', 'master', or 'slave'")

        sync_byte = mode_map[sync_mode] if enable else 0x00

        self._device.send_command(CMD_EXT, [EXT_HW_SYNC, sync_byte, 0x00])

    # -------------------------------------------------------------------------
    # Limit Stop Behavior
    # -------------------------------------------------------------------------

    def configure_limit_stop(
        self,
        action: str = "decelerate",
        auto_recover: bool = False
    ) -> None:
        """
        Configure limit switch stop behavior (CMD 0x0E, subcommand 0x00).

        Controls how the servo responds when a limit switch is triggered.

        Args:
            action: Stop action when limit hit:
                   "decelerate" - Controlled deceleration (default, safe)
                   "immediate" - Immediate hard stop (fastest)
                   "disable" - Disable servo and amplifier
            auto_recover: If True, automatically clear limit fault and
                         allow motion away from limit

        Example:
            # Decelerate smoothly when hitting limit
            safety.configure_limit_stop("decelerate", auto_recover=True)

            # Emergency hard stop on limit (for critical safety)
            safety.configure_limit_stop("immediate", auto_recover=False)

        Note:
            - "decelerate" uses configured deceleration rate (gentler on mechanics)
            - "immediate" stops instantly (may cause mechanical shock)
            - "disable" turns off servo (motor will coast/brake depending on config)
        """
        from .servo import CMD_EXT

        # Encode action
        action_map = {
            "decelerate": 0x00,  # Default: controlled deceleration
            "immediate": 0x01,   # Hard stop
            "disable": 0x02      # Disable servo
        }

        if action not in action_map:
            raise ValueError(f"Invalid action: {action}. Must be 'decelerate', 'immediate', or 'disable'")

        action_byte = action_map[action]

        # Add auto-recover flag
        if auto_recover:
            action_byte |= 0x80  # Bit 7: Auto-recover enable

        self._device.send_command(CMD_EXT, [EXT_LIMIT_STOP, action_byte, 0x00])

    # -------------------------------------------------------------------------
    # Emergency Operations
    # -------------------------------------------------------------------------

    def emergency_stop(self) -> None:
        """
        Emergency stop - immediately halt motion and disable servo.

        This is the safest stop method but will cause mechanical shock.
        Use for emergency conditions only.

        Note:
            After emergency stop, you must clear faults and re-enable
            the servo before resuming normal operation.
        """
        from .servo import CMD_STOP_MOTOR

        # Stop immediately without deceleration
        self._device.send_command(CMD_STOP_MOTOR, [0x01])  # Immediate stop flag

    def reset_faults(self) -> None:
        """
        Reset all fault conditions (alias for clear_faults).

        Clears sticky fault bits including:
        - Current limit exceeded
        - Position error
        - Servo overrun
        - Position wraparound

        Note:
            Some faults (like encoder errors) require a hard reset
            and cannot be cleared by this command.
        """
        from .servo import CMD_CLEAR_BITS

        self._device.send_command(CMD_CLEAR_BITS, [])
