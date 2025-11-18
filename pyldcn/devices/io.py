"""
IOController Base Class

Abstract base class for LDCN I/O controllers (LS-773, SK-2310g2).

Provides common functionality for I/O devices:
- Digital output control with state tracking
- PWM output control
- Synchronized output operations
- Counter/timer operations
- Common bit manipulation utilities

Author: NickyDoes
License: GPL v2 or later
"""

from abc import ABC, abstractmethod
from typing import List, Optional
from pyldcn.device import LDCNDevice
from pyldcn.network import LDCNNetwork


# =============================================================================
# Common I/O Controller Commands
# =============================================================================

CMD_SET_PWM_IO = 0x04  # Set PWM duty cycle
CMD_SYNCH_OUTPUT = 0x05  # Apply staged outputs
CMD_SET_OUTPUTS = 0x06  # Set all output states
CMD_SET_SYNCH_OUTPUT = 0x07  # Stage outputs for sync
CMD_SET_TIMER_MODE = 0x08  # Configure counter/timer
CMD_SYNCH_INPUT = 0x0C  # Capture inputs atomically


class IOController(LDCNDevice, ABC):
    """
    Abstract base class for LDCN I/O controllers.

    Provides common functionality for I/O devices like LS-773 and SK-2310g2:
    - Digital output control with state persistence
    - PWM output control
    - Synchronized output operations
    - Counter/timer operations
    - Bit manipulation utilities

    Device-specific implementations must provide:
    - PWM channel configuration
    - Output validation logic
    """

    DEVICE_ID = 0x02  # Common to all I/O controllers

    def __init__(self, network: LDCNNetwork, address: int):
        """
        Initialize I/O controller.

        Args:
            network: Parent LDCNNetwork object
            address: Device address (1-127)
        """
        super().__init__(network, address)

        # Track output state (devices don't report this back)
        self._output_byte0: int = 0x00
        self._output_byte1: int = 0x00

    # -------------------------------------------------------------------------
    # Abstract Methods - Device-specific behavior
    # -------------------------------------------------------------------------

    @abstractmethod
    def get_pwm_channels(self) -> List[int]:
        """
        Return list of available PWM channel numbers.

        Returns:
            List of PWM channel numbers supported by this device
        """
        pass

    @abstractmethod
    def _validate_outputs(self, byte0: int, byte1: int) -> None:
        """
        Validate output bytes before sending to device.

        Device-specific validation (e.g., safety constraints).

        Args:
            byte0: Output byte 0 (bits 0-7)
            byte1: Output byte 1 (bits 8-15)

        Raises:
            ValueError: If output combination is invalid
            LDCNError: If safety constraint is violated
        """
        pass

    # -------------------------------------------------------------------------
    # Digital Output Control
    # -------------------------------------------------------------------------

    def set_outputs(self, byte0: int = 0x00, byte1: int = 0x00) -> None:
        """
        Set all digital outputs immediately.

        Args:
            byte0: Output byte 0 (bits 0-7)
            byte1: Output byte 1 (bits 8-15)

        Raises:
            ValueError: If validation fails
        """
        # Validate device-specific constraints
        self._validate_outputs(byte0, byte1)

        # Send command
        self.send_command(CMD_SET_OUTPUTS, bytes([byte0, byte1]))

        # Persist state (device doesn't report output state)
        self._output_byte0 = byte0
        self._output_byte1 = byte1

    def get_output_state(self) -> tuple[int, int]:
        """
        Return cached output state.

        Returns:
            Tuple of (byte0, byte1)
        """
        return (self._output_byte0, self._output_byte1)

    def set_output_bit(self, bit: int, state: bool) -> None:
        """
        Set a single output bit while preserving others.

        Args:
            bit: Output bit number (0-15)
            state: True to set bit, False to clear bit

        Raises:
            ValueError: If bit out of range or validation fails
        """
        if not 0 <= bit <= 15:
            raise ValueError(f"Bit {bit} out of range. Must be 0-15")

        # Calculate new output state
        if bit < 8:
            new_byte0 = self._set_bit(self._output_byte0, bit, state)
            new_byte1 = self._output_byte1
        else:
            new_byte0 = self._output_byte0
            new_byte1 = self._set_bit(self._output_byte1, bit - 8, state)

        # Set outputs (includes validation)
        self.set_outputs(new_byte0, new_byte1)

    # -------------------------------------------------------------------------
    # PWM Output Control
    # -------------------------------------------------------------------------

    def set_pwm(self, channel: int, duty_cycle: int) -> None:
        """
        Set PWM duty cycle for a channel.

        Args:
            channel: PWM channel number
            duty_cycle: 0-255 (0% to 100%)

        Raises:
            ValueError: If invalid channel or duty cycle
        """
        if channel not in self.get_pwm_channels():
            raise ValueError(
                f"Invalid PWM channel {channel}. "
                f"Supported channels: {self.get_pwm_channels()}"
            )

        if not 0 <= duty_cycle <= 255:
            raise ValueError(f"Duty cycle {duty_cycle} out of range. Must be 0-255")

        self.send_command(CMD_SET_PWM_IO, bytes([channel, duty_cycle]))

    # -------------------------------------------------------------------------
    # Synchronized Output Operations
    # -------------------------------------------------------------------------

    def sync_output(self, time_ms: float) -> None:
        """
        Synchronize output changes across network.

        All devices that have received SET_SYNCH_OUTPUT commands will
        apply their staged outputs at this synchronized time.

        Args:
            time_ms: Synchronization time in milliseconds
        """
        # Convert milliseconds to 5MHz clock ticks
        time_5mhz = int((time_ms / 1000.0) * 5_000_000)
        time_bytes = time_5mhz.to_bytes(4, byteorder='little')
        self.send_command(CMD_SYNCH_OUTPUT, time_bytes)

    def set_sync_output(self, byte0: int, byte1: int, time_ms: float) -> None:
        """
        Set outputs to be applied at synchronized time.

        Stages output changes to be applied when sync_output() is called.

        Args:
            byte0: Output byte 0 (bits 0-7)
            byte1: Output byte 1 (bits 8-15)
            time_ms: Synchronization time in milliseconds

        Raises:
            ValueError: If validation fails
        """
        # Validate device-specific constraints
        self._validate_outputs(byte0, byte1)

        # Convert time to 5MHz clock ticks
        time_5mhz = int((time_ms / 1000.0) * 5_000_000)
        time_bytes = time_5mhz.to_bytes(4, byteorder='little')

        # Send staged output command
        data = bytes([byte0, byte1]) + time_bytes
        self.send_command(CMD_SET_SYNCH_OUTPUT, data)

        # Update cached state (will be applied at sync time)
        self._output_byte0 = byte0
        self._output_byte1 = byte1

    # -------------------------------------------------------------------------
    # Counter/Timer Operations
    # -------------------------------------------------------------------------

    def set_timer_mode(self, mode: int) -> None:
        """
        Set counter/timer mode.

        Args:
            mode: Timer mode value (device-specific)

        Raises:
            ValueError: If mode out of range
        """
        if not 0 <= mode <= 255:
            raise ValueError(f"Timer mode {mode} out of range. Must be 0-255")

        self.send_command(CMD_SET_TIMER_MODE, bytes([mode]))

    def sync_input(self, time_ms: float) -> None:
        """
        Capture input state and counter at synchronized time.

        Atomically captures digital inputs and counter/timer value.
        Use status read with sync_inputs/sync_counter bits to retrieve.

        Args:
            time_ms: Synchronization time in milliseconds
        """
        # Convert time to 5MHz clock ticks
        time_5mhz = int((time_ms / 1000.0) * 5_000_000)
        time_bytes = time_5mhz.to_bytes(4, byteorder='little')
        self.send_command(CMD_SYNCH_INPUT, time_bytes)

    # -------------------------------------------------------------------------
    # Bit Manipulation Utilities
    # -------------------------------------------------------------------------

    @staticmethod
    def _get_bit(value: int, bit: int) -> bool:
        """
        Extract a single bit from an integer value.

        Args:
            value: Integer value
            bit: Bit position (0-7 for byte operations)

        Returns:
            True if bit is set, False otherwise
        """
        return bool((value >> bit) & 0x01)

    @staticmethod
    def _set_bit(value: int, bit: int, state: bool) -> int:
        """
        Set a single bit in an integer value.

        Args:
            value: Current integer value
            bit: Bit position (0-7 for byte operations)
            state: True to set bit, False to clear bit

        Returns:
            Modified integer value
        """
        if state:
            return value | (1 << bit)
        else:
            return value & ~(1 << bit)
