"""
LS-231SE Servo Drive I/O Subsystem

I/O control operations (brake, limits, switches).

Author: NickyDoes
License: GPL v2 or later
"""

from typing import TYPE_CHECKING, Dict

from .servo_state import ServoState

if TYPE_CHECKING:
    from .servo import LS231SE


class IO:
    """
    I/O control operations.

    This subsystem handles:
    - Brake control (OUTbit0)
    - Limit switch monitoring
    - Home switch monitoring
    - General digital I/O
    """

    def __init__(self, state: ServoState, device: 'LS231SE'):
        """
        Initialize IO subsystem.

        Args:
            state: Shared ServoState object
            device: Parent LS231SE device instance
        """
        self._state = state
        self._device = device

    # -------------------------------------------------------------------------
    # Brake Control
    # -------------------------------------------------------------------------

    def set_brake(self, released: bool) -> None:
        """
        Control brake output (OUTbit0) via I/O Control command.

        The brake output is typically connected to an external motor brake.
        When released=True, the brake is released (motor can move).
        When released=False, the brake is engaged (motor locked).

        Args:
            released: True to release brake, False to engage

        Note:
            Uses I/O Control command (CMD 0x08) to control OUTbit0.
            BrakeMODE (OUTbit0 bit) must be set to 1 for manual control.
        """
        from .servo import CMD_IO_CTRL

        # I/O Control byte format:
        # Bit 0 (OUTbit0): Brake output
        # Bit 1-7: Other outputs
        # When BrakeMODE=1: Bit 0 controls brake directly
        # Bit=0: Brake engaged, Bit=1: Brake released

        io_byte = 0x01 if released else 0x00

        # Send I/O control command
        # Note: This sets OUTbit0 only. To control multiple outputs,
        # would need to read current state and merge
        self._device.send_command(CMD_IO_CTRL, [io_byte])

    def set_output(self, output_num: int, state: bool) -> None:
        """
        Set digital output state via I/O Control command.

        Args:
            output_num: Output number (0-7)
            state: True for HIGH, False for LOW

        Raises:
            ValueError: If output_num is out of range
        """
        from .servo import CMD_IO_CTRL

        if output_num < 0 or output_num > 7:
            raise ValueError(f"Output number {output_num} out of range (0-7)")

        # Create output byte with specified bit set/cleared
        io_byte = (1 << output_num) if state else 0

        self._device.send_command(CMD_IO_CTRL, [io_byte])

    def set_outputs(self, output_mask: int) -> None:
        """
        Set multiple digital outputs simultaneously.

        Args:
            output_mask: 8-bit mask where each bit controls one output
                        Bit 0 = OUTbit0 (brake)
                        Bit 1 = OUTbit1
                        ...
                        Bit 7 = OUTbit7
        """
        from .servo import CMD_IO_CTRL

        self._device.send_command(CMD_IO_CTRL, [output_mask & 0xFF])

    def read_inputs(self) -> Dict[str, bool]:
        """
        Read all digital input states.

        Returns:
            Dictionary with input names and states:
            {
                'home_source': bool,  # Home switch state
                'limit2': bool,       # Forward limit state
                'index': bool,        # Index signal state
                ...
            }

        Note:
            Input states are available in the status byte and aux status byte.
            This method reads the current cached values from state.
        """
        return {
            'home_source': self._state.home_source,
            'limit2': self._state.limit2,
            'index': self._state.index,
        }

    # -------------------------------------------------------------------------
    # Limit Switch Monitoring
    # -------------------------------------------------------------------------

    def get_limit2_state(self) -> bool:
        """
        Get forward limit switch state.

        Returns:
            True if limit switch active, False otherwise
        """
        return self._state.limit2

    def get_home_switch_state(self) -> bool:
        """
        Get home switch state.

        Returns:
            True if home switch active, False otherwise
        """
        return self._state.home_source
