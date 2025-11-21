"""
LS-231SE Servo Drive I/O Subsystem

I/O control operations (brake, limits, switches).

Author: NickyDoes
License: GPL v2 or later
"""

from typing import TYPE_CHECKING, Dict, Tuple

from .servo_state import ServoState
from .servo_mappings import IOMapping, LS231SE_IO_MAPPING

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

    def __init__(self, state: ServoState, device: 'LS231SE', mapping: IOMapping = LS231SE_IO_MAPPING):
        """
        Initialize IO subsystem.

        Args:
            state: Shared ServoState object
            device: Parent LS231SE device instance
        """
        self._state = state
        self._device = device
        self._mapping = mapping

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
            Bit 0: Mode select (0=automatic, 1=manual control)
            Bit 1: Brake state when manual (0=brake OFF/released, 1=brake ON/engaged)
        """
        from .servo import CMD_IO_CTRL

        # I/O Control byte format (from servo_commands.md):
        # Bit 0: Brake mode (0=automatic, 1=manual)
        # Bit 1: Brake output (0=brake OFF/released, 1=brake ON/engaged)
        # For manual control, bit 0 must be 1
        #
        # released=True  → brake OFF/released → 0x01 (bit0=1 manual, bit1=0 OFF)
        # released=False → brake ON/engaged   → 0x03 (bit0=1 manual, bit1=1 ON)

        io_byte = 0x01 if released else 0x03

        # Send I/O control command
        # Note: This sets bits 0-1 only. To control multiple outputs,
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
        Read all digital input states (raw bit values).

        Returns:
            Dictionary with generic bit names and boolean states:
            {
                'home_source': bool,  # Status bit 5
                'limit2': bool,       # Status bit 6
                'index': bool,        # Aux bit 0
            }

        Note:
            For physical signal names mapped via HomeSEL, use read_mapped_inputs().
        """
        return {
            'home_source': self._state.home_source,
            'limit2': self._state.limit2,
            'index': self._state.index,
        }

    def read_mapped_inputs(self) -> Dict[str, Dict[str, str]]:
        """
        Read digital inputs with physical signal names and label-style states.

        Applies HomeSEL mapping to show actual physical inputs being monitored.

        Returns:
            Dictionary with physical signal names as keys, each containing:
            {
                'signal_name': {
                    'label': str,      # Display label (e.g., "Limit1", "HomeIN")
                    'state': str,      # Label-style state ("ACTIVE" or "INACTIVE")
                    'raw_value': bool  # Raw boolean value
                }
            }

        Example:
            >>> inputs = servo.io.read_mapped_inputs()
            >>> print(inputs)
            {
                'bit5': {'label': 'Limit1', 'state': 'INACTIVE', 'raw_value': False},
                'bit6': {'label': 'Limit2', 'state': 'INACTIVE', 'raw_value': False},
                'index': {'label': 'Index', 'state': 'ACTIVE', 'raw_value': True}
            }
        """
        from .servo_mappings import LS231SE_HOME_SOURCE_TABLE

        # Get current HomeSEL configuration
        homesel = self._state.home_selection
        lookup = {(e.homesel2, e.homesel1): e for e in LS231SE_HOME_SOURCE_TABLE}
        entry = lookup.get(homesel)

        # Read raw values
        raw = self.read_inputs()

        # Build mapped output
        result = {}

        # Bit 5 (home_source) - mapped via HomeSEL
        bit5_label = entry.status_bit5 if entry else 'Unknown'
        result['bit5'] = {
            'label': bit5_label,
            'state': 'ACTIVE' if raw['home_source'] else 'INACTIVE',
            'raw_value': raw['home_source']
        }

        # Bit 6 (limit2) - mapped via HomeSEL
        bit6_label = entry.status_bit6 if entry else 'Unknown'
        result['bit6'] = {
            'label': bit6_label,
            'state': 'ACTIVE' if raw['limit2'] else 'INACTIVE',
            'raw_value': raw['limit2']
        }

        # Index - always the same signal
        result['index'] = {
            'label': 'Index',
            'state': 'ACTIVE' if raw['index'] else 'INACTIVE',
            'raw_value': raw['index']
        }

        return result

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

    # -------------------------------------------------------------------------
    # Home Source Selection Tracking
    # -------------------------------------------------------------------------

    def set_home_selection_state(self, homesel2: int, homesel1: int) -> None:
        """
        Record the currently selected HomeSEL bits.

        Note: This does NOT write to hardware; callers must issue the appropriate
        I/O control command separately. This method simply updates shared state
        so status decoding can reflect the correct physical mapping.
        """
        self._state.home_selection = (homesel2 & 0x01, homesel1 & 0x01)

    def get_home_selection_state(self) -> Tuple[int, int]:
        """Return the cached HomeSEL (homesel2, homesel1) tuple."""
        return self._state.home_selection
