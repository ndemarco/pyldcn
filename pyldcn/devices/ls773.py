"""
LS-773 Network I/O Node

General-purpose LDCN I/O controller with:
- 10 digital inputs (INPUT_IN0-IN9)
- 7 digital outputs (OUTPUT_POWER, OUTPUT_OUT1-OUT6)
- PWM on OUTPUT_OUT1 and OUTPUT_OUT2 (channels 1 and 2)
- 3 analog inputs (8-bit)
- 32-bit counter/timer with prescaler

Author: NickyDoes
License: GPL v2 or later
"""

from typing import List
from pyldcn.network import LDCNNetwork
from .io import IOController
from .status.ls773_status import LS773Status


# =============================================================================
# LS-773 Device Constants
# =============================================================================

# Device identification
DEVICE_VERSION = 0x32  # Version 50 decimal

# Digital Inputs (Byte0: bits 0-7, Byte1: bits 0-1, bit 7)
INPUT_IN0 = 0
INPUT_IN1 = 1
INPUT_IN2 = 2
INPUT_IN3 = 3
INPUT_IN4 = 4
INPUT_IN5 = 5
INPUT_IN6 = 6
INPUT_IN7 = 7
INPUT_IN8 = 8   # Byte1 bit 0
INPUT_IN9 = 9   # Byte1 bit 1 (also counter input)

# Digital Outputs (only byte0 bits 0-6 used, byte1 must be 0x00)
OUTPUT_POWER = 0   # OUTPUT 0/POWER - Solid-state relay (0.5A source)
OUTPUT_OUT1 = 1    # OUTPUT 1 - Open collector (1A sink) with PWM
OUTPUT_OUT2 = 2    # OUTPUT 2 - Open collector (1A sink) with PWM
OUTPUT_OUT3 = 3    # OUTPUT 3 - Open collector (1A sink)
OUTPUT_OUT4 = 4    # OUTPUT 4 - Open collector (1A sink)
OUTPUT_OUT5 = 5    # OUTPUT 5 - Open collector (1A sink)
OUTPUT_OUT6 = 6    # OUTPUT 6 - Open collector (1A sink)

# Byte1 bit 7: Output short circuit flag (read-only)
OUTPUT_SHORT = 7   # Byte1 bit 7: 1 when outputs shorted to POWER(+)


class LS773(IOController):
    """
    LS-773 Network I/O Node.

    General-purpose I/O controller with 10 digital inputs, 7 digital outputs,
    3 analog inputs, PWM outputs, and counter/timer capabilities.

    Hardware Features:
        - 10 digital inputs with configurable pull-up/pull-down resistors
        - 7 digital outputs:
          * OUTPUT 0/POWER: Solid-state relay (0.5A source)
          * OUTPUT 1-6: Open collector (1A sink) with protective diodes
        - PWM mode on OUTPUT 1 and OUTPUT 2 (20 kHz)
        - 3 analog inputs (8-bit, 0-5V/10V/20V/30V selectable)
        - 32-bit counter/timer with prescaler (5 MHz clock)
        - Short circuit protection on all outputs

    Output Configuration:
        - Only byte0 bits 0-6 used for outputs
        - byte1 must always be 0x00 (except bit 7 read-only status)
        - PWM channels 1 and 2 map to OUTPUT 1 and OUTPUT 2
    """

    DEVICE_VERSION = DEVICE_VERSION

    def __init__(self, network: LDCNNetwork, address: int):
        """
        Initialize LS-773 I/O controller.

        Args:
            network: Parent LDCNNetwork object
            address: Device address (1-127)
        """
        super().__init__(network, address)
        self.status_manager = LS773Status(self)

    # -------------------------------------------------------------------------
    # Abstract Method Implementations
    # -------------------------------------------------------------------------

    def get_pwm_channels(self) -> List[int]:
        """
        Return list of available PWM channel numbers.

        Returns:
            [1, 2] - PWM on OUTPUT 1 and OUTPUT 2
        """
        return [1, 2]

    def _validate_outputs(self, byte0: int, byte1: int) -> None:
        """
        Validate output bytes before sending to device.

        LS-773 Constraints:
        - byte0: bits 0-6 are valid outputs (bit 7 unused)
        - byte1: must be 0x00 (bit 7 is read-only status)

        Args:
            byte0: Output byte 0 (bits 0-6 valid)
            byte1: Output byte 1 (must be 0x00)

        Raises:
            ValueError: If byte0 bit 7 is set or byte1 is non-zero
        """
        # Check byte0 bit 7 (unused)
        if byte0 & 0x80:
            raise ValueError(
                "LS-773 byte0 bit 7 is unused. Only bits 0-6 are valid outputs. "
                f"Received byte0=0x{byte0:02X}"
            )

        # Check byte1 (must be 0x00)
        if byte1 != 0x00:
            raise ValueError(
                "LS-773 byte1 must be 0x00 (no outputs on byte1). "
                f"Received byte1=0x{byte1:02X}"
            )

    # -------------------------------------------------------------------------
    # High-Level Output Control Methods
    # -------------------------------------------------------------------------

    def set_power_output(self, state: bool) -> None:
        """
        Control OUTPUT 0/POWER (solid-state relay).

        Args:
            state: True to enable, False to disable
        """
        self.set_output_bit(OUTPUT_POWER, state)

    def set_output(self, output: int, state: bool) -> None:
        """
        Control a specific output (OUTPUT 1-6).

        Args:
            output: Output number (1-6)
            state: True to enable, False to disable

        Raises:
            ValueError: If output number invalid
        """
        if not 1 <= output <= 6:
            raise ValueError(f"Output {output} out of range. Must be 1-6")

        self.set_output_bit(output, state)

    # -------------------------------------------------------------------------
    # Input Reading Methods
    # -------------------------------------------------------------------------

    def read_inputs(self) -> dict:
        """
        Read all digital inputs and return decoded status.

        Returns:
            Dictionary with input states and additional status fields
        """
        status = self.status_manager.read(mask=0x01)  # digital_inputs only
        return status

    def read_analog(self) -> dict:
        """
        Read all analog inputs.

        Returns:
            Dictionary with analog_in_0, analog_in_1, analog_in_2 (0-255)
        """
        status = self.status_manager.read(mask=0x0E)  # analog channels 0-2
        return {
            'analog_in_0': status.get('analog_in_0', 0),
            'analog_in_1': status.get('analog_in_1', 0),
            'analog_in_2': status.get('analog_in_2', 0),
        }

    def read_counter(self) -> int:
        """
        Read counter/timer value.

        Returns:
            32-bit counter/timer value
        """
        status = self.status_manager.read(mask=0x10)  # counter_timer
        return status.get('counter_timer', 0)

    # -------------------------------------------------------------------------
    # Counter/Timer Configuration
    # -------------------------------------------------------------------------

    def configure_counter(self, enabled: bool = True, counter_mode: bool = True,
                         prescaler: int = 0) -> None:
        """
        Configure counter/timer mode.

        Args:
            enabled: True to enable counter/timer
            counter_mode: True for counter mode (INPUT 9 transitions),
                         False for timer mode (5 MHz internal clock)
            prescaler: Prescaler value (0=none, 1=2:1, 2=4:1, 3=8:1)

        Raises:
            ValueError: If prescaler out of range
        """
        if not 0 <= prescaler <= 3:
            raise ValueError(f"Prescaler {prescaler} out of range. Must be 0-3")

        # Build timer mode byte
        mode = 0x00
        if enabled:
            mode |= 0x01
        if counter_mode:
            mode |= 0x02
        mode |= (prescaler << 4)

        self.set_timer_mode(mode)

    def reset_counter(self) -> None:
        """
        Reset counter/timer to zero by disabling and re-enabling.
        """
        # Disable counter/timer
        self.set_timer_mode(0x00)
        # Re-enable in counter mode with no prescaler
        self.set_timer_mode(0x03)
