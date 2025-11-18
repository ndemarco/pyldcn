"""
LS-773 Network I/O Node Status Management

Status parsing and input decoding for LS-773 general-purpose I/O controller.

The LS-773 uses standard I/O device protocol format with:
- 8-bit status mask (bits 0-7)
- Variable-length status response based on configured mask
- 10 digital inputs across byte0 and byte1
- Output short circuit detection flag in byte1 bit 7

Author: NickyDoes
License: GPL v2 or later
"""

from typing import Dict, Any
from .io_status import IOStatus


class LS773Status(IOStatus):
    """
    LS-773 I/O controller status management.

    Handles status reading, parsing, and input decoding for the
    LS-773 general-purpose I/O controller.

    Status Response Format (I/O Device Protocol):
        [status_byte(1)] [status_items based on mask] [checksum(1)]

    Example with status_mask=0x01 (digital inputs only):
        [status(1)] [byte0(1)] [byte1(1)] [checksum(1)] = 4 bytes

    Digital Inputs (LS-773 manual page 9):
        Byte0 [7:0]: INPUT 7, INPUT 6, INPUT 5, INPUT 4, INPUT 3, INPUT 2, INPUT 1, INPUT 0
        Byte1 [7:0]: OUT_SH, X, X, X, X, X, INPUT 9, INPUT 8

    Where:
        - INPUT 0-9: Digital input states
        - INPUT 9: Also serves as counter input
        - OUT_SH: Output short circuit flag (1 when outputs shorted to POWER+)
        - X: Unused bits
    """

    def __init__(self, device):
        """
        Initialize LS-773 status manager.

        Args:
            device: Parent LS773 device instance
        """
        super().__init__(device)
        # LS-773 uses standard IO_STATUS_ITEMS from IOStatus base class

    def _decode_inputs(self, status: Dict[str, Any]) -> None:
        """
        Decode LS-773 specific digital inputs.

        Adds device-specific decoded input fields to status dictionary.

        Args:
            status: Status dictionary to modify in-place
        """
        byte0 = status.get('byte0', 0)
        byte1 = status.get('byte1', 0)

        # Decode Byte0 digital inputs (INPUT 0-7)
        status['input0'] = bool(byte0 & 0x01)
        status['input1'] = bool(byte0 & 0x02)
        status['input2'] = bool(byte0 & 0x04)
        status['input3'] = bool(byte0 & 0x08)
        status['input4'] = bool(byte0 & 0x10)
        status['input5'] = bool(byte0 & 0x20)
        status['input6'] = bool(byte0 & 0x40)
        status['input7'] = bool(byte0 & 0x80)

        # Decode Byte1 digital inputs (INPUT 8-9) and status
        status['input8'] = bool(byte1 & 0x01)
        status['input9'] = bool(byte1 & 0x02)  # Also counter input
        # Bits 2-6 unused
        status['output_short'] = bool(byte1 & 0x80)  # Short circuit flag

        # Create convenience list of all inputs
        status['inputs'] = [
            status['input0'],
            status['input1'],
            status['input2'],
            status['input3'],
            status['input4'],
            status['input5'],
            status['input6'],
            status['input7'],
            status['input8'],
            status['input9'],
        ]

        # Counter input state (INPUT 9)
        status['counter_input'] = status['input9']
