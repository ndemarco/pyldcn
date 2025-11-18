"""
SK-2310g2 I/O Controller Status Management

Status parsing and diagnostic decoding for SK-2310g2 supervisory I/O controller.

The SK-2310g2 uses LS-773 I/O device protocol format with:
- 8-bit status mask (bits 0-7)
- Variable-length status response based on configured mask
- Diagnostic code embedded in Byte1 bits [7:3]

Author: NickyDoes
License: GPL v2 or later
"""

from typing import Dict, Any
from .io_status import IOStatus


# =============================================================================
# SK-2310g2 Status Items (LS-773 I/O Device Protocol)
# =============================================================================

# Format: [(bit_mask, field_name, byte_size, description), ...]
SK2310G2_STATUS_ITEMS = [
    (0x01, 'digital_inputs', 2, '16-bit digital input state (Byte 0 + Byte 1)'),
    (0x02, 'analog_in_0', 1, 'Analog input channel 0 (0-255, typically spindle load)'),
    (0x04, 'analog_in_1', 1, 'Analog input channel 1 (0-255, general purpose)'),
    (0x08, 'analog_in_2', 1, 'Analog input channel 2 (0-255, general purpose)'),
    (0x10, 'counter_timer', 4, 'Counter/timer value (32-bit, LSB first)'),
    (0x20, 'device_id', 2, 'Device ID and firmware version'),
    (0x40, 'sync_inputs', 2, 'Digital inputs captured with Sync Input command'),
    (0x80, 'sync_counter', 4, 'Counter/timer captured with Sync Input command'),
]


# =============================================================================
# Diagnostic Code Descriptions
# =============================================================================

DIAGNOSTIC_CODES = {
    0x00: "Power OFF delay in progress",
    0x01: "Initializing",
    0x02: "Control voltage shorted",
    0x03: "Output shorted",
    0x04: "Control voltage LOW (less than 18V)",
    0x05: "Safe zone/Manual override switch malfunction (both contacts ON)",
    0x06: "Power UP safe zone sensor error",
    0x07: "Power UP manual override switch error",
    0x08: "System LOCKED",
    0x09: "Watchdog Stop",
    0x0A: "Safety Link Error",
    0x0B: "Guard Open Stop - Guards open, spindle not stopped",
    0x0C: "Guard Open Stop - Guards open, not in safe zone",
    0x0D: "Guard Open Stop - Manual override without Enable button held",
    0x0E: "Guard contact fault (one or more contacts malfunctioning)",
    0x0F: "Limit Switch Stop",
    0x10: "Emergency Stop",
    0x11: "Emergency Stop contact fault or Monitor Loop Open",
    0x12: "Busy (≤6s) or Power button short/Monitor Loop Open (>6s)",
    0x13: "Motor Power Supply under-voltage",
    0x14: "Guard-1 Open; Guard-2 Open (ready to power)",
    0x15: "Guard-1 Closed; Guard-2 Open (ready to power)",
    0x16: "Guard-1 Open; Guard-2 Closed (ready to power)",
    0x17: "Guard-1 Closed; Guard-2 Closed (ready to power)",
    0x18: "Guard-1 Open; Guard-2 Open; Manual override",
    0x19: "Guard-1 Closed; Guard-2 Open; Manual override",
    0x1A: "Guard-1 Open; Guard-2 Closed; Manual override",
    0x1B: "Guard-1 Closed; Guard-2 Closed; Manual override",
    0x1C: "Guard-1 Open; Guard-2 Open; Safe zone; Spindle stopped",
    0x1D: "Guard-1 Closed; Guard-2 Open; Safe zone; Spindle stopped",
    0x1E: "Guard-1 Open; Guard-2 Closed; Safe zone; Spindle stopped",
    0x1F: "Normal operation - All guards closed",
}


class SK2310g2Status(IOStatus):
    """
    SK-2310g2 I/O controller status management.

    Handles status reading, parsing, and diagnostic decoding for the
    SK-2310g2 supervisory I/O controller using LS-773 I/O device protocol.

    Status Response Format (LS-773):
        [status_byte(1)] [status_items based on mask] [checksum(1)]

    Example with status_mask=0x01 (digital inputs only):
        [status(1)] [byte0(1)] [byte1(1)] [checksum(1)] = 4 bytes

    Diagnostic Code:
        Embedded in Byte1 bits [7:3] of digital inputs
        See DIAGNOSTIC_CODES for human-readable descriptions
    """

    def __init__(self, device):
        """
        Initialize SK-2310g2 status manager.

        Args:
            device: Parent SK2310g2 device instance
        """
        super().__init__(device)
        # Override with SK2310g2-specific status items
        self.status_items = SK2310G2_STATUS_ITEMS

    def _decode_inputs(self, status: Dict[str, Any]) -> None:
        """
        Decode SK-2310g2 specific digital inputs.

        Adds device-specific decoded input fields to status dictionary.

        Args:
            status: Status dictionary to modify in-place
        """
        byte0 = status.get('byte0', 0)
        byte1 = status.get('byte1', 0)

        # Extract diagnostic code from byte1 bits [7:3] (SK-2310g2 manual page 20)
        diagnostic = (byte1 >> 3) & 0x1F

        # Decode Byte0 digital inputs
        status['input1'] = bool(byte0 & 0x01)
        status['input2'] = bool(byte0 & 0x02)
        status['spindle_stopped'] = bool(byte0 & 0x04)
        status['spindle_fault'] = bool(byte0 & 0x08)
        status['input3'] = bool(byte0 & 0x10)
        status['input4'] = bool(byte0 & 0x20)
        status['input5'] = bool(byte0 & 0x40)
        status['input6'] = bool(byte0 & 0x80)

        # Decode Byte1 internal status
        status['safe_state'] = bool(byte1 & 0x01)
        status['manual_override'] = bool(byte1 & 0x02)
        status['servo_fault'] = bool(byte1 & 0x04)
        # Bits [7:3] are diagnostic code

        # SK-2310g2 specific fields
        status['diagnostic'] = diagnostic

        # Power state inferred from diagnostic code patterns (SK-2310g2 manual page 20)
        # READY_TO_POWER: 0x14-0x17 (guards in various states, power not yet enabled)
        # POWER_ON: 0x13 (under-voltage condition), 0x18-0x1F (normal powered operation)
        status['power_state'] = (diagnostic == 0x13) or (0x18 <= diagnostic <= 0x1F)

    def get_diagnostic_description(self, code: int) -> str:
        """
        Get human-readable description for diagnostic code.

        Args:
            code: 5-bit diagnostic code (0x00-0x1F)

        Returns:
            Description string
        """
        return DIAGNOSTIC_CODES.get(code, f"Unknown diagnostic code 0x{code:02X}")
