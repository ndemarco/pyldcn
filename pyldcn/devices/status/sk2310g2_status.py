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
from pyldcn.device_status import StatusManager


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
    0x05: "Home/Test switch malfunction",
    0x06: "Power UP Home error",
    0x07: "Power UP manual override error",
    0x08: "System LOCKED",
    0x09: "Watchdog Stop",
    0x0A: "Safety Link Error",
    0x0B: "Guard Open Stop - spindle not stopped",
    0x0C: "Guard Open Stop - not in safe zone",
    0x0D: "Guard Open Stop - manual override w/o Enable",
    0x0E: "Guard contact fault",
    0x0F: "Limit Switch Stop",
    0x10: "Emergency Stop",
    0x11: "E-Stop contact fault or Monitor Loop Open",
    0x12: "Busy or Power button short/Monitor Loop Open",
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


class SK2310g2Status(StatusManager):
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
        self.status_items = SK2310G2_STATUS_ITEMS

    def parse(self, response: bytes, mask: int) -> Dict[str, Any]:
        """
        Parse SK-2310g2 status response.

        Parses variable-length response based on mask, decoding:
        - Digital inputs (Byte0, Byte1)
        - Analog inputs (3 channels)
        - Counter/timer
        - Device ID and version
        - Diagnostic code (from Byte1)
        - Power state (derived from diagnostic)

        Args:
            response: Raw response bytes from device
            mask: Status mask indicating which fields are present

        Returns:
            Dictionary with parsed status fields
        """
        if len(response) < 1:
            return {}

        idx = 0
        status_byte = response[idx]
        idx += 1

        # Initialize all fields with defaults
        byte0 = 0
        byte1 = 0
        analog_in_0 = 0
        analog_in_1 = 0
        analog_in_2 = 0
        counter_timer = 0
        device_id = 0
        version = 0
        sync_byte0 = 0
        sync_byte1 = 0
        sync_counter = 0

        # Parse fields based on status_mask
        # Process bits in order from SK2310G2_STATUS_ITEMS

        # Bit 0 (0x01): digital_inputs (2 bytes - byte0, byte1)
        if mask & 0x01 and idx + 2 <= len(response):
            byte0 = response[idx]
            idx += 1
            byte1 = response[idx]
            idx += 1

        # Bit 1 (0x02): analog_in_0 (1 byte)
        if mask & 0x02 and idx + 1 <= len(response):
            analog_in_0 = response[idx]
            idx += 1

        # Bit 2 (0x04): analog_in_1 (1 byte)
        if mask & 0x04 and idx + 1 <= len(response):
            analog_in_1 = response[idx]
            idx += 1

        # Bit 3 (0x08): analog_in_2 (1 byte)
        if mask & 0x08 and idx + 1 <= len(response):
            analog_in_2 = response[idx]
            idx += 1

        # Bit 4 (0x10): counter_timer (4 bytes, LSB first)
        if mask & 0x10 and idx + 4 <= len(response):
            counter_timer = int.from_bytes(response[idx:idx+4], 'little', signed=False)
            idx += 4

        # Bit 5 (0x20): device_id (2 bytes)
        if mask & 0x20 and idx + 2 <= len(response):
            device_id_raw = int.from_bytes(response[idx:idx+2], 'little')
            idx += 2
            device_id = device_id_raw & 0xFF
            version = (device_id_raw >> 8) & 0xFF

        # Bit 6 (0x40): sync_inputs (2 bytes)
        if mask & 0x40 and idx + 2 <= len(response):
            sync_byte0 = response[idx]
            idx += 1
            sync_byte1 = response[idx]
            idx += 1

        # Bit 7 (0x80): sync_counter (4 bytes, LSB first)
        if mask & 0x80 and idx + 4 <= len(response):
            sync_counter = int.from_bytes(response[idx:idx+4], 'little', signed=False)
            idx += 4

        # Extract diagnostic code from byte1 bits [7:3] (SK-2310g2 manual page 20)
        diagnostic = (byte1 >> 3) & 0x1F

        # Decode Byte0 digital inputs
        input1 = bool(byte0 & 0x01)
        input2 = bool(byte0 & 0x02)
        spindle_stopped = bool(byte0 & 0x04)
        spindle_fault = bool(byte0 & 0x08)
        input3 = bool(byte0 & 0x10)
        input4 = bool(byte0 & 0x20)
        input5 = bool(byte0 & 0x40)
        input6 = bool(byte0 & 0x80)

        # Decode Byte1 internal status
        safe_state = bool(byte1 & 0x01)
        manual_override = bool(byte1 & 0x02)
        servo_fault = bool(byte1 & 0x04)
        # Bits [7:3] are diagnostic code (already extracted)

        # Power state inferred from diagnostic code patterns (SK-2310g2 manual page 20)
        # READY_TO_POWER: 0x14-0x17 (guards in various states, power not yet enabled)
        # POWER_ON: 0x13 (under-voltage condition), 0x18-0x1F (normal powered operation)
        power_state = (diagnostic == 0x13) or (0x18 <= diagnostic <= 0x1F)

        return {
            # Raw bytes
            'status': status_byte,
            'byte0': byte0,
            'byte1': byte1,

            # Status items (mapped from status_mask bits)
            'analog_in_0': analog_in_0,
            'analog_in_1': analog_in_1,
            'analog_in_2': analog_in_2,
            'counter_timer': counter_timer,
            'device_id': device_id,
            'version': version,
            'sync_byte0': sync_byte0,
            'sync_byte1': sync_byte1,
            'sync_counter': sync_counter,

            # SK-2310g2 specific fields
            'diagnostic': diagnostic,
            'power_state': power_state,

            # Byte0 - Digital inputs
            'input1': input1,
            'input2': input2,
            'spindle_stopped': spindle_stopped,
            'spindle_fault': spindle_fault,
            'input3': input3,
            'input4': input4,
            'input5': input5,
            'input6': input6,

            # Byte1 - Internal status
            'safe_state': safe_state,
            'manual_override': manual_override,
            'servo_fault': servo_fault,
        }

    def get_diagnostic_description(self, code: int) -> str:
        """
        Get human-readable description for diagnostic code.

        Args:
            code: 5-bit diagnostic code (0x00-0x1F)

        Returns:
            Description string
        """
        return DIAGNOSTIC_CODES.get(code, f"Unknown diagnostic code 0x{code:02X}")
