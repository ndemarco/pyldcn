"""
I/O Controller Status Management Base Class

Base status management for LDCN I/O controllers (LS-773 and SK-2310g2).

Provides common status item definitions and parsing logic for I/O controllers
which share the same protocol format (8-bit status mask, bits 0-7).

Author: NickyDoes
License: GPL v2 or later
"""

from abc import abstractmethod
from typing import Dict, Any, List, Tuple
from pyldcn.device_status import StatusManager


# =============================================================================
# Common I/O Controller Status Items (8-bit mask, bits 0-7)
# =============================================================================

# Format: [(bit_mask, field_name, byte_size, description), ...]
IO_STATUS_ITEMS: List[Tuple[int, str, int, str]] = [
    (0x01, 'digital_inputs', 2, '16-bit digital input state (Byte0 + Byte1)'),
    (0x02, 'analog_in_0', 1, 'Analog input channel 0 (0-255)'),
    (0x04, 'analog_in_1', 1, 'Analog input channel 1 (0-255)'),
    (0x08, 'analog_in_2', 1, 'Analog input channel 2 (0-255)'),
    (0x10, 'counter_timer', 4, 'Counter/timer value (32-bit, 5MHz clock)'),
    (0x20, 'device_id', 2, 'Device ID and firmware version'),
    (0x40, 'sync_inputs', 2, 'Digital inputs captured with Sync Input command'),
    (0x80, 'sync_counter', 4, 'Counter/timer captured with Sync Input command'),
]


class IOStatus(StatusManager):
    """
    Base class for I/O controller status management.

    Provides common status parsing for LS-773 and SK-2310g2 I/O controllers,
    which share the same protocol format:
    - 8-bit status mask (bits 0-7)
    - Variable-length status response based on configured mask
    - Common status items: digital inputs, analog inputs, counter/timer

    Device-specific subclasses must implement:
    - _decode_inputs(): Convert raw input bytes to device-specific named fields
    - _decode_outputs(): Convert raw output bytes to device-specific named fields (if applicable)
    """

    def __init__(self, device):
        """
        Initialize I/O status manager.

        Args:
            device: Parent IOController device instance
        """
        super().__init__(device)
        self.status_items = IO_STATUS_ITEMS

    @abstractmethod
    def _decode_inputs(self, status: Dict[str, Any]) -> None:
        """
        Decode raw input bytes into device-specific named inputs.

        This method should extract byte0 and byte1 from status['digital_inputs']
        and add device-specific input field names to the status dictionary.

        Args:
            status: Status dictionary to modify in-place
        """
        pass

    def parse(self, response: bytes, mask: int) -> Dict[str, Any]:
        """
        Parse I/O controller status response.

        Parses variable-length response based on mask, extracting:
        - Digital inputs (Byte0, Byte1)
        - Analog inputs (3 channels)
        - Counter/timer (32-bit, 5MHz)
        - Device ID and version
        - Sync inputs and counter

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

        # Initialize status dictionary
        status: Dict[str, Any] = {
            'status': status_byte,
        }

        # Parse fields based on status mask (bits 0-7)

        # Bit 0 (0x01): digital_inputs (2 bytes - byte0, byte1)
        if mask & 0x01 and idx + 2 <= len(response):
            byte0 = response[idx]
            idx += 1
            byte1 = response[idx]
            idx += 1

            status['byte0'] = byte0
            status['byte1'] = byte1
            status['digital_inputs'] = (byte1 << 8) | byte0

        # Bit 1 (0x02): analog_in_0 (1 byte)
        if mask & 0x02 and idx + 1 <= len(response):
            status['analog_in_0'] = response[idx]
            idx += 1

        # Bit 2 (0x04): analog_in_1 (1 byte)
        if mask & 0x04 and idx + 1 <= len(response):
            status['analog_in_1'] = response[idx]
            idx += 1

        # Bit 3 (0x08): analog_in_2 (1 byte)
        if mask & 0x08 and idx + 1 <= len(response):
            status['analog_in_2'] = response[idx]
            idx += 1

        # Bit 4 (0x10): counter_timer (4 bytes, LSB first)
        if mask & 0x10 and idx + 4 <= len(response):
            status['counter_timer'] = int.from_bytes(
                response[idx:idx+4], 'little', signed=False
            )
            idx += 4

        # Bit 5 (0x20): device_id (2 bytes - ID in byte0, version in byte1)
        if mask & 0x20 and idx + 2 <= len(response):
            device_id_raw = int.from_bytes(response[idx:idx+2], 'little')
            idx += 2
            status['device_id'] = device_id_raw & 0xFF
            status['version'] = (device_id_raw >> 8) & 0xFF

        # Bit 6 (0x40): sync_inputs (2 bytes)
        if mask & 0x40 and idx + 2 <= len(response):
            sync_byte0 = response[idx]
            idx += 1
            sync_byte1 = response[idx]
            idx += 1

            status['sync_byte0'] = sync_byte0
            status['sync_byte1'] = sync_byte1
            status['sync_inputs'] = (sync_byte1 << 8) | sync_byte0

        # Bit 7 (0x80): sync_counter (4 bytes, LSB first)
        if mask & 0x80 and idx + 4 <= len(response):
            status['sync_counter'] = int.from_bytes(
                response[idx:idx+4], 'little', signed=False
            )
            idx += 4

        # Call device-specific input decoding if digital inputs present
        if 'digital_inputs' in status:
            self._decode_inputs(status)

        return status
