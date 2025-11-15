"""
LDCN Device Status Management Base Classes

Provides common status mask tracking, DEFINE_STATUS/READ_STATUS operations,
and mask-based parsing for all LDCN devices.

This module abstracts the common patterns for:
- Tracking status mask state (what fields device is configured to return)
- Sending DEFINE_STATUS (permanent configuration) and READ_STATUS (one-time query)
- Parsing variable-length responses based on status item masks

Device-specific status implementations inherit from StatusManager and provide:
- status_items definitions (bit mask, field name, size, description)
- parse() method for device-specific status decoding

Author: NickyDoes
License: GPL v2 or later
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional, Any, TYPE_CHECKING
import struct

if TYPE_CHECKING:
    from .device import LDCNDevice


class StatusManager(ABC):
    """
    Base class for device status management.

    Handles common status operations for LDCN devices:
    - Status mask state tracking (DEFINE_STATUS configuration)
    - READ_STATUS command with optional one-time mask override
    - Mask-based field parsing via status item definitions
    - Device-specific status decoding (delegated to subclasses)

    Subclasses must:
    - Define status_items list: [(bit_mask, field_name, size, description), ...]
    - Implement parse() method for device-specific status decoding

    Attributes:
        status_mask: Current DEFINE_STATUS configuration (0x00 after reset)
        status_items: Device-specific status item definitions
    """

    def __init__(self, device: 'LDCNDevice'):
        """
        Initialize status manager.

        Args:
            device: Parent LDCNDevice instance
        """
        self._device = device
        self.status_mask: int = 0x00  # Matches hardware reset state
        self.status_items: List[Tuple[int, str, int, str]] = []

    def configure(self, mask: int) -> None:
        """
        Send DEFINE_STATUS command and update instance state.

        This permanently configures which status fields the device returns
        in all future NOP and status responses.

        Args:
            mask: Status bitmask (8-bit or 16-bit depending on device)
        """
        self._device.define_status(mask)
        self.status_mask = mask

    def read(self, mask: Optional[int] = None) -> Dict[str, Any]:
        """
        Read status via READ_STATUS command (one-time query).

        READ_STATUS allows requesting specific status fields for a single
        response without changing the persistent DEFINE_STATUS configuration.

        Args:
            mask: Status mask for this read (None = use configured mask)

        Returns:
            Parsed status dictionary (device-specific)
        """
        from .protocol import CMD_READ_STATUS

        # Use configured mask if none provided
        if mask is None:
            mask = self.status_mask

        # Send READ_STATUS command with mask
        if mask <= 0xFF:
            response = self._device.send_command(CMD_READ_STATUS, [mask & 0xFF])
        else:
            # 16-bit mask (for devices with extended status items)
            response = self._device.send_command(
                CMD_READ_STATUS,
                [mask & 0xFF, (mask >> 8) & 0xFF]
            )

        return self.parse(response, mask)

    @abstractmethod
    def parse(self, response: bytes, mask: int) -> Dict[str, Any]:
        """
        Parse device-specific status response.

        Subclasses implement this to decode status bytes into meaningful
        data structures (positions, velocities, diagnostics, I/O states, etc.).

        Args:
            response: Raw response bytes from device
            mask: Status mask used in query (indicates which fields are present)

        Returns:
            Parsed status dictionary (device-specific format)
        """
        pass

    def _parse_fields_by_mask(
        self,
        response: bytes,
        mask: int,
        start_idx: int = 1
    ) -> Dict[str, Any]:
        """
        Generic field parser using status_items definitions.

        Parses variable-length response by checking mask bits and extracting
        fields in order as defined in status_items.

        Args:
            response: Raw response bytes
            mask: Status mask indicating which fields are present
            start_idx: Index to start parsing (1 = after status byte)

        Returns:
            Dictionary of parsed fields
        """
        result = {}
        idx = start_idx

        for bit_mask, field_name, size, description in self.status_items:
            # Check if this field is present in the response
            if mask & bit_mask and idx + size <= len(response):
                if size == 1:
                    result[field_name] = response[idx]
                elif size == 2:
                    result[field_name] = struct.unpack('<h', response[idx:idx+2])[0]
                elif size == 4:
                    result[field_name] = struct.unpack('<i', response[idx:idx+4])[0]
                else:
                    # Variable size - return as bytes
                    result[field_name] = response[idx:idx+size]

                idx += size

        return result


class StatusItemParser:
    """
    Utility class for common status parsing operations.

    Provides static helper methods for bit extraction, flag decoding,
    and other common status parsing tasks.
    """

    @staticmethod
    def extract_bits(value: int, bit_map: Dict[int, str]) -> Dict[str, bool]:
        """
        Extract flag bits into named dictionary.

        Args:
            value: Integer with bit flags
            bit_map: {bit_position: field_name} mapping

        Returns:
            {field_name: bool} dictionary

        Example:
            >>> bit_map = {0: 'move_done', 1: 'error', 2: 'ready'}
            >>> extract_bits(0x05, bit_map)
            {'move_done': True, 'error': False, 'ready': True}
        """
        return {name: bool((value >> bit) & 0x01) for bit, name in bit_map.items()}

    @staticmethod
    def get_bit(value: int, bit: int) -> bool:
        """
        Extract a single bit from an integer value.

        Args:
            value: Integer value
            bit: Bit position (0-15)

        Returns:
            True if bit is set, False otherwise
        """
        return bool((value >> bit) & 0x01)
