"""
LDCN Base Device Classes

Abstract base class and utilities for all LDCN devices.

Author: NickyDoes
License: GPL v2 or later
"""

from abc import ABC, abstractmethod
from typing import Optional, List, Dict, TYPE_CHECKING

from .protocol import (
    CMD_NOP,
    CMD_DEFINE_STATUS,
)

if TYPE_CHECKING:
    from .network import LDCNNetwork


class LDCNDevice(ABC):
    """
    Abstract base class for all LDCN devices.

    Provides common functionality for device communication and status reading.
    Device-specific operations are implemented in subclasses.

    Attributes:
        network: Reference to parent LDCNNetwork
        address: Device address (1-127)
        device_type: Device type string (e.g., "LS-231SE", "SK-2310g2")
        model_id: Device model ID from hardware (if known)
        version: Firmware version from hardware (if known)
    """

    def __init__(self, network: 'LDCNNetwork', address: int):
        """
        Initialize base device.

        Args:
            network: Parent LDCNNetwork object
            address: Device address (1-127)
        """
        self.network = network
        self.address = address
        self.device_type = "Unknown"
        self.model_id: Optional[int] = None
        self.version: Optional[int] = None

    def send_command(self, command: int, data: Optional[List[int]] = None) -> bytes:
        """
        Send command to this device.

        Delegates to network.send_command() with this device's address.

        Args:
            command: LDCN command code
            data: Data bytes

        Returns:
            Response bytes from device
        """
        return self.network.send_command(self.address, command, data)

    def nop(self) -> bytes:
        """
        Send NOP command, return status.

        Returns:
            Raw status response bytes
        """
        return self.send_command(CMD_NOP)

    def define_status(self, status_bits: int) -> None:
        """
        Configure status reporting (permanent).

        Args:
            status_bits: 8-bit or 16-bit status configuration

        Notes:
            - If status_bits <= 0xFF: Sends 1 byte (command byte 0x12)
            - If status_bits > 0xFF: Sends 2 bytes (command byte 0x22)
            - Per LS-231SE datasheet: "Number of data bytes: 1 or 2"
        """
        if status_bits <= 0xFF:
            # 1 byte sufficient for bits 0-7 (I/O devices, simple servo configs)
            self.send_command(CMD_DEFINE_STATUS, [status_bits & 0xFF])
        else:
            # 2 bytes needed for bits 8-15 (servo extended status)
            self.send_command(CMD_DEFINE_STATUS, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

    @abstractmethod
    def read_status(self) -> Dict:
        """
        Read device status (abstract - implemented by subclasses).

        Returns:
            Device-specific status dictionary
        """
        pass

    def __repr__(self) -> str:
        """Return string representation."""
        model = self.model_id if self.model_id is not None else 0
        return f"{self.device_type}(address={self.address}, model_id=0x{model:02X})"


class UnknownDevice(LDCNDevice):
    """
    Fallback device class for unrecognized LDCN devices.

    This class provides basic functionality for devices whose type
    is not yet implemented or recognized.
    """

    def __init__(self, network: 'LDCNNetwork', address: int):
        """Initialize unknown device."""
        super().__init__(network, address)
        self.device_type = "Unknown"

    def read_status(self) -> Dict:
        """
        Read basic status from unknown device.

        Returns:
            Dictionary with raw status byte
        """
        response = self.nop()
        if len(response) >= 1:
            return {'status': response[0], 'raw': response}
        return {'status': 0, 'raw': response}
