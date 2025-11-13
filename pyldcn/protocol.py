"""
LDCN Protocol Layer

Low-level LDCN protocol implementation handling serial communication,
packet formatting, and baud rate management.

Author: NickyDoes
License: GPL v2 or later
"""

import serial
import time
from typing import Optional, List

from .exceptions import (
    LDCNError,
    LDCNTimeoutError,
    LDCNChecksumError,
    LDCNDetectionError,
)


# =============================================================================
# Protocol Constants
# =============================================================================

# Protocol framing
HEADER = 0xAA
ADDRESS_UNADDRESSED = 0x00
ADDRESS_GROUP = 0xFF

# Generic LDCN commands (supported by all device types)
CMD_RESET_POS = 0x00
CMD_SET_ADDRESS = 0x01
CMD_DEFINE_STATUS = 0x02
CMD_READ_STATUS = 0x03
CMD_SET_BAUD = 0x0A
CMD_NOP = 0x0E
CMD_HARD_RESET = 0x0F

# Baud rate divisor (BRD) values
BAUD_RATES = {
    9600: 0x81,
    19200: 0x3F,
    57600: 0x14,
    115200: 0x0A,
    125000: 0x27,
    312500: 0x0F,
    625000: 0x07,
    1250000: 0x03
}

# Default baud rate after reset
DEFAULT_BAUD = 19200

# Common baud rates for auto-detection (in order of likelihood)
# 125000 first - typical operating speed after initialization
COMMON_BAUDS = [125000, 19200, 115200, 57600, 9600]

# Timing constants (seconds)
DELAY_AFTER_COMMAND = 0.001  # 1ms - devices respond within microseconds
DELAY_AFTER_RESET = 2.0
DELAY_AFTER_ADDRESS = 0.05   # Reduced from 0.3s
DELAY_AFTER_BAUD_CHANGE = 0.1  # Reduced from 0.5s


class LDCNProtocol:
    """
    Low-level LDCN protocol handler.

    Manages serial port communication and implements the LDCN packet protocol.
    All device communication flows through send_command().

    Attributes:
        port: Serial port path (e.g., '/dev/ttyUSB0')
        baud_rate: Current baud rate
        serial: PySerial Serial object
        timeout: Serial read timeout in seconds
    """

    def __init__(self, port: str, timeout: float = 0.015):
        """
        Initialize LDCN protocol handler.

        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            timeout: Serial read timeout in seconds (default 0.015s/15ms)
        """
        self.port = port
        self.baud_rate = DEFAULT_BAUD
        self.serial: Optional[serial.Serial] = None
        self.timeout = timeout

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    def open(self, baud: int = DEFAULT_BAUD) -> None:
        """
        Open serial port at specified baud rate.

        Args:
            baud: Baud rate (must be in BAUD_RATES dict)

        Raises:
            ValueError: If baud rate not supported
        """
        self._open_port(baud)

    def close(self) -> None:
        """Close serial port and cleanup resources."""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.serial = None

    def _open_port(self, baud: int) -> None:
        """
        Internal: Open serial port at specific baud rate.

        Args:
            baud: Baud rate (must be in BAUD_RATES dict)

        Raises:
            ValueError: If baud rate not supported
        """
        if baud not in BAUD_RATES:
            raise ValueError(f"Unsupported baud rate: {baud}")

        # Close existing connection
        if self.serial and self.serial.is_open:
            self.serial.close()
            time.sleep(0.05)  # Brief delay for port cleanup

        # Open at specified baud rate
        self.serial = serial.Serial(
            port=self.port,
            baudrate=baud,
            timeout=self.timeout,
            write_timeout=self.timeout,
            inter_byte_timeout=0.01  # 10ms max gap between bytes
        )
        self.baud_rate = baud
        time.sleep(0.05)  # Brief delay for port to stabilize

    # -------------------------------------------------------------------------
    # Core Protocol
    # -------------------------------------------------------------------------

    def send_command(self, address: int, command: int, data: Optional[List[int]] = None) -> bytes:
        """
        Send LDCN command packet and return response.

        This is the core LDCN communication method.

        Packet format:
            [HEADER] [ADDRESS] [CMD_BYTE] [DATA...] [CHECKSUM]

        Where:
            CMD_BYTE = (len(data) << 4) | (command & 0x0F)
            CHECKSUM = (address + cmd_byte + sum(data)) & 0xFF

        Args:
            address: Device address (1-127) or group (128-255)
            command: LDCN command (0x00-0x0F)
            data: Data bytes (0-16 bytes)

        Returns:
            Response bytes from device (status + data + checksum)

        Raises:
            LDCNTimeoutError: No response received
            LDCNChecksumError: Response checksum mismatch
        """
        if data is None:
            data = []

        if not self.serial or not self.serial.is_open:
            raise LDCNError("Serial port not open")

        # Build packet
        header = HEADER
        num_data = len(data)
        cmd_byte = (num_data << 4) | (command & 0x0F)
        checksum = (address + cmd_byte + sum(data)) & 0xFF
        packet = bytes([header, address, cmd_byte] + data + [checksum])

        # Flush input buffer to discard any stale data
        self.serial.reset_input_buffer()

        # Send packet
        self.serial.write(packet)
        self.serial.flush()

        # Read response with generous buffer (max LDCN response ~64 bytes)
        # Serial timeout will stop reading when no more data arrives
        response = self.serial.read(64)

        if len(response) < 2:
            # Some commands (like SET_BAUD to group address) don't return responses
            if address == ADDRESS_GROUP:
                return b''
            raise LDCNTimeoutError(f"No response from address {address}")

        # Verify checksum
        if not self._verify_checksum(response):
            raise LDCNChecksumError(f"Checksum mismatch in response from address {address}")

        return response

    def _verify_checksum(self, response: bytes) -> bool:
        """
        Verify checksum of response packet.

        Args:
            response: Response bytes

        Returns:
            True if checksum valid
        """
        if len(response) < 2:
            return False

        expected = sum(response[:-1]) & 0xFF
        actual = response[-1]
        return expected == actual

    # -------------------------------------------------------------------------
    # Baud Rate Management
    # -------------------------------------------------------------------------

    def set_baud_rate(self, baud: int) -> None:
        """
        Upgrade network baud rate for all devices.

        Steps:
        1. Send SET_BAUD command to group address 0xFF
        2. Close serial port
        3. Wait for devices to process
        4. Reopen serial port at new baud rate

        Args:
            baud: Target baud rate (must be in BAUD_RATES dict)

        Raises:
            ValueError: If baud rate not supported
        """
        if baud not in BAUD_RATES:
            raise ValueError(f"Unsupported baud rate: {baud}. Supported: {list(BAUD_RATES.keys())}")

        # Send SET_BAUD to group address
        brd_value = BAUD_RATES[baud]
        packet = bytes([HEADER, ADDRESS_GROUP, 0x1A, brd_value, (ADDRESS_GROUP + 0x1A + brd_value) & 0xFF])

        if self.serial and self.serial.is_open:
            self.serial.write(packet)
            self.serial.flush()

        # Close port, wait, reopen at new baud
        time.sleep(DELAY_AFTER_BAUD_CHANGE)
        self._open_port(baud)

    def auto_detect_baud(self, baud_list: Optional[List[int]] = None) -> int:
        """
        Auto-detect current network baud rate.

        Tries common baud rates in order of likelihood until a device responds.

        Args:
            baud_list: List of baud rates to try (default: COMMON_BAUDS)

        Returns:
            Detected baud rate

        Raises:
            LDCNDetectionError: No response at any baud rate
        """
        if baud_list is None:
            baud_list = COMMON_BAUDS

        for baud in baud_list:
            if self._try_baud(baud):
                return baud

        # No devices responding at any baud rate
        raise LDCNDetectionError("No devices responding at any baud rate")

    def _try_baud(self, baud: int) -> bool:
        """
        Test if devices respond at specific baud rate.

        Tries to communicate with common addresses (1, 2, 3, 6) using NOP.

        Args:
            baud: Baud rate to test

        Returns:
            True if any device responds
        """
        try:
            self._open_port(baud)

            # Try common addresses
            for addr in [1, 2, 3, 6]:
                try:
                    response = self.send_command(addr, CMD_NOP)
                    if len(response) >= 2:
                        return True
                except (LDCNTimeoutError, LDCNChecksumError):
                    continue

            return False
        except Exception:
            return False
