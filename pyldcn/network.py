#!/usr/bin/env python3
"""
ldcn_network.py - LDCN Network Communication Module

This module provides object-oriented Python classes for communicating with
Logosol LDCN (Logosol Distributed Control Network) devices including servo
drives and I/O controllers.

Architecture:
    LDCNNetwork - Top-level network manager (serial port, protocol)
    LDCNDevice - Abstract base class for all devices
    LS231SE - Servo drive specific implementation
    SK2310g2 - I/O controller specific implementation

Usage Example:
    with LDCNNetwork('/dev/ttyUSB0') as network:
        num_devices, device_info = network.initialize()
        network.set_baud_rate(125000)

        servo = network.devices[0]  # LS231SE at address 1
        servo.initialize()
        servo.move_to(position=10.0, velocity=100.0, accel=50.0, scale=2000.0)

⚠️ HARDWARE VERIFICATION STATUS: UNVERIFIED
All functions in this module are marked as UNVERIFIED until tested against
real LDCN hardware and behavior compared with original utility scripts.

Author: LinuxCNC Community
License: GPL v2 or later
Date: 2025-10-29
"""

import serial
import time
import struct
from typing import Optional, List, Dict, Tuple
from abc import ABC, abstractmethod
from enum import IntEnum


# =============================================================================
# Exception Classes
# =============================================================================

class LDCNError(Exception):
    """Base exception for LDCN errors."""
    pass


class LDCNTimeoutError(LDCNError):
    """No response from device (timeout)."""
    pass


class LDCNChecksumError(LDCNError):
    """Response checksum mismatch."""
    pass


class LDCNDetectionError(LDCNError):
    """Auto-detection failed."""
    pass


class LDCNInitializationError(LDCNError):
    """Device initialization failed."""
    pass


# =============================================================================
# Protocol Constants
# =============================================================================

# Protocol header
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

# Device-specific commands (used by servo drives)
CMD_LOAD_TRAJECTORY = 0x04
CMD_START_MOTION = 0x05
CMD_LOAD_GAINS = 0x06
CMD_STOP_MOTOR = 0x07
CMD_CLEAR_BITS = 0x0B

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
COMMON_BAUDS = [19200, 125000, 115200, 57600, 9600]

# Timing constants (seconds)
DELAY_AFTER_COMMAND = 0.02
DELAY_AFTER_RESET = 2.0
DELAY_AFTER_ADDRESS = 0.3
DELAY_AFTER_BAUD_CHANGE = 0.5

# Status bits for device discovery
STATUS_BIT_POSITION = 0x0001      # Bit 0: Position (4 bytes)
STATUS_BIT_AD_VALUE = 0x0002      # Bit 1: A/D value (1 byte)
STATUS_BIT_VELOCITY = 0x0004      # Bit 2: Velocity (2 bytes)
STATUS_BIT_AUX = 0x0008           # Bit 3: Auxiliary status byte
STATUS_BIT_HOME = 0x0010          # Bit 4: Home position (4 bytes)
STATUS_BIT_DEVICE_ID = 0x0020     # Bit 5: Device ID and version (2 bytes)
STATUS_BIT_POS_ERROR = 0x0040     # Bit 6: Position error (2 bytes)
STATUS_BIT_PATH_COUNT = 0x0080    # Bit 7: Path buffer count (1 byte)

# Device IDs (hardware-reported, TBD - verify from real hardware)
DEVICE_ID_UNKNOWN = 0x00
DEVICE_ID_LS231SE = 0x17          # TBD - verify with hardware
DEVICE_ID_SK2310G2 = 0x23         # TBD - verify with hardware

# Servo status bit masks
STATUS_MOVE_DONE = 0x01
STATUS_CKSUM_ERROR = 0x02
STATUS_CURRENT_LIMIT = 0x04
STATUS_POWER_ON = 0x08
STATUS_POS_ERROR = 0x10
STATUS_HOME_SOURCE = 0x20
STATUS_LIMIT2 = 0x40
STATUS_HOME_IN_PROG = 0x80

# Stop motor flags
STOP_ABRUPT = 0x01
STOP_SMOOTH = 0x02
MOTOR_OFF = 0x04
AMP_ENABLE = 0x10


# =============================================================================
# LDCNNetwork - Network Manager
# =============================================================================

class LDCNNetwork:
    """
    LDCN Network Manager

    Manages serial communication and network-level LDCN protocol operations.
    Provides device discovery and management.

    The network always initializes at 19200 baud (LDCN default). Use
    set_baud_rate() after initialization to upgrade to higher speeds.

    Attributes:
        port: Serial port path (e.g., '/dev/ttyUSB0')
        baud_rate: Current baud rate
        serial: PySerial Serial object
        devices: List of discovered LDCNDevice objects
        timeout: Serial read timeout in seconds
    """

    def __init__(self, port: str, timeout: float = 0.2):
        """
        Initialize LDCN network manager.

        The network will be opened at 19200 baud (default LDCN reset state).
        Use set_baud_rate() after initialization to upgrade to higher speeds.

        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            timeout: Serial read timeout in seconds (default: 0.2)
        """
        self.port = port
        self.baud_rate = DEFAULT_BAUD
        self.serial: Optional[serial.Serial] = None
        self.devices: List['LDCNDevice'] = []
        self.timeout = timeout
        self._last_addressed_count = 0

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------

    def open(self) -> None:
        """
        Open serial port at 19200 baud (LDCN default).

        All LDCN networks start at 19200 baud after reset.
        Use set_baud_rate() after initialization to upgrade speed.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self._open_port(DEFAULT_BAUD)

    def close(self) -> None:
        """
        Close serial port and cleanup resources.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
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

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if baud not in BAUD_RATES:
            raise ValueError(f"Unsupported baud rate: {baud}")

        # Close existing connection
        if self.serial and self.serial.is_open:
            self.serial.close()
            time.sleep(0.2)

        # Open at specified baud rate
        self.serial = serial.Serial(
            port=self.port,
            baudrate=baud,
            timeout=self.timeout,
            write_timeout=self.timeout
        )
        self.baud_rate = baud
        time.sleep(0.2)  # Let port stabilize

    # -------------------------------------------------------------------------
    # Core Protocol
    # -------------------------------------------------------------------------

    def send_command(self, address: int, command: int, data: List[int] = None) -> bytes:
        """
        Send LDCN command packet and return response.

        This is the SINGLE source of truth for LDCN communication.
        All other send_command() methods delegate to this.

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

        🔴 UNVERIFIED - Not yet tested on hardware
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

        # Send packet
        self.serial.write(packet)
        self.serial.flush()
        time.sleep(DELAY_AFTER_COMMAND)

        # Read response
        response = self.serial.read(50)

        if len(response) < 2:
            # Some commands (like SET_BAUD to group address) don't return responses
            if address == ADDRESS_GROUP or command == CMD_HARD_RESET:
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

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if len(response) < 2:
            return False

        expected = sum(response[:-1]) & 0xFF
        actual = response[-1]
        return expected == actual

    # -------------------------------------------------------------------------
    # Baud Rate Management
    # -------------------------------------------------------------------------

    def _try_baud(self, baud: int) -> bool:
        """
        Test if devices respond at specific baud rate.

        Tries to communicate with common addresses (1, 2, 3, 6) using NOP.

        Args:
            baud: Baud rate to test

        Returns:
            True if any device responds

        🔴 UNVERIFIED - Not yet tested on hardware
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

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if baud_list is None:
            baud_list = COMMON_BAUDS

        for baud in baud_list:
            if self._try_baud(baud):
                return baud

        # Default to 19200 if nothing found
        return DEFAULT_BAUD

    def set_baud_rate(self, baud: int) -> None:
        """
        Upgrade network baud rate for all devices.

        This is called AFTER initialize() to upgrade from 19200 to a higher speed.

        Steps:
        1. Send SET_BAUD command to group address 0xFF
        2. Close serial port
        3. Wait 500ms
        4. Reopen serial port at new baud rate

        Args:
            baud: Target baud rate (must be in BAUD_RATES dict)

        Raises:
            ValueError: If baud rate not supported
            LDCNError: If upgrade fails

        Example:
            network.initialize()  # At 19200 baud
            network.set_baud_rate(125000)  # Upgrade to 125kbps

        🔴 UNVERIFIED - Not yet tested on hardware
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

    # -------------------------------------------------------------------------
    # Network Initialization
    # -------------------------------------------------------------------------

    def reset(self) -> None:
        """
        Send hard reset to all devices.

        Devices return to address 0x00 and 19200 baud.
        Waits 2 seconds after reset for devices to initialize.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if not self.serial or not self.serial.is_open:
            raise LDCNError("Serial port not open")

        # Send hard reset packet
        packet = bytes([HEADER, ADDRESS_GROUP, 0x0F, 0x0E])
        self.serial.write(packet)
        self.serial.flush()

        # Wait for devices to reset
        time.sleep(DELAY_AFTER_RESET)

        # Flush input buffer
        self.serial.reset_input_buffer()

    def address_devices(self, max_devices: int = 127) -> int:
        """
        Sequentially address devices on network.

        Sends SET_ADDRESS to address 0x00 repeatedly until no response.
        Each successful command enables the next device in the daisy chain
        and assigns it the next sequential address (1, 2, 3, ...).

        Args:
            max_devices: Maximum address to try (default 127, safety limit)

        Returns:
            Number of devices successfully addressed

        Example:
            num_found = network.address_devices()
            # Devices are now at addresses 1, 2, 3, ..., num_found

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        addressed = 0

        for addr in range(1, max_devices + 1):
            try:
                # Send SET_ADDRESS to unaddressed device
                response = self.send_command(ADDRESS_UNADDRESSED, CMD_SET_ADDRESS, [addr, ADDRESS_GROUP])

                if len(response) >= 2:
                    addressed += 1
                    time.sleep(DELAY_AFTER_ADDRESS)
                else:
                    # No more devices
                    break

            except (LDCNTimeoutError, LDCNChecksumError):
                # No more devices responding
                break

        self._last_addressed_count = addressed
        return addressed

    def discover_devices(self, start_address: int = 1, end_address: Optional[int] = None) -> List[Dict]:
        """
        Discover device types and versions on the network.

        Queries each address using READ_STATUS (0x3) with status bit 5 (device ID).
        Returns device information without creating device objects.

        Args:
            start_address: First address to query (default: 1)
            end_address: Last address to query (default: last addressed device)

        Returns:
            List of device info dictionaries:
            [
                {
                    'address': int,
                    'device_id': int,      # Device type ID from hardware
                    'version': int,        # Firmware version
                    'responding': bool     # True if device responded
                },
                ...
            ]

        Example:
            devices = network.discover_devices()
            # [{'address': 1, 'device_id': 0x17, 'version': 0x23, 'responding': True},
            #  {'address': 2, 'device_id': 0x17, 'version': 0x23, 'responding': True},
            #  ...]

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if end_address is None:
            end_address = self._last_addressed_count if self._last_addressed_count > 0 else 6

        device_list = []

        for addr in range(start_address, end_address + 1):
            device_info = {
                'address': addr,
                'device_id': DEVICE_ID_UNKNOWN,
                'version': 0,
                'responding': False
            }

            try:
                # Query with device ID bit set
                response = self.send_command(addr, CMD_READ_STATUS, [0x20, 0x00])

                if len(response) >= 4:  # status + 2 bytes device ID + checksum
                    device_info['responding'] = True
                    device_info['device_id'] = response[1]
                    device_info['version'] = response[2]

            except (LDCNTimeoutError, LDCNChecksumError):
                # Device not responding
                pass

            device_list.append(device_info)

        return device_list

    def verify_devices(self, device_list: List[Dict]) -> List[int]:
        """
        Verify devices are still responding.

        Sends NOP command to each device in the list to confirm communication.

        Args:
            device_list: List of device info dicts from discover_devices()

        Returns:
            List of addresses that responded successfully

        Example:
            devices = network.discover_devices()
            responding = network.verify_devices(devices)
            print(f"Responding addresses: {responding}")

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        responding = []

        for device_info in device_list:
            addr = device_info['address']
            try:
                response = self.send_command(addr, CMD_NOP)
                if len(response) >= 2:
                    responding.append(addr)
            except (LDCNTimeoutError, LDCNChecksumError):
                pass

        return responding

    def create_device_objects(self, device_list: List[Dict]) -> List['LDCNDevice']:
        """
        Create device objects from device info list.

        Maps device IDs to appropriate classes (LS231SE, SK2310g2, etc.)
        and populates self.devices list.

        Args:
            device_list: List of device info dicts from discover_devices()

        Returns:
            List of LDCNDevice objects

        Example:
            device_info = network.discover_devices()
            network.create_device_objects(device_info)
            # Now network.devices is populated

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.devices = []

        for device_info in device_list:
            if not device_info['responding']:
                continue

            addr = device_info['address']
            device_id = device_info['device_id']

            # Map device ID to class
            if device_id == DEVICE_ID_LS231SE:
                device = LS231SE(self, addr)
            elif device_id == DEVICE_ID_SK2310G2:
                device = SK2310g2(self, addr)
            else:
                # Unknown device type - create fallback device
                device = UnknownDevice(self, addr)

            device.model_id = device_id
            device.version = device_info['version']
            self.devices.append(device)

        return self.devices

    def initialize(self, create_objects: bool = True) -> Tuple[int, List[Dict]]:
        """
        Complete network initialization sequence at 19200 baud.

        Steps:
        1. Hard reset all devices (at 19200 baud)
        2. Wait 2 seconds
        3. Address devices sequentially (assigns addresses 1, 2, 3...)
        4. Discover device types and versions (queries each device)
        5. Verify all devices are responding
        6. Optionally create device objects and populate self.devices

        This initializes the network at 19200 baud. Use set_baud_rate()
        afterwards to upgrade to a higher speed if desired.

        Args:
            create_objects: If True, create device objects and populate self.devices

        Returns:
            Tuple of (num_devices, device_info_list)
            - num_devices: Number of devices addressed
            - device_info_list: List of device info dicts from discover_devices()

        Raises:
            LDCNInitializationError: If initialization fails

        Example:
            network = LDCNNetwork('/dev/ttyUSB0')
            network.open()
            num_devices, device_info = network.initialize()  # At 19200 baud
            print(f"Found {num_devices} devices:")
            for dev in device_info:
                print(f"  Address {dev['address']}: ID=0x{dev['device_id']:02X}, Version=0x{dev['version']:02X}")
            network.set_baud_rate(125000)  # Upgrade to 125kbps

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        try:
            # Step 1: Hard reset
            self.reset()

            # Step 2: Address devices
            num_devices = self.address_devices()

            if num_devices == 0:
                raise LDCNInitializationError("No devices found during addressing")

            # Step 3: Discover device types
            device_info = self.discover_devices()

            # Step 4: Verify communication
            responding = self.verify_devices(device_info)

            if len(responding) == 0:
                raise LDCNInitializationError("No devices responding after addressing")

            # Step 5: Create device objects if requested
            if create_objects:
                self.create_device_objects(device_info)

            return num_devices, device_info

        except Exception as e:
            raise LDCNInitializationError(f"Initialization failed: {e}") from e

    # -------------------------------------------------------------------------
    # Context Manager Support
    # -------------------------------------------------------------------------

    def __enter__(self) -> 'LDCNNetwork':
        """Enable 'with' statement usage."""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Cleanup on 'with' statement exit."""
        self.close()


# =============================================================================
# LDCNDevice - Base Device Class
# =============================================================================

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

    def __init__(self, network: LDCNNetwork, address: int):
        """
        Initialize base device.

        Args:
            network: Parent LDCNNetwork object
            address: Device address (1-127)

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.network = network
        self.address = address
        self.device_type = "Unknown"
        self.model_id: Optional[int] = None
        self.version: Optional[int] = None

    def send_command(self, command: int, data: List[int] = None) -> bytes:
        """
        Send command to this device.

        Delegates to network.send_command() with this device's address.

        Args:
            command: LDCN command code
            data: Data bytes

        Returns:
            Response bytes from device

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        return self.network.send_command(self.address, command, data)

    def nop(self) -> bytes:
        """
        Send NOP command, return status.

        Returns:
            Raw status response bytes

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        return self.send_command(CMD_NOP)

    def define_status(self, status_bits: int) -> None:
        """
        Configure status reporting (permanent).

        Args:
            status_bits: 16-bit status configuration

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.send_command(CMD_DEFINE_STATUS, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

    @abstractmethod
    def read_status(self) -> Dict:
        """
        Read device status (abstract - implemented by subclasses).

        Returns:
            Device-specific status dictionary
        """
        pass

    def reset_position(self) -> None:
        """
        Reset position counter to zero (if supported).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.send_command(CMD_RESET_POS)

    def __repr__(self) -> str:
        """Return string representation."""
        model = self.model_id if self.model_id is not None else 0
        return f"{self.device_type}(address={self.address}, model_id=0x{model:02X})"


# =============================================================================
# UnknownDevice - Fallback for unrecognized devices
# =============================================================================

class UnknownDevice(LDCNDevice):
    """
    Fallback device class for unrecognized LDCN devices.

    This class provides basic functionality for devices whose type
    is not yet implemented or recognized.
    """

    def __init__(self, network: LDCNNetwork, address: int):
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


# =============================================================================
# LS231SE - Servo Drive
# =============================================================================

class LS231SE(LDCNDevice):
    """
    LS-231SE Servo Drive

    Implements servo-specific operations including motion control, status
    reading, and PID gain configuration.

    Additional Attributes:
        position: Last known position (encoder counts)
        velocity: Last known velocity
        status_byte: Last status byte
        aux_status: Last auxiliary status
        pos_error: Last position error
        kp, kd, ki: PID gains
    """

    def __init__(self, network: LDCNNetwork, address: int):
        """
        Initialize servo drive.

        Args:
            network: Parent LDCNNetwork object
            address: Device address (1-127)

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        super().__init__(network, address)
        self.device_type = "LS-231SE"

        # Cached status values
        self.position: Optional[int] = None
        self.velocity: Optional[int] = None
        self.status_byte: Optional[int] = None
        self.aux_status: Optional[int] = None
        self.pos_error: Optional[int] = None

        # PID gains
        self.kp: Optional[int] = None
        self.kd: Optional[int] = None
        self.ki: Optional[int] = None

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def initialize(self,
                   kp: int = 2, kd: int = 50, ki: int = 0,
                   il: int = 40, ol: int = 255, cl: int = 0,
                   el: int = 2000, sr: int = 20, db: int = 0) -> bool:
        """
        Complete 7-step servo initialization.

        Steps:
        1. Define status reporting (pos, vel, aux, pos_err)
        2. Set PID gains
        3. Load initial trajectory (position 0)
        4. Enable amplifier
        5. Reset position counter
        6. Clear sticky status bits
        7. Read and verify status

        Args:
            kp, kd, ki: PID gains
            il: Integration limit
            ol: Output limit
            cl: Current limit (0 = disabled)
            el: Position error limit (encoder counts)
            sr: Servo rate divisor
            db: Deadband

        Returns:
            True if initialization successful

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        try:
            # Step 1: Define status reporting
            status_bits = STATUS_BIT_POSITION | STATUS_BIT_VELOCITY | STATUS_BIT_AUX | STATUS_BIT_POS_ERROR
            self.define_status(status_bits)
            time.sleep(0.1)

            # Step 2: Set PID gains
            self.set_gains(kp, kd, ki, il, ol, cl, el, sr, db)
            time.sleep(0.1)

            # Step 3: Load initial trajectory (position 0)
            traj_ctrl = 0x10  # servo_mode=1
            traj_data = [traj_ctrl, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0]  # pos=0, vel=0, accel=100
            self.send_command(CMD_LOAD_TRAJECTORY, traj_data)
            time.sleep(0.1)

            # Step 4: Enable amplifier
            self.enable()
            time.sleep(0.1)

            # Step 5: Reset position counter
            self.reset_position()
            time.sleep(0.1)

            # Step 6: Clear sticky status bits
            self.clear_faults()
            time.sleep(0.1)

            # Step 7: Read and verify status
            status = self.read_status()

            # Check for faults
            faults = self.check_faults(status['status'])
            if faults:
                return False

            return True

        except Exception:
            return False

    # -------------------------------------------------------------------------
    # Status Reading
    # -------------------------------------------------------------------------

    def read_status(self) -> Dict:
        """
        Read complete servo status.

        Returns:
            {
                'status': status_byte,
                'position': position_counts,
                'velocity': velocity,
                'aux_status': aux_byte,
                'pos_error': following_error,
                'flags': {
                    'move_done': bool,
                    'cksum_error': bool,
                    'current_limit': bool,
                    'power_on': bool,
                    'pos_error': bool,
                    ...
                }
            }

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        status_bits = STATUS_BIT_POSITION | STATUS_BIT_VELOCITY | STATUS_BIT_AUX | STATUS_BIT_POS_ERROR
        response = self.send_command(CMD_READ_STATUS, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

        return self._parse_status(response, status_bits)

    def read_position(self) -> Dict:
        """
        Read position only (fast status read).

        Returns:
            {'position': position_counts, 'status': status_byte}

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        response = self.send_command(CMD_READ_STATUS, [STATUS_BIT_POSITION & 0xFF, 0x00])

        if len(response) < 6:  # status + 4 bytes position + checksum
            return {'position': None, 'status': None}

        status_byte = response[0]
        position = struct.unpack('<i', bytes(response[1:5]))[0]

        self.status_byte = status_byte
        self.position = position

        return {'position': position, 'status': status_byte}

    def _parse_status(self, response: bytes, status_bits: int) -> Dict:
        """
        Parse variable-length servo status response.

        Args:
            response: Raw response bytes
            status_bits: Status bits used in query

        Returns:
            Parsed status dictionary

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if len(response) < 2:
            return {}

        status_byte = response[0]
        data = response[1:-1]  # Everything except first byte and checksum

        result = {
            'status': status_byte,
            'flags': self.decode_status_flags(status_byte)
        }

        idx = 0

        # Position (4 bytes)
        if status_bits & STATUS_BIT_POSITION and len(data) >= idx + 4:
            result['position'] = struct.unpack('<i', bytes(data[idx:idx+4]))[0]
            self.position = result['position']
            idx += 4

        # A/D value (1 byte)
        if status_bits & STATUS_BIT_AD_VALUE and len(data) >= idx + 1:
            result['ad_value'] = data[idx]
            idx += 1

        # Velocity (2 bytes)
        if status_bits & STATUS_BIT_VELOCITY and len(data) >= idx + 2:
            result['velocity'] = struct.unpack('<h', bytes(data[idx:idx+2]))[0]
            self.velocity = result['velocity']
            idx += 2

        # Auxiliary status (1 byte)
        if status_bits & STATUS_BIT_AUX and len(data) >= idx + 1:
            aux = data[idx]
            result['aux_status'] = aux
            result['servo_on'] = bool(aux & 0x04)
            result['servo_overrun'] = bool(aux & 0x20)
            self.aux_status = aux
            idx += 1

        # Position error (2 bytes)
        if status_bits & STATUS_BIT_POS_ERROR and len(data) >= idx + 2:
            result['pos_error'] = struct.unpack('<h', bytes(data[idx:idx+2]))[0]
            self.pos_error = result['pos_error']
            idx += 2

        self.status_byte = status_byte

        return result

    def decode_status_flags(self, status_byte: int) -> Dict[str, bool]:
        """
        Decode status byte into flag dictionary.

        Args:
            status_byte: Status byte from response

        Returns:
            Dictionary of flag names to boolean values

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        return {
            'move_done': bool(status_byte & STATUS_MOVE_DONE),
            'cksum_error': bool(status_byte & STATUS_CKSUM_ERROR),
            'current_limit': bool(status_byte & STATUS_CURRENT_LIMIT),
            'power_on': bool(status_byte & STATUS_POWER_ON),
            'pos_error': bool(status_byte & STATUS_POS_ERROR),
            'home_source': bool(status_byte & STATUS_HOME_SOURCE),
            'limit2': bool(status_byte & STATUS_LIMIT2),
            'home_in_progress': bool(status_byte & STATUS_HOME_IN_PROG),
        }

    def check_faults(self, status_byte: int) -> List[str]:
        """
        Check status byte for fault conditions.

        Args:
            status_byte: Status byte to check

        Returns:
            List of active fault names

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        faults = []

        if status_byte & STATUS_CKSUM_ERROR:
            faults.append('cksum_error')
        if status_byte & STATUS_CURRENT_LIMIT:
            faults.append('current_limit')
        if status_byte & STATUS_POS_ERROR:
            faults.append('pos_error')

        return faults

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------

    def set_gains(self,
                  kp: int, kd: int, ki: int,
                  il: int, ol: int, cl: int,
                  el: int, sr: int, db: int) -> None:
        """
        Set PID gains (LOAD_GAINS command).

        Args:
            kp: Proportional gain
            kd: Derivative gain
            ki: Integral gain
            il: Integration limit
            ol: Output limit
            cl: Current limit
            el: Position error limit
            sr: Servo rate divisor
            db: Deadband

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        gain_data = struct.pack('<HHHBBBHB', kp, kd, ki, il, ol, cl, el, sr)
        self.send_command(CMD_LOAD_GAINS, list(gain_data) + [db])

        # Cache gains
        self.kp = kp
        self.kd = kd
        self.ki = ki

    def configure_status(self, status_bits: int) -> None:
        """
        Configure status reporting.

        Args:
            status_bits: 16-bit status configuration

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.define_status(status_bits)

    # -------------------------------------------------------------------------
    # Motion Control
    # -------------------------------------------------------------------------

    def move_to(self, position: float, velocity: float, accel: float, scale: float = 1.0) -> None:
        """
        Command motion to absolute position.

        Args:
            position: Target position (physical units, e.g., mm)
            velocity: Velocity (physical units/sec)
            accel: Acceleration (physical units/sec²)
            scale: Counts per physical unit (e.g., 2000 counts/mm)

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        position_counts = int(position * scale)
        velocity_counts = int(velocity * scale)
        accel_counts = int(accel * scale)

        self.move_to_counts(position_counts, velocity_counts, accel_counts)

    def move_to_counts(self, position: int, velocity: int, accel: int) -> None:
        """
        Command motion to absolute position (raw counts).

        Args:
            position: Target position (encoder counts)
            velocity: Velocity (counts per servo tick)
            accel: Acceleration (counts per tick²)

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        traj_ctrl = 0x80 | 0x10  # start_now=1, servo_mode=1
        traj_data = struct.pack('<Biii', traj_ctrl, position, velocity, accel)
        self.send_command(CMD_LOAD_TRAJECTORY, list(traj_data))

    # -------------------------------------------------------------------------
    # Amplifier Control
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """
        Enable amplifier (STOP_MOTOR with AMP_ENABLE flag).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        stop_ctrl = STOP_ABRUPT | AMP_ENABLE
        self.send_command(CMD_STOP_MOTOR, [stop_ctrl])

    def disable(self) -> None:
        """
        Disable amplifier and position servo.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.send_command(CMD_STOP_MOTOR, [0x00])

    # -------------------------------------------------------------------------
    # Fault Management
    # -------------------------------------------------------------------------

    def clear_faults(self) -> None:
        """
        Clear sticky status bits (CLEAR_BITS command).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.send_command(CMD_CLEAR_BITS)


# =============================================================================
# SK2310g2 - I/O Controller
# =============================================================================

class SK2310g2(LDCNDevice):
    """
    SK-2310g2 I/O Controller

    Generic LDCN I/O controller device used in this application as a
    supervisory controller with safety and spindle control functions.

    Hardware Capabilities:
    - Dual mechanical relay power control
    - Spindle control with spindle enable mechanical relay
    - Dual line emergency stop monitoring
    - Dual work zone "covers" contacts (guarded area monitoring)
    - Dual safe zone sensor interface
    - 3 analog inputs
    - 1 analog output (CN6.11 is 0-10V spindle speed control)
    - Digital I/O (16 inputs, 16 outputs)

    Additional Attributes:
        diagnostic_code: Last diagnostic code (LED display)
        power_state: Power button state
        estop_state: Emergency stop state
        digital_inputs: 16-bit digital input state
        digital_outputs: 16-bit digital output state
    """

    def __init__(self, network: LDCNNetwork, address: int):
        """
        Initialize I/O controller.

        Args:
            network: Parent LDCNNetwork object
            address: Device address (1-127)

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        super().__init__(network, address)
        self.device_type = "SK-2310g2"

        # Cached status values
        self.diagnostic_code: Optional[int] = None
        self.status_byte: Optional[int] = None
        self.power_state: Optional[bool] = None
        self.estop_state: Optional[bool] = None
        self.digital_inputs: Optional[int] = None
        self.digital_outputs: Optional[int] = None

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------

    def configure(self) -> None:
        """
        Configure I/O controller for full status reporting.

        Sends DEFINE_STATUS with 0xFFFF (all status data).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.define_status(0xFFFF)
        time.sleep(1.0)

    # -------------------------------------------------------------------------
    # Status Reading
    # -------------------------------------------------------------------------

    def read_status(self) -> Dict:
        """
        Read complete I/O controller status.

        Returns:
            {
                'status': status_byte,
                'diagnostic': diagnostic_code,
                'power_state': bool,
                'digital_inputs': int (16-bit),
                ...
            }

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        # Configure for full status if not already done
        response = self.send_command(CMD_READ_STATUS, [0xFF, 0xFF])

        if len(response) < 3:
            return {}

        # For I/O controller with full status, status byte is at index 1
        diagnostic = response[0]
        status_byte = response[1]

        self.diagnostic_code = diagnostic
        self.status_byte = status_byte
        self.power_state = bool(status_byte & STATUS_POWER_ON)

        result = {
            'status': status_byte,
            'diagnostic': diagnostic,
            'power_state': self.power_state,
        }

        # Parse additional data if available
        # (Implementation depends on actual response format - TBD from hardware)

        return result

    def read_diagnostic(self) -> int:
        """
        Read diagnostic code (LED display value).

        Returns:
            Diagnostic code (0x00-0xFF)

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        status = self.read_status()
        return status.get('diagnostic', 0)

    def read_power_state(self) -> bool:
        """
        Read power button state from status bit 3.

        Returns:
            True if power ON, False if power OFF

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        status = self.read_status()
        return status.get('power_state', False)

    # -------------------------------------------------------------------------
    # Power and Safety Monitoring
    # -------------------------------------------------------------------------

    def wait_for_power_button(self, timeout: Optional[float] = None, poll_rate: float = 0.1) -> bool:
        """
        Wait for power button press detection.

        Continuously monitors power state until transition from OFF to ON.

        Args:
            timeout: Maximum wait time (None = infinite)
            poll_rate: Status polling rate (seconds)

        Returns:
            True if power button pressed, False if timeout

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        start_time = time.time()

        while True:
            power_state = self.read_power_state()

            if power_state:
                return True

            if timeout is not None:
                elapsed = time.time() - start_time
                if elapsed >= timeout:
                    return False

            time.sleep(poll_rate)

    def read_estop_state(self) -> bool:
        """
        Read emergency stop state (dual line monitoring).

        Returns:
            True if E-stop is OK, False if E-stop is active

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD - depends on I/O channel mapping
        """
        # TBD: Read appropriate digital inputs
        raise NotImplementedError("E-stop monitoring not yet implemented - I/O mapping TBD")

    def read_cover_state(self) -> bool:
        """
        Read work zone cover state (guarded area contacts).

        Returns:
            True if covers closed (safe), False if any cover open

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD - depends on I/O channel mapping
        """
        # TBD: Read appropriate digital inputs
        raise NotImplementedError("Cover monitoring not yet implemented - I/O mapping TBD")

    def read_safe_zone_state(self) -> bool:
        """
        Read safe zone sensor state (dual sensor interface).

        Returns:
            True if safe zone clear, False if zone occupied

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD - depends on I/O channel mapping
        """
        # TBD: Read appropriate digital inputs
        raise NotImplementedError("Safe zone monitoring not yet implemented - I/O mapping TBD")

    # -------------------------------------------------------------------------
    # Digital I/O (Stubs - Implementation TBD)
    # -------------------------------------------------------------------------

    def read_digital_inputs(self) -> int:
        """
        Read all digital input states.

        Returns:
            16-bit digital input value

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Digital I/O not yet implemented")

    def set_digital_outputs(self, outputs: int) -> None:
        """
        Set all digital output states.

        Args:
            outputs: 16-bit digital output value

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Digital I/O not yet implemented")

    # -------------------------------------------------------------------------
    # Analog I/O (Stubs - Implementation TBD)
    # -------------------------------------------------------------------------

    def read_analog_inputs(self) -> Dict[int, int]:
        """
        Read all analog input values (3 channels).

        Returns:
            Dictionary of {channel: value} pairs

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Analog I/O not yet implemented")

    def set_analog_output(self, voltage: float) -> None:
        """
        Set analog output voltage (CN6.11 spindle speed control).

        Args:
            voltage: Output voltage (0.0 - 10.0V)

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Analog output not yet implemented")

    # -------------------------------------------------------------------------
    # Spindle Control (Stubs - Implementation TBD)
    # -------------------------------------------------------------------------

    def set_spindle_speed(self, speed_percent: float) -> None:
        """
        Set spindle speed via analog output.

        Args:
            speed_percent: Speed as percentage (0.0 - 100.0)

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        voltage = (speed_percent / 100.0) * 10.0
        self.set_analog_output(voltage)

    def enable_spindle(self) -> None:
        """
        Enable spindle via mechanical relay.

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Spindle control not yet implemented")

    def disable_spindle(self) -> None:
        """
        Disable spindle via mechanical relay.

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Spindle control not yet implemented")


# =============================================================================
# Module Test / Example
# =============================================================================

if __name__ == '__main__':
    """
    Simple test/example of module usage.
    Run with: python3 ldcn_network.py
    """
    import sys

    print("="*70)
    print("LDCN Network Module Test")
    print("="*70)
    print()

    PORT = '/dev/ttyUSB0'

    try:
        # Initialize network
        with LDCNNetwork(PORT) as network:
            print(f"Opening {PORT} at 19200 baud...")

            # Initialize
            print("\nInitializing network...")
            num_devices, device_info = network.initialize()

            print(f"\n✓ Found {num_devices} devices:")
            for dev in device_info:
                print(f"  Address {dev['address']}: ID=0x{dev['device_id']:02X}, Version=0x{dev['version']:02X}, Responding={dev['responding']}")

            # Upgrade baud rate
            print("\nUpgrading to 125000 baud...")
            network.set_baud_rate(125000)
            print("✓ Baud rate upgraded")

            # Show device objects
            print(f"\n✓ Created {len(network.devices)} device objects:")
            for device in network.devices:
                print(f"  {device}")

            print("\n✓ Network ready!")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    print("\n✓ Test complete!")
    sys.exit(0)
