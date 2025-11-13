#!/usr/bin/env python3
"""
LDCN Network Manager

High-level network orchestration for Logosol LDCN (Logosol Distributed Control
Network) devices. Coordinates protocol layer, device discovery, and device
management.

Modular Architecture:
    - protocol.py: Low-level serial communication and LDCN protocol
    - discovery.py: Device discovery, addressing, and verification
    - device.py: Base device classes (LDCNDevice, UnknownDevice)
    - devices/: Device-specific implementations (LS231SE, SK2310g2)
    - network.py: High-level orchestration (this file)

Usage Example:
    from pyldcn import LDCNNetwork, InitMode

    with LDCNNetwork('/dev/ttyUSB0') as network:
        num_devices, device_info = network.initialize()
        network.set_baud_rate(125000)

        servo = network.devices[0]  # LS231SE at address 1
        servo.initialize()
        servo.move_to(position=10.0, velocity=100.0, accel=50.0, scale=2000.0)

Author: NickyDoes
License: GPL v2 or later
Date: 2025-10-29
"""

import time
from typing import Optional, List, Dict, Tuple

from .protocol import LDCNProtocol
from .discovery import DeviceDiscovery
from .device import LDCNDevice, UnknownDevice
from .exceptions import (
    LDCNError,
    LDCNTimeoutError,
    LDCNChecksumError,
    LDCNDetectionError,
    LDCNInitializationError,
)
from .constants import (
    InitMode,
    HEADER,
    ADDRESS_GROUP,
    CMD_HARD_RESET,
    DEFAULT_BAUD,
    DELAY_AFTER_RESET,
)
from . import util


class LDCNNetwork:
    """
    LDCN Network Manager

    High-level network orchestration that coordinates protocol layer and
    device discovery. Manages device list and provides initialization modes.

    The network always initializes at 19200 baud (LDCN default). Use
    set_baud_rate() after initialization to upgrade to higher speeds.

    Attributes:
        port: Serial port path (e.g., '/dev/ttyUSB0')
        baud_rate: Current baud rate (delegated to protocol)
        devices: List of discovered LDCNDevice objects
        protocol: LDCNProtocol instance for low-level communication
        discovery: DeviceDiscovery instance for device operations
    """

    def __init__(self, port: str, timeout: float = 0.015):
        """
        Initialize LDCN network manager.

        Creates protocol and discovery layers. The network will be opened
        at 19200 baud (default LDCN reset state). Upgrade to higher speeds
        with set_baud_rate().

        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            timeout: Serial read timeout in seconds (default 0.015s/15ms)
        """
        # Create protocol and discovery layers
        self.protocol = LDCNProtocol(port, timeout)
        self.discovery = DeviceDiscovery(self.protocol)

        # Device management
        self.devices: List[LDCNDevice] = []

        # Expose commonly used attributes for convenience
        self.port = port
        self.timeout = timeout

    @property
    def baud_rate(self) -> int:
        """Current baud rate (from protocol layer)."""
        return self.protocol.baud_rate

    @property
    def serial(self):
        """Serial port object (from protocol layer)."""
        return self.protocol.serial

    # -------------------------------------------------------------------------
    # Connection Management (delegate to protocol)
    # -------------------------------------------------------------------------

    def open(self) -> None:
        """
        Open serial port at 19200 baud (LDCN default).

        All LDCN networks start at 19200 baud after reset.
        Use set_baud_rate() after initialization to upgrade speed.
        """
        self.protocol.open(DEFAULT_BAUD)

    def close(self) -> None:
        """Close serial port and cleanup resources."""
        self.protocol.close()

    # -------------------------------------------------------------------------
    # Core Protocol (delegate to protocol)
    # -------------------------------------------------------------------------

    def send_command(self, address: int, command: int, data: Optional[List[int]] = None) -> bytes:
        """
        Send LDCN command packet and return response.

        Delegates to protocol layer for low-level communication.

        Args:
            address: Device address (1-127) or group (128-255)
            command: LDCN command (0x00-0x0F)
            data: Data bytes (0-16 bytes)

        Returns:
            Response bytes from device

        Raises:
            LDCNTimeoutError: No response received
            LDCNChecksumError: Response checksum mismatch
        """
        return self.protocol.send_command(address, command, data)

    # -------------------------------------------------------------------------
    # Baud Rate Management (delegate to protocol)
    # -------------------------------------------------------------------------

    def set_baud_rate(self, baud: int) -> None:
        """
        Upgrade network baud rate for all devices.

        Called AFTER initialize() to upgrade from 19200 to higher speed.

        Args:
            baud: Target baud rate (must be in BAUD_RATES dict)

        Raises:
            ValueError: If baud rate not supported

        Example:
            network.initialize()  # At 19200 baud
            network.set_baud_rate(125000)  # Upgrade to 125kbps
        """
        self.protocol.set_baud_rate(baud)

    def auto_detect_baud(self, baud_list: Optional[List[int]] = None) -> int:
        """
        Auto-detect current network baud rate.

        Tries common baud rates until a device responds.

        Args:
            baud_list: List of baud rates to try (default: COMMON_BAUDS)

        Returns:
            Detected baud rate

        Raises:
            LDCNDetectionError: No response at any baud rate
        """
        return self.protocol.auto_detect_baud(baud_list)

    # -------------------------------------------------------------------------
    # Device Discovery (delegate to discovery)
    # -------------------------------------------------------------------------

    def reset(self) -> None:
        """
        Send hard reset to all devices at all possible baud rates.

        Tries sending reset at every known baud rate to ensure devices
        are reset regardless of their current baud rate. After reset,
        devices return to address 0x00 and 19200 baud.
        """
        self.discovery.reset()

    def address_devices(self, max_devices: int = 127) -> int:
        """
        Sequentially address devices on network.

        Sends SET_ADDRESS to address 0x00 repeatedly until no response.
        Each successful command assigns the next sequential address.

        Args:
            max_devices: Maximum address to try (default 127)

        Returns:
            Number of devices successfully addressed
        """
        return self.discovery.address_devices(max_devices)

    def discover_devices(
        self,
        start_address: int = 1,
        end_address: Optional[int] = None,
        early_exit_threshold: int = 3
    ) -> List[Dict]:
        """
        Discover device types and versions on the network.

        Queries each address using READ_STATUS with device ID bit set.

        Args:
            start_address: First address to query (default: 1)
            end_address: Last address to query (default: last addressed, or 127)
            early_exit_threshold: Stop after N consecutive non-responding addresses

        Returns:
            List of device info dictionaries
        """
        return self.discovery.discover_devices(start_address, end_address, early_exit_threshold)

    def validate_devices(self, expected_devices: Optional[List[Dict]] = None) -> bool:
        """
        Validate existing device objects respond correctly (InitMode.VALIDATE).

        Fast validation that checks if cached device objects still respond
        at the current baud rate without making any state changes.

        Args:
            expected_devices: Optional list of expected device info dicts

        Returns:
            True if all devices respond correctly
        """
        return self.discovery.validate_devices(self.devices, expected_devices)

    def verify_devices(self, device_list: List[Dict]) -> List[int]:
        """
        Verify devices are still responding.

        Sends NOP command to each device to confirm communication.

        Args:
            device_list: List of device info dicts from discover_devices()

        Returns:
            List of addresses that responded successfully
        """
        return self.discovery.verify_devices(device_list)

    def create_device_objects(self, device_list: List[Dict]) -> List[LDCNDevice]:
        """
        Create device objects from device info list.

        Maps device IDs to appropriate classes and populates self.devices list.

        Args:
            device_list: List of device info dicts from discover_devices()

        Returns:
            List of LDCNDevice objects
        """
        self.devices = self.discovery.create_device_objects(device_list, self)
        return self.devices

    # -------------------------------------------------------------------------
    # Device List Management
    # -------------------------------------------------------------------------

    def save_device_list(self, filename: str) -> None:
        """
        Save discovered device list to JSON file.

        Saves hardware facts from network discovery only.
        Does not include axis configuration.

        Args:
            filename: Path to save file (e.g., 'device_list.json')
        """
        util.save_device_list(self.devices, self.port, self.baud_rate, filename)

    def load_device_list(self, filename: str) -> List[Dict]:
        """
        Load device list from JSON file.

        Validates file format and returns device information.
        Does not create device objects or open serial port.

        Args:
            filename: Path to device list file

        Returns:
            List of device info dictionaries

        Raises:
            LDCNError: If file format invalid or unsupported version
        """
        return util.load_device_list(filename)

    # -------------------------------------------------------------------------
    # High-Level Initialization
    # -------------------------------------------------------------------------

    def soft_initialize(
        self,
        create_objects: bool = True,
        baud_list: Optional[List[int]] = None
    ) -> Tuple[int, List[Dict]]:
        """
        Soft initialization without reset (InitMode.SOFT).

        Discovers devices at their current addresses and baud rate without
        performing a hard reset. This preserves device state including:
        - Servo positions and gains
        - Device configurations
        - Status reporting settings

        Steps:
        1. Auto-detect current baud rate
        2. Scan addresses for responding devices
        3. Query device types and versions
        4. Verify communication
        5. Optionally create device objects

        Args:
            create_objects: If True, create device objects and populate self.devices
            baud_list: List of baud rates to try (default: COMMON_BAUDS)

        Returns:
            Tuple of (num_devices, device_info_list)

        Raises:
            LDCNInitializationError: If no devices found or baud detection fails
        """
        try:
            # Auto-detect baud rate
            detected_baud = self.auto_detect_baud(baud_list)

            # Discover devices at current addresses (no reset)
            device_info = self.discover_devices(start_address=1)

            # Filter to only responding devices
            responding_devices = [d for d in device_info if d['responding']]

            if len(responding_devices) == 0:
                raise LDCNInitializationError("No devices found during soft discovery")

            # Create device objects if requested
            if create_objects:
                self.create_device_objects(responding_devices)

            return len(responding_devices), responding_devices

        except LDCNDetectionError as e:
            raise LDCNInitializationError(f"Soft initialization failed: {e}") from e
        except Exception as e:
            raise LDCNInitializationError(f"Soft initialization failed: {e}") from e

    def initialize(
        self,
        mode: InitMode = InitMode.FULL,
        create_objects: bool = True,
        expected_devices: Optional[List[Dict]] = None
    ) -> Tuple[int, List[Dict]]:
        """
        Adaptive network initialization with multiple modes.

        Supports multiple initialization strategies from fast validation to
        full hard reset. Default mode is FULL for backwards compatibility.

        Initialization Modes:
        - VALIDATE: Fast validation (~100ms) - verify existing connections
        - SOFT: Soft discovery (~500ms) - preserve device state
        - READDRESS: Reset and re-address (~1s) - current baud only
        - FULL: Full reset (~2s+) - reset at all bauds (default)
        - AUTO: Adaptive - tries VALIDATE -> SOFT -> READDRESS -> FULL

        Args:
            mode: Initialization mode (default: InitMode.FULL)
            create_objects: If True, create device objects and populate self.devices
            expected_devices: Optional list for VALIDATE/AUTO modes

        Returns:
            Tuple of (num_devices, device_info_list)

        Raises:
            LDCNInitializationError: If initialization fails

        Examples:
            # Default: Full reset (backwards compatible)
            network.initialize()

            # Fast validation of existing connection
            network.initialize(mode=InitMode.VALIDATE)

            # Soft discovery (preserves servo positions/gains)
            network.initialize(mode=InitMode.SOFT)

            # Automatic adaptive initialization
            network.initialize(mode=InitMode.AUTO)
        """
        # Handle AUTO mode - try progressively more invasive approaches
        if mode == InitMode.AUTO:
            # Level 0: Try VALIDATE if devices exist and expected list provided
            if self.devices and expected_devices:
                if self.validate_devices(expected_devices):
                    # Validation successful - network is healthy
                    num_devices = len(self.devices)
                    device_info = [
                        {
                            'address': dev.address,
                            'device_id': dev.model_id if dev.model_id else 0xFF,
                            'version': dev.version if dev.version else 0x00,
                            'responding': True
                        }
                        for dev in self.devices
                    ]
                    return num_devices, device_info

            # Level 1: Try SOFT initialization
            try:
                return self.soft_initialize(create_objects=create_objects)
            except LDCNInitializationError:
                pass  # Fall through to next level

            # Level 2: Try READDRESS (reset at detected baud only)
            try:
                detected_baud = self.auto_detect_baud()
                self.protocol._open_port(detected_baud)

                # Reset at detected baud only
                packet = bytes([HEADER, ADDRESS_GROUP, CMD_HARD_RESET,
                               CMD_HARD_RESET ^ ADDRESS_GROUP ^ HEADER])
                assert self.protocol.serial is not None
                self.protocol.serial.write(packet)
                self.protocol.serial.flush()
                time.sleep(DELAY_AFTER_RESET)

                # Re-address and discover
                self.protocol._open_port(DEFAULT_BAUD)
                num_devices = self.address_devices()
                if num_devices > 0:
                    device_info = self.discover_devices()
                    if create_objects:
                        self.create_device_objects(device_info)
                    return num_devices, device_info
            except (LDCNDetectionError, LDCNInitializationError):
                pass  # Fall through to FULL

            # Level 3: Fallback to FULL reset
            mode = InitMode.FULL

        # Handle explicit modes
        if mode == InitMode.VALIDATE:
            if self.validate_devices(expected_devices):
                num_devices = len(self.devices)
                device_info = [
                    {
                        'address': dev.address,
                        'device_id': dev.model_id if dev.model_id else 0xFF,
                        'version': dev.version if dev.version else 0x00,
                        'responding': True
                    }
                    for dev in self.devices
                ]
                return num_devices, device_info
            else:
                raise LDCNInitializationError("Validation failed")

        elif mode == InitMode.SOFT:
            return self.soft_initialize(create_objects=create_objects)

        elif mode == InitMode.READDRESS:
            # Detect baud, reset at that baud, re-address
            try:
                detected_baud = self.auto_detect_baud()
                self.protocol._open_port(detected_baud)

                # Reset at detected baud only
                packet = bytes([HEADER, ADDRESS_GROUP, CMD_HARD_RESET,
                               CMD_HARD_RESET ^ ADDRESS_GROUP ^ HEADER])
                assert self.protocol.serial is not None
                self.protocol.serial.write(packet)
                self.protocol.serial.flush()
                time.sleep(DELAY_AFTER_RESET)

                # Re-address and discover at default baud
                self.protocol._open_port(DEFAULT_BAUD)
                num_devices = self.address_devices()
                if num_devices == 0:
                    raise LDCNInitializationError("No devices found during re-addressing")

                device_info = self.discover_devices()
                responding = self.verify_devices(device_info)
                if len(responding) == 0:
                    raise LDCNInitializationError("No devices responding after re-addressing")

                if create_objects:
                    self.create_device_objects(device_info)

                return num_devices, device_info

            except LDCNDetectionError as e:
                raise LDCNInitializationError(f"Re-addressing failed: {e}") from e

        elif mode == InitMode.FULL:
            # Full reset at all baud rates (original behavior)
            try:
                # Step 1: Hard reset at all bauds
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
                raise LDCNInitializationError(f"Full initialization failed: {e}") from e

        else:
            raise ValueError(f"Unknown initialization mode: {mode}")

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


# Re-export device classes for backwards compatibility
__all__ = [
    'LDCNNetwork',
    'LDCNDevice',
    'UnknownDevice',
]
