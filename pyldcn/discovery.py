"""
LDCN Device Discovery

Handles LDCN device discovery, addressing, and verification operations.

Author: NickyDoes
License: GPL v2 or later
"""

import time
from enum import Enum
from typing import Optional, List, Dict, TYPE_CHECKING

from .protocol import (
    LDCNProtocol,
    ADDRESS_UNADDRESSED,
    ADDRESS_GROUP,
    CMD_SET_ADDRESS,
    CMD_READ_STATUS,
    CMD_NOP,
    CMD_HARD_RESET,
    DEFAULT_BAUD,
    DELAY_AFTER_RESET,
    DELAY_AFTER_ADDRESS,
)
from .exceptions import LDCNTimeoutError, LDCNChecksumError

if TYPE_CHECKING:
    from .device import LDCNDevice
    from .network import LDCNNetwork


# =============================================================================
# Initialization Modes
# =============================================================================

class InitMode(Enum):
    """
    Initialization modes in order of invasiveness.

    Each mode represents a different level of network initialization,
    from simple validation to full hard reset.
    """

    VALIDATE = 0
    """
    Validation only
    - Verify existing device objects respond at current baud rate
    - Check device IDs match expected types
    - No state changes, no reset, no re-addressing
    - Use when: Reconnecting to a known healthy network
    """

    SOFT = 1
    """
    Soft recovery (~500ms)
    - Auto-detect current baud rate
    - Discover devices at current addresses
    - Create/update device objects
    - No reset or re-addressing
    - Preserves: Device state, positions, gains, configurations
    - Use when: Network may have changed, but devices are at correct addresses
    """

    READDRESS = 2
    """
    Re-addressing (~1s, node quantity dependent)
    - Detect current baud rate
    - Hard reset at detected baud only
    - Re-address devices sequentially (1, 2, 3, ...)
    - Full discovery
    - Loses: Device state, positions, gains
    - Use when: Addressing is corrupted but baud rate is known
    """

    FULL = 3
    """
    Full reset (~2s+, node quantity dependent)
    - Reset at ALL baud rates (230400, 125000, 57600, 38400, 19200, 9600)
    - Re-address devices from scratch
    - Full discovery
    - Loses: Everything (state, positions, gains)
    - Use when: Network state is completely unknown or corrupted
    - Default: Backwards compatible with existing behavior
    """

    AUTO = 4
    """
    Level 4: Automatic mode selection (adaptive)
    - Tries progressively more invasive approaches:
      1. VALIDATE (if expected_devices provided)
      2. SOFT (if baud can be detected)
      3. READDRESS (if soft discovery fails)
      4. FULL (last resort fallback)
    - Use when: Want intelligent recovery with minimal disruption
    """


# =============================================================================
# Device Discovery Constants
# =============================================================================

# Device discovery status bit - Universal across ALL LDCN devices
STATUS_BIT_DEVICE_ID = 0x0020  # Bit 5: Device ID and version (2 bytes)

# Device IDs (hardware-reported, verified from real hardware)
DEVICE_ID_UNKNOWN = 0xFF    # Placeholder for truly unknown devices
DEVICE_ID_LS231SE = 0x00    # ✅ VERIFIED on hardware: Version 0x15
DEVICE_ID_SK2310G2 = 0x02   # ✅ VERIFIED on hardware: Version 0x34


class DeviceDiscovery:
    """
    Handles LDCN device discovery and addressing operations.

    Works with a protocol layer to perform network-level device operations
    including reset, addressing, discovery, and verification.

    Attributes:
        protocol: LDCNProtocol instance for communication
        _last_addressed_count: Number of devices addressed in last operation
    """

    def __init__(self, protocol: LDCNProtocol):
        """
        Initialize device discovery handler.

        Args:
            protocol: LDCNProtocol instance for communication
        """
        self.protocol = protocol
        self._last_addressed_count = 0

    # -------------------------------------------------------------------------
    # Network Initialization
    # -------------------------------------------------------------------------

    def reset(self) -> None:
        """
        Send hard reset to all devices at all possible baud rates.

        Tries sending the reset command at every known baud rate to ensure
        devices are reset regardless of their current baud rate.
        After reset, devices return to address 0x00 and 19200 baud.
        Waits 2 seconds after final reset for devices to initialize.
        """
        # Reset packet
        packet = bytes([HEADER, ADDRESS_GROUP, 0x0F, 0x0E])

        # Try sending reset at all known baud rates
        for baud in [230400, 125000, 57600, 38400, 19200, 9600]:
            try:
                self.protocol._open_port(baud)
                assert self.protocol.serial is not None
                self.protocol.serial.write(packet)
                self.protocol.serial.flush()
                time.sleep(0.05)
            except Exception:
                continue

        # Wait for devices to complete reset
        time.sleep(DELAY_AFTER_RESET)

        # Devices are now at 19200 baud - reopen port
        self.protocol._open_port(DEFAULT_BAUD)

        # Flush input buffer
        assert self.protocol.serial is not None
        self.protocol.serial.reset_input_buffer()

    def address_devices(self, max_devices: int = 127) -> int:
        """
        Sequentially address devices on network.

        Sends SET_ADDRESS to address 0x00 repeatedly until no response.
        Each successful command enables the next device in the daisy chain
        and assigns it the next sequential address (1, 2, 3, ...).

        Args:
            max_devices: Maximum address to try (default 127)

        Returns:
            Number of devices successfully addressed
        """
        addressed = 0

        for addr in range(1, max_devices + 1):
            try:
                response = self.protocol.send_command(
                    ADDRESS_UNADDRESSED,
                    CMD_SET_ADDRESS,
                    [addr, ADDRESS_GROUP]
                )

                if len(response) >= 2:
                    addressed += 1
                    time.sleep(DELAY_AFTER_ADDRESS)
                else:
                    break

            except (LDCNTimeoutError, LDCNChecksumError):
                break

        self._last_addressed_count = addressed
        return addressed

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
            List of device info dictionaries:
            [
                {
                    'address': int,
                    'device_id': int,
                    'version': int,
                    'responding': bool
                },
                ...
            ]
        """
        if end_address is None:
            end_address = self._last_addressed_count if self._last_addressed_count > 0 else 127

        device_list = []
        consecutive_non_responding = 0

        for addr in range(start_address, end_address + 1):
            device_info = {
                'address': addr,
                'device_id': DEVICE_ID_UNKNOWN,
                'version': 0,
                'responding': False
            }

            try:
                # Query with device ID bit set (bit 5 = 0x20)
                response = self.protocol.send_command(addr, CMD_READ_STATUS, [0x20, 0x00])

                if len(response) >= 4:
                    device_info['responding'] = True
                    device_info['device_id'] = response[1]
                    device_info['version'] = response[2]
                    consecutive_non_responding = 0

            except (LDCNTimeoutError, LDCNChecksumError):
                consecutive_non_responding += 1

            device_list.append(device_info)

            # Early exit if too many consecutive non-responding
            if early_exit_threshold > 0 and consecutive_non_responding >= early_exit_threshold:
                break

        return device_list

    # -------------------------------------------------------------------------
    # Device Verification
    # -------------------------------------------------------------------------

    def validate_devices(
        self,
        devices: List['LDCNDevice'],
        expected_devices: Optional[List[Dict]] = None
    ) -> bool:
        """
        Validate existing device objects respond correctly.

        Fast validation that checks if cached device objects still respond
        at the current baud rate without making any state changes.

        Args:
            devices: List of LDCNDevice objects to validate
            expected_devices: Optional list of expected device info dicts

        Returns:
            True if all devices respond correctly
        """
        if not devices and not expected_devices:
            return False

        if expected_devices and not devices:
            return False

        try:
            for device in devices:
                response = self.protocol.send_command(device.address, CMD_NOP)
                if len(response) < 2:
                    return False

                # Verify device ID if expected devices provided
                if expected_devices:
                    expected = next(
                        (d for d in expected_devices if d['address'] == device.address),
                        None
                    )
                    if expected:
                        id_response = self.protocol.send_command(
                            device.address,
                            CMD_READ_STATUS,
                            [STATUS_BIT_DEVICE_ID & 0xFF, (STATUS_BIT_DEVICE_ID >> 8) & 0xFF]
                        )
                        if len(id_response) >= 7:
                            device_id = id_response[5]
                            if device_id != expected['device_id']:
                                return False

            return True

        except (LDCNTimeoutError, LDCNChecksumError):
            return False

    def verify_devices(self, device_list: List[Dict]) -> List[int]:
        """
        Verify devices are still responding.

        Sends NOP command to each device to confirm communication.

        Args:
            device_list: List of device info dicts from discover_devices()

        Returns:
            List of addresses that responded successfully
        """
        responding = []

        for device_info in device_list:
            addr = device_info['address']
            try:
                response = self.protocol.send_command(addr, CMD_NOP)
                if len(response) >= 2:
                    responding.append(addr)
            except (LDCNTimeoutError, LDCNChecksumError):
                pass

        return responding

    # -------------------------------------------------------------------------
    # Device Object Creation
    # -------------------------------------------------------------------------

    def create_device_objects(
        self,
        device_list: List[Dict],
        network: 'LDCNNetwork'
    ) -> List['LDCNDevice']:
        """
        Create device objects from device info list.

        Maps device IDs to appropriate classes (LS231SE, SK2310g2, etc.).

        Args:
            device_list: List of device info dicts from discover_devices()
            network: LDCNNetwork instance (parent for device objects)

        Returns:
            List of LDCNDevice objects
        """
        # Import device classes here to avoid circular import
        from .devices import LS231SE, SK2310g2
        from .device import UnknownDevice

        devices = []

        for device_info in device_list:
            if not device_info['responding']:
                continue

            addr = device_info['address']
            device_id = device_info['device_id']

            # Map device ID to class
            if device_id == DEVICE_ID_LS231SE:
                device = LS231SE(network, addr)
            elif device_id == DEVICE_ID_SK2310G2:
                device = SK2310g2(network, addr)
            else:
                device = UnknownDevice(network, addr)

            device.model_id = device_id
            device.version = device_info['version']
            devices.append(device)

        return devices
