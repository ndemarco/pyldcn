"""
LS-231SE Servo Drive Status Management

Status reading, parsing, and diagnostic detection for LS-231SE servo drives.

Author: NickyDoes
License: GPL v2 or later
"""

import struct
from enum import IntFlag
from typing import Dict, List, Optional, Any, TYPE_CHECKING, Tuple, Union

from pyldcn.device_status import StatusManager
from ..servo_state import ServoState
from .. import servo_diagnostics as diag
from ..servo_mappings import StatusProfile, LS231SE_STATUS_PROFILE

if TYPE_CHECKING:
    from ..servo import LS231SE


# =============================================================================
# Servo Status Byte Flags
# =============================================================================

STATUS_MOVE_DONE = 0x01
STATUS_CKSUM_ERROR = 0x02
STATUS_CURRENT_LIMIT = 0x04
STATUS_POWER = 0x08  # Bit 3: Amplifier power enabled (Pic_ae)
STATUS_POS_ERROR = 0x10
STATUS_HOME_SOURCE = 0x20
STATUS_LIMIT2 = 0x40
STATUS_HOME_IN_PROG = 0x80

# =============================================================================
# Auxiliary Status Byte Flags
# =============================================================================

AUX_INDEX = 0x01           # Bit 0: Complement of index input or diagnostic bit
AUX_POS_WRAP = 0x02        # Bit 1: 32-bit position counter wrapped (sticky)
AUX_SERVO_ON = 0x04        # Bit 2: Position servo loop enabled
AUX_ACCEL_DONE = 0x08      # Bit 3: Acceleration phase complete
AUX_SLEW_DONE = 0x10       # Bit 4: Constant velocity phase complete
AUX_SERVO_OVERRUN = 0x20   # Bit 5: Servo calculation exceeded tick time (sticky)
AUX_PATH_MODE = 0x40       # Bit 6: Currently executing path

# =============================================================================
# LS-231SE Status Items (for DEFINE_STATUS command)
# =============================================================================

LS231SE_STATUS_ITEMS = [
    (0x0001, 'position', 4, 'Current position in encoder counts'),
    (0x0002, 'ad_value', 1, 'Analog-to-digital converter value (0-255)'),
    (0x0004, 'velocity', 2, 'Current velocity in counts per servo tick'),
    (0x0008, 'aux', 1, 'Auxiliary status byte (servo on, overrun flags)'),
    (0x0010, 'home', 4, 'Captured home position in encoder counts'),
    (0x0020, 'device_id', 2, 'Device ID and firmware version'),
    (0x0040, 'pos_error', 2, 'Position following error in encoder counts'),
    (0x0080, 'path_count', 1, 'Path buffer count (motion queue depth)'),
    (0x1000, 'watchdog', 2, 'Watchdog timer status'),
    (0x2000, 'motor_pos', 6, 'Motor position and error (6 bytes)'),
]

# =============================================================================
# Status Item Flags (IntFlag Enum)
# =============================================================================

# Generate IntFlag enum from status items tuple (single source of truth)
_status_item_members = {
    item[1].upper(): item[0]  # 'position' -> 'POSITION': 0x0001
    for item in LS231SE_STATUS_ITEMS
}

StatusItem = IntFlag('StatusItem', _status_item_members)
StatusItem.__doc__ = """Status item flags for LS-231SE READ_STATUS/DEFINE_STATUS commands.

Combine flags with bitwise OR (|) to request multiple items in a single command.

Examples:
    # Single item
    servo.status.read(StatusItem.POSITION)

    # Multiple items (efficient - one command)
    servo.status.read(StatusItem.POSITION | StatusItem.VELOCITY | StatusItem.PATH_COUNT)

    # Use presets for common combinations
    servo.status.read(StatusItem.MOTION)     # position + velocity
    servo.status.read(StatusItem.PATH_MODE)  # position + velocity + aux + path_count

Available items:
    POSITION (0x0001)   - Current position (4 bytes, int32)
    AD_VALUE (0x0002)   - A/D converter value (1 byte, 0-255)
    VELOCITY (0x0004)   - Current velocity (2 bytes, int16)
    AUX (0x0008)        - Auxiliary status byte (servo_on, path_mode, etc.)
    HOME (0x0010)       - Home position (4 bytes, int32)
    DEVICE_ID (0x0020)  - Device ID + firmware version (2 bytes)
    POS_ERROR (0x0040)  - Position following error (2 bytes, int16)
    PATH_COUNT (0x0080) - Path buffer count (1 byte, 0-255)
    WATCHDOG (0x1000)   - Watchdog timer status (2 bytes)
    MOTOR_POS (0x2000)  - Motor position + error (6 bytes)

Common presets:
    MOTION              - POSITION | VELOCITY
    FULL                - POSITION | VELOCITY | AUX | POS_ERROR
    PATH_MODE           - POSITION | VELOCITY | AUX | PATH_COUNT
"""

# Add common presets as class attributes
StatusItem.MOTION = StatusItem.POSITION | StatusItem.VELOCITY
StatusItem.FULL = StatusItem.POSITION | StatusItem.VELOCITY | StatusItem.AUX | StatusItem.POS_ERROR
StatusItem.PATH_MODE = StatusItem.POSITION | StatusItem.VELOCITY | StatusItem.AUX | StatusItem.PATH_COUNT


class ServoStatus(StatusManager):
    """
    LS-231SE servo drive status management.

    This subsystem handles:
    - Reading servo status (full or partial)
    - Parsing variable-length status responses
    - Decoding status and auxiliary flags
    - Detecting fault conditions
    - Updating shared ServoState
    """

    def __init__(self, state: ServoState, device: 'LS231SE', profile: StatusProfile = LS231SE_STATUS_PROFILE):
        """
        Initialize ServoStatus subsystem.

        Args:
            state: Shared ServoState object
            device: Parent LS231SE device instance
        """
        super().__init__(device)
        self._state = state
        self.profile = profile
        self.status_items = profile.status_items

    # -------------------------------------------------------------------------
    # Status Reading
    # -------------------------------------------------------------------------

    def read_status(self, items: Optional[Union[int, StatusItem]] = None) -> Dict:
        """
        Read servo status with optional item selection.

        Args:
            items: Status items to read (StatusItem flags, int mask, or None for default)

        Examples:
            # Default (position, velocity, aux, pos_error)
            servo.status.read_status()

            # Using enum flags (recommended)
            servo.status.read_status(StatusItem.POSITION | StatusItem.VELOCITY)

            # Using presets
            servo.status.read_status(StatusItem.MOTION)
            servo.status.read_status(StatusItem.PATH_MODE)

            # Raw mask still works (for compatibility)
            servo.status.read_status(0x0085)

        Returns:
            {
                'status': status_byte,
                'position': position_counts,  # if requested
                'velocity': velocity,         # if requested
                'aux_status': aux_byte,       # if requested
                'pos_error': following_error, # if requested
                'path_count': count,          # if requested
                'flags': {
                    'move_done': bool,
                    'cksum_error': bool,
                    'current_limit': bool,
                    'power': bool,
                    'pos_error': bool,
                    ...
                }
            }
        """
        from pyldcn.protocol import CMD_READ_STATUS

        # Default to FULL preset if no items specified
        if items is None:
            items = StatusItem.FULL

        # Convert StatusItem enum to int if needed
        status_bits = int(items)

        response = self._device.send_command(
            CMD_READ_STATUS,
            [status_bits & 0xFF, (status_bits >> 8) & 0xFF]
        )

        return self._parse_status(response, status_bits)

    def read_position(self) -> Dict:
        """
        Read position only (fast status read).

        Returns:
            {'position': position_counts, 'status': status_byte}
        """
        from pyldcn.protocol import CMD_READ_STATUS

        # Request only position (bit 0)
        response = self._device.send_command(CMD_READ_STATUS, [0x01, 0x00])

        if len(response) < 6:  # status + 4 bytes position + checksum
            return {'position': None, 'status': None}

        status_byte = response[0]
        position = struct.unpack('<i', bytes(response[1:5]))[0]

        self._state.status_byte = status_byte
        self._state.position = position

        return {'position': position, 'status': status_byte}

    # -------------------------------------------------------------------------
    # Status Parsing
    # -------------------------------------------------------------------------

    def parse(self, response: bytes, mask: int) -> Dict:
        """
        Parse servo status response (implements StatusManager abstract method).

        Args:
            response: Raw response bytes
            mask: Status bits mask used in query

        Returns:
            Parsed status dictionary
        """
        return self._parse_status(response, mask)

    def _parse_status(self, response: bytes, status_bits: int) -> Dict:
        """
        Parse variable-length servo status response.

        Args:
            response: Raw response bytes
            status_bits: Status bits used in query

        Returns:
            Parsed status dictionary
        """
        if len(response) < 2:
            return {}

        status_byte = response[0]
        data = response[1:-1]  # Everything except first byte and checksum

        # Update status byte flag properties in shared state
        self._update_status_flags(status_byte)

        result: Dict[str, Any] = {
            'status': status_byte,
        }

        idx = 0
        aux_byte: Optional[int] = None

        # Position (4 bytes, bit 0)
        if status_bits & 0x0001 and len(data) >= idx + 4:
            result['position'] = struct.unpack('<i', bytes(data[idx:idx+4]))[0]
            self._state.position = result['position']
            idx += 4

        # A/D value (1 byte, bit 1)
        if status_bits & 0x0002 and len(data) >= idx + 1:
            result['ad_value'] = data[idx]
            self._state.ad_value = result['ad_value']
            idx += 1

        # Velocity (2 bytes, bit 2)
        if status_bits & 0x0004 and len(data) >= idx + 2:
            result['velocity'] = struct.unpack('<h', bytes(data[idx:idx+2]))[0]
            self._state.velocity = result['velocity']
            idx += 2

        # Auxiliary status (1 byte, bit 3)
        if status_bits & 0x0008 and len(data) >= idx + 1:
            aux_byte = data[idx]
            result['aux_status'] = aux_byte
            self._update_aux_flags(aux_byte)

            # Also add to result dict for backwards compatibility
            result['servo_on'] = self._state.servo_on
            result['servo_overrun'] = self._state.servo_overrun
            result['path_mode'] = self._state.path_mode
            idx += 1

        # Home position (4 bytes, bit 4)
        if status_bits & 0x0010 and len(data) >= idx + 4:
            result['home'] = struct.unpack('<i', bytes(data[idx:idx+4]))[0]
            self._state.home_position = result['home']
            idx += 4

        # Position error (2 bytes, bit 6)
        if status_bits & 0x0040 and len(data) >= idx + 2:
            result['pos_error'] = struct.unpack('<h', bytes(data[idx:idx+2]))[0]
            self._state.pos_error = result['pos_error']
            idx += 2

        # Path count (1 byte, bit 7)
        if status_bits & 0x0080 and len(data) >= idx + 1:
            result['path_count'] = data[idx]
            self._state.path_count = result['path_count']
            idx += 1

        if aux_byte is None:
            aux_byte = self._state.aux_status or 0

        context = {
            "homesel": getattr(self._state, "home_selection", (0, 0)),
            "mode": getattr(self._state, "current_mode", self.profile.default_mode),
        }
        flags = self.profile.resolver(status_byte, aux_byte, context)
        self._state.home_source_signal = flags.get("home_source_signal")
        self._state.limit2_signal = flags.get("limit2_signal")
        result['flags'] = flags

        return result

    def _update_status_flags(self, status_byte: int) -> None:
        """Update status byte flags in shared state."""
        self._state.status_byte = status_byte
        self._state.move_done = bool(status_byte & STATUS_MOVE_DONE)
        self._state.cksum_error = bool(status_byte & STATUS_CKSUM_ERROR)
        self._state.current_limit = bool(status_byte & STATUS_CURRENT_LIMIT)
        self._state.power = bool(status_byte & STATUS_POWER)
        self._state.pos_error_flag = bool(status_byte & STATUS_POS_ERROR)
        self._state.home_source = bool(status_byte & STATUS_HOME_SOURCE)
        self._state.limit2 = bool(status_byte & STATUS_LIMIT2)
        self._state.home_in_progress = bool(status_byte & STATUS_HOME_IN_PROG)

    def _update_aux_flags(self, aux: int) -> None:
        """Update auxiliary status flags in shared state."""
        self._state.aux_status = aux
        self._state.index = bool(aux & AUX_INDEX)
        self._state.pos_wrap = bool(aux & AUX_POS_WRAP)
        self._state.servo_on = bool(aux & AUX_SERVO_ON)
        self._state.accel_done = bool(aux & AUX_ACCEL_DONE)
        self._state.slew_done = bool(aux & AUX_SLEW_DONE)
        self._state.servo_overrun = bool(aux & AUX_SERVO_OVERRUN)
        self._state.path_mode = bool(aux & AUX_PATH_MODE)

    # -------------------------------------------------------------------------
    # Status Decoding
    # -------------------------------------------------------------------------

    def decode_status_flags(self, status_byte: int, aux_byte: Optional[int] = None) -> Dict[str, bool]:
        """
        Decode status byte into flag dictionary.

        Args:
            status_byte: Status byte from response

        Returns:
            Dictionary of flag names to boolean values
        """
        if aux_byte is None:
            aux_byte = self._state.aux_status or 0
        context = {
            "homesel": getattr(self._state, "home_selection", (0, 0)),
            "mode": getattr(self._state, "current_mode", self.profile.default_mode),
        }
        return self.profile.resolver(status_byte, aux_byte, context)

    # -------------------------------------------------------------------------
    # Fault Detection
    # -------------------------------------------------------------------------

    def check_faults(self, status_byte: int) -> List[str]:
        """
        Check status byte for fault conditions.

        Args:
            status_byte: Status byte to check

        Returns:
            List of active fault names
        """
        faults = []

        if status_byte & STATUS_CKSUM_ERROR:
            faults.append('cksum_error')
        if status_byte & STATUS_CURRENT_LIMIT:
            faults.append('current_limit')
        if status_byte & STATUS_POS_ERROR:
            faults.append('pos_error')

        return faults
