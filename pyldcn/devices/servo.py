"""
LS-231SE Servo Drive Device

Servo-specific operations including motion control, status reading,
and PID gain configuration.

Author: NickyDoes
License: GPL v2 or later
"""

import time
import struct
from typing import Optional, Dict, List

# Import from parent package (modular architecture)
from pyldcn.device import LDCNDevice, STATUS_POWER_ON
from pyldcn.network import LDCNNetwork
from pyldcn.protocol import CMD_READ_STATUS



# =============================================================================
# Servo Drive Commands
# =============================================================================

CMD_LOAD_TRAJECTORY = 0x04
CMD_START_MOTION = 0x05
CMD_LOAD_GAINS = 0x06
CMD_STOP_MOTOR = 0x07
CMD_SET_HOME_MODE = 0x09
CMD_CLEAR_BITS = 0x0B

# =============================================================================
# LS-231SE Status Items (for DEFINE_STATUS command)
# =============================================================================

# Format: [(bit_mask, name, byte_size, description), ...]
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
# Servo Status Byte Flags
# =============================================================================

STATUS_MOVE_DONE = 0x01
STATUS_CKSUM_ERROR = 0x02
STATUS_CURRENT_LIMIT = 0x04
# STATUS_POWER_ON imported from network.py (shared constant)
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
# Motor Control Flags (STOP_MOTOR command)
# =============================================================================

AMP_ENABLE = 0x01      # Bit 0: Pic_ae (Power Driver enable)
MOTOR_OFF = 0x02       # Bit 1: Turn motor off
STOP_ABRUPT = 0x04     # Bit 2: Stop abruptly
STOP_SMOOTH = 0x08     # Bit 3: Stop smoothly
STOP_HERE = 0x10       # Bit 4: Stop here


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

        Status byte flags:
            move_done: Trapezoidal move complete or velocity stable
            cksum_error: Checksum error in last command
            current_limit: Current limiting exceeded (sticky)
            power_on: Amplifier power enabled
            pos_error_flag: Position error exceeded limit (sticky)
            home_source: Home switch input state
            limit2: Forward limit switch state
            home_in_progress: Searching for home position

        Auxiliary status byte flags:
            index: Complement of index input
            pos_wrap: Position counter wrapped (sticky)
            servo_on: Position servo loop enabled
            accel_done: Acceleration phase complete
            slew_done: Constant velocity phase complete
            servo_overrun: Servo calculation exceeded tick time (sticky)
            path_mode: Currently executing path

        Control signals:
            stop_cmd: Stop motor command bit
            pic_ae: Power driver enable (Pic_ae≡DE)

        PID gains:
            kp, kd, ki: Proportional, derivative, integral gains
    """

    def __init__(self, network: LDCNNetwork, address: int):
        """
        Initialize servo drive.

        Args:
            network: Parent LDCNNetwork object
            address: Device address (1-127)

        Default property values reflect the "No Motor Power after LDCN Init"
        condition from the diagnostic table (power-up/reset state).

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

        # Status byte flags (defaults: "No Motor Power after LDCN Init" condition)
        self.move_done: bool = True          # Bit 0: 1
        self.cksum_error: bool = False       # Bit 1: 0
        self.current_limit: bool = False     # Bit 2: 0
        self.power_on: bool = False          # Bit 3: 0 (power driver disabled)
        self.pos_error_flag: bool = True     # Bit 4: 1
        self.home_source: bool = True        # Bit 5: 1
        self.limit2: bool = False            # Bit 6: 0
        self.home_in_progress: bool = False  # Bit 7: 0

        # Auxiliary status byte flags (defaults: "No Motor Power after LDCN Init")
        self.index: bool = True              # Aux Bit 0: 1
        self.pos_wrap: bool = False          # Aux Bit 1: 0 (sticky)
        self.servo_on: bool = False          # Aux Bit 2: 0
        self.accel_done: bool = False        # Aux Bit 3: 0
        self.slew_done: bool = False         # Aux Bit 4: 0
        self.servo_overrun: bool = False     # Aux Bit 5: 0 (sticky)
        self.path_mode: bool = False         # Aux Bit 6: 0

        # Control signals (defaults: "No Motor Power after LDCN Init")
        self.stop_cmd: bool = False          # Stop command bit: 0
        self.pic_ae: bool = False            # Drive enable (Pic_ae≡DE): 0

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
            # Request: position, velocity, aux, pos_error
            status_bits = 0x0001 | 0x0004 | 0x0008 | 0x0040
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
        # Request: position, velocity, aux, pos_error
        status_bits = 0x0001 | 0x0004 | 0x0008 | 0x0040
        response = self.send_command(CMD_READ_STATUS, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

        return self._parse_status(response, status_bits)

    def read_position(self) -> Dict:
        """
        Read position only (fast status read).

        Returns:
            {'position': position_counts, 'status': status_byte}

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        # Request only position (bit 0)
        response = self.send_command(CMD_READ_STATUS, [0x01, 0x00])

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

        # Update status byte flag properties
        self.status_byte = status_byte
        self.move_done = bool(status_byte & STATUS_MOVE_DONE)
        self.cksum_error = bool(status_byte & STATUS_CKSUM_ERROR)
        self.current_limit = bool(status_byte & STATUS_CURRENT_LIMIT)
        self.power_on = bool(status_byte & STATUS_POWER_ON)
        self.pos_error_flag = bool(status_byte & STATUS_POS_ERROR)
        self.home_source = bool(status_byte & STATUS_HOME_SOURCE)
        self.limit2 = bool(status_byte & STATUS_LIMIT2)
        self.home_in_progress = bool(status_byte & STATUS_HOME_IN_PROG)

        result = {
            'status': status_byte,
            'flags': self.decode_status_flags(status_byte)
        }

        idx = 0

        # Position (4 bytes, bit 0)
        if status_bits & 0x0001 and len(data) >= idx + 4:
            result['position'] = struct.unpack('<i', bytes(data[idx:idx+4]))[0]
            self.position = result['position']
            idx += 4

        # A/D value (1 byte, bit 1)
        if status_bits & 0x0002 and len(data) >= idx + 1:
            result['ad_value'] = data[idx]
            idx += 1

        # Velocity (2 bytes, bit 2)
        if status_bits & 0x0004 and len(data) >= idx + 2:
            result['velocity'] = struct.unpack('<h', bytes(data[idx:idx+2]))[0]
            self.velocity = result['velocity']
            idx += 2

        # Auxiliary status (1 byte, bit 3)
        if status_bits & 0x0008 and len(data) >= idx + 1:
            aux = data[idx]
            result['aux_status'] = aux

            # Update auxiliary status flag properties
            self.aux_status = aux
            self.index = bool(aux & AUX_INDEX)
            self.pos_wrap = bool(aux & AUX_POS_WRAP)
            self.servo_on = bool(aux & AUX_SERVO_ON)
            self.accel_done = bool(aux & AUX_ACCEL_DONE)
            self.slew_done = bool(aux & AUX_SLEW_DONE)
            self.servo_overrun = bool(aux & AUX_SERVO_OVERRUN)
            self.path_mode = bool(aux & AUX_PATH_MODE)

            # Also add to result dict for backwards compatibility
            result['servo_on'] = self.servo_on
            result['servo_overrun'] = self.servo_overrun
            result['path_mode'] = self.path_mode
            idx += 1

        # Position error (2 bytes, bit 6)
        if status_bits & 0x0040 and len(data) >= idx + 2:
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

    def set_home_mode(self, limit_switch: Optional[int] = None,
                      use_index: bool = False,
                      stop_mode: str = 'abrupt') -> None:
        """
        Configure homing mode to capture home position.

        Args:
            limit_switch: Which limit switch to home to (1=reverse, 2=forward, None=no limit)
            use_index: If True, capture on encoder index pulse
            stop_mode: How to stop when home is found: 'abrupt', 'smooth', or 'off'

        🔴 UNVERIFIED - Not yet tested on hardware

        Example:
            # Home to Limit 2 (forward), stop abruptly
            device.set_home_mode(limit_switch=2, stop_mode='abrupt')

            # Home to index pulse, stop smoothly
            device.set_home_mode(use_index=True, stop_mode='smooth')
        """
        control_byte = 0

        # Limit switch selection
        if limit_switch == 1:
            control_byte |= 0x01  # Bit 0: Limit 1 (reverse)
        elif limit_switch == 2:
            control_byte |= 0x02  # Bit 1: Limit 2 (forward)

        # Index pulse
        if use_index:
            control_byte |= 0x08  # Bit 3: Index

        # Stop mode (only one should be set)
        if stop_mode == 'off':
            control_byte |= 0x04  # Bit 2: Turn motor off
        elif stop_mode == 'abrupt':
            control_byte |= 0x10  # Bit 4: Stop abruptly
        elif stop_mode == 'smooth':
            control_byte |= 0x20  # Bit 5: Stop smoothly
        else:
            raise ValueError(f"Invalid stop_mode: {stop_mode}. Must be 'abrupt', 'smooth', or 'off'")

        self.send_command(CMD_SET_HOME_MODE, [control_byte])

    def home_to_limit(self, limit_switch: int, velocity: int, accel: int,
                      use_index: bool = False, index_velocity: Optional[int] = None) -> None:
        """
        Perform complete homing sequence to a limit switch.

        This executes the homing procedure:
        1. Set home mode for limit switch
        2. Load velocity trajectory
        3. Start motion
        4. Wait for homing to complete
        5. Optionally: Fine-tune with index pulse

        Args:
            limit_switch: Which limit switch (1=reverse, 2=forward)
            velocity: Homing velocity in counts per servo tick
            accel: Homing acceleration in counts per tick²
            use_index: If True, perform second stage homing to index pulse
            index_velocity: Velocity for index homing (default: velocity/4)

        🔴 UNVERIFIED - Not yet tested on hardware

        Example:
            # Home to Limit 2, then index pulse
            device.home_to_limit(limit_switch=2, velocity=40000, accel=10000, use_index=True)
        """
        if limit_switch not in (1, 2):
            raise ValueError("limit_switch must be 1 (reverse) or 2 (forward)")

        # Stage 1: Home to limit switch
        print(f"Homing to Limit {limit_switch}...")

        # Set home mode
        self.set_home_mode(limit_switch=limit_switch, stop_mode='abrupt')

        # Load velocity trajectory
        # Direction: forward for Limit 2, reverse for Limit 1
        direction_bit = 0 if limit_switch == 2 else 0x40
        traj_ctrl = 0x36 | direction_bit  # Bits: 1,2,4,5 + optional direction bit
        traj_data = struct.pack('<Biii', traj_ctrl, 0, velocity, accel)
        self.send_command(CMD_LOAD_TRAJECTORY, list(traj_data))

        # Start motion
        self.send_command(CMD_START_MOTION, [])

        # Wait for homing to complete
        print("Waiting for limit switch...")
        while True:
            status = self.read_status()
            if not status.get('home_in_progress', False):
                break
            time.sleep(0.05)

        print(f"Reached Limit {limit_switch}")

        # Stage 2: Optional index pulse homing
        if use_index:
            if index_velocity is None:
                index_velocity = velocity // 4  # Use 25% of homing velocity

            print("Fine-tuning to index pulse...")

            # Set home mode for index
            self.set_home_mode(use_index=True, stop_mode='abrupt')

            # Load velocity trajectory in opposite direction
            reverse_direction_bit = 0x40 if limit_switch == 2 else 0
            traj_ctrl = 0x36 | reverse_direction_bit
            traj_data = struct.pack('<Biii', traj_ctrl, 0, index_velocity, accel)
            self.send_command(CMD_LOAD_TRAJECTORY, list(traj_data))

            # Start motion
            self.send_command(CMD_START_MOTION, [])

            # Wait for index capture
            print("Waiting for index pulse...")
            while True:
                status = self.read_status()
                if not status.get('home_in_progress', False):
                    break
                time.sleep(0.05)

            print("Homed to index pulse")

        print("Homing complete")

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

