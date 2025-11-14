"""
LS-231SE Servo Drive State

Shared state container for all servo subsystems - single source of truth.

Author: NickyDoes
License: GPL v2 or later
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class ServoState:
    """
    Shared state container - single source of truth for all subsystems.

    Default values reflect the "No Motor Power after LDCN Init" condition
    from the diagnostic table (power-up/reset state).

    All properties are updated by the Status subsystem and read by other
    subsystems as needed.
    """

    # -------------------------------------------------------------------------
    # Status Byte Flags (8 bits)
    # -------------------------------------------------------------------------

    move_done: bool = True          # Bit 0: Trapezoidal move complete
    cksum_error: bool = False       # Bit 1: Checksum error in last command
    current_limit: bool = False     # Bit 2: Current limiting exceeded (sticky)
    power_on: bool = False          # Bit 3: Amplifier power enabled
    pos_error_flag: bool = True     # Bit 4: Position error exceeded limit (sticky)
    home_source: bool = True        # Bit 5: Home switch input state
    limit2: bool = False            # Bit 6: Forward limit switch state
    home_in_progress: bool = False  # Bit 7: Searching for home position

    # -------------------------------------------------------------------------
    # Auxiliary Status Byte Flags (7 bits)
    # -------------------------------------------------------------------------

    index: bool = True              # Aux Bit 0: Complement of index input
    pos_wrap: bool = False          # Aux Bit 1: Position counter wrapped (sticky)
    servo_on: bool = False          # Aux Bit 2: Position servo loop enabled
    accel_done: bool = False        # Aux Bit 3: Acceleration phase complete
    slew_done: bool = False         # Aux Bit 4: Constant velocity phase complete
    servo_overrun: bool = False     # Aux Bit 5: Servo calculation exceeded tick (sticky)
    path_mode: bool = False         # Aux Bit 6: Currently executing path

    # -------------------------------------------------------------------------
    # Control Signals
    # -------------------------------------------------------------------------

    stop_cmd: bool = False          # Stop motor command bit
    pic_ae: bool = False            # Power driver enable (Pic_ae≡DE)

    # -------------------------------------------------------------------------
    # Motion State
    # -------------------------------------------------------------------------

    position: Optional[int] = None      # Current position (encoder counts)
    velocity: Optional[int] = None      # Current velocity (counts per servo tick)
    pos_error: Optional[int] = None     # Position following error (encoder counts)

    # -------------------------------------------------------------------------
    # Cached Values
    # -------------------------------------------------------------------------

    status_byte: Optional[int] = None   # Last status byte received
    aux_status: Optional[int] = None    # Last auxiliary status byte received
    ad_value: Optional[int] = None      # A/D converter value (0-255)
    home_position: Optional[int] = None # Captured home position
    path_count: Optional[int] = None    # Path buffer count (motion queue depth)

    # -------------------------------------------------------------------------
    # PID Gains
    # -------------------------------------------------------------------------

    kp: Optional[int] = None    # Proportional gain
    kd: Optional[int] = None    # Derivative gain
    ki: Optional[int] = None    # Integral gain
