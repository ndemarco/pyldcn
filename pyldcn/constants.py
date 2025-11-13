"""
LDCN Protocol Constants

All protocol constants, commands, timing values, and initialization modes
for the Logosol LDCN (Logosol Distributed Control Network).

Author: NickyDoes
License: GPL v2 or later
"""

from enum import Enum


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

# Status bits for device discovery (16-bit, little-endian)
STATUS_BIT_POSITION = 0x0001      # Bit 0: Position (4 bytes)
STATUS_BIT_AD_VALUE = 0x0002      # Bit 1: A/D value (1 byte)
STATUS_BIT_VELOCITY = 0x0004      # Bit 2: Velocity (2 bytes)
STATUS_BIT_AUX = 0x0008           # Bit 3: Auxiliary status byte (1 byte)
STATUS_BIT_HOME = 0x0010          # Bit 4: Home position (4 bytes)
STATUS_BIT_DEVICE_ID = 0x0020     # Bit 5: Device ID and version (2 bytes)
STATUS_BIT_POS_ERROR = 0x0040     # Bit 6: Position error (2 bytes)
STATUS_BIT_PATH_COUNT = 0x0080    # Bit 7: Path buffer count (1 byte)
STATUS_BIT_DIGITAL_IN = 0x0100    # Bit 8: Digital inputs (2 bytes)
STATUS_BIT_ANALOG_IN = 0x0200     # Bit 9: Analog inputs (2 bytes)
STATUS_BIT_WATCHDOG = 0x1000      # Bit 12: Watchdog status (2 bytes)
STATUS_BIT_MOTOR_POS = 0x2000     # Bit 13: Motor position and error (6 bytes)

# Map status bits to their byte sizes for response calculation
STATUS_BIT_SIZES = {
    STATUS_BIT_POSITION: 4,
    STATUS_BIT_AD_VALUE: 1,
    STATUS_BIT_VELOCITY: 2,
    STATUS_BIT_AUX: 1,
    STATUS_BIT_HOME: 4,
    STATUS_BIT_DEVICE_ID: 2,
    STATUS_BIT_POS_ERROR: 2,
    STATUS_BIT_PATH_COUNT: 1,
    STATUS_BIT_DIGITAL_IN: 2,
    STATUS_BIT_ANALOG_IN: 2,
    STATUS_BIT_WATCHDOG: 2,
    STATUS_BIT_MOTOR_POS: 6,
}

# Status byte flags (common to all LDCN devices)
STATUS_POWER_ON = 0x08            # Bit 3: Power button state

# Device IDs (hardware-reported, TBD - verify from real hardware)
DEVICE_ID_UNKNOWN = 0xFF          # Placeholder for truly unknown devices
DEVICE_ID_LS231SE = 0x00          # ✅ VERIFIED on hardware: Version 0x15
DEVICE_ID_SK2310G2 = 0x02         # ✅ VERIFIED on hardware: Version 0x34
