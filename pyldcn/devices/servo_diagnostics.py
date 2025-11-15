"""
LS-231SE Servo Drive Diagnostic Conditions

Status byte pattern matching and diagnostic condition determination.
Implements three-layer architecture:
- Layer 1 (Primary): Raw status byte and auxiliary status byte bits
- Layer 2 (Derived): Pattern-matched CONDITION from diagnostic tables
- Layer 3 (Metadata): Error classification, brake state, LED states

Author: NickyDoes
License: GPL v2 or later
"""

from typing import Dict, Any, Optional, NamedTuple, List


# =============================================================================
# Layer 1: Status Byte Flag Definitions (Primary Hardware State)
# =============================================================================

# Status Byte Flags (8 bits)
STATUS_MOVE_DONE = 0x01        # Bit 0: Trapezoidal move complete
STATUS_CKSUM_ERROR = 0x02      # Bit 1: Checksum error in last command
STATUS_CURRENT_LIMIT = 0x04    # Bit 2: Current limiting exceeded (sticky)
STATUS_POWER = 0x08            # Bit 3: Amplifier power enabled
STATUS_POS_ERROR = 0x10        # Bit 4: Position error exceeded limit (sticky)
STATUS_HOME_SOURCE = 0x20      # Bit 5: Home switch input state
STATUS_LIMIT2 = 0x40           # Bit 6: Forward limit switch state
STATUS_HOME_IN_PROG = 0x80     # Bit 7: Searching for home position

# Auxiliary Status Byte Flags (7 bits)
AUX_INDEX = 0x01               # Bit 0: Complement of index input or diagnostic
AUX_POS_WRAP = 0x02            # Bit 1: Position counter wrapped (sticky)
AUX_SERVO_ON = 0x04            # Bit 2: Position servo loop enabled
AUX_ACCEL_DONE = 0x08          # Bit 3: Acceleration phase complete
AUX_SLEW_DONE = 0x10           # Bit 4: Constant velocity phase complete
AUX_SERVO_OVERRUN = 0x20       # Bit 5: Servo calculation exceeded tick (sticky)
AUX_PATH_MODE = 0x40           # Bit 6: Currently executing path


# =============================================================================
# Layer 2 & 3: Diagnostic Condition (Derived State + Metadata)
# =============================================================================

class DiagnosticCondition(NamedTuple):
    """
    Diagnostic condition derived from status byte patterns.

    Represents a matched condition from LDCN or Amplifier mode diagnostic tables.

    Attributes:
        condition: Condition name (e.g., "ServoON", "AxisOFF", "ErrHALL")
        mode: Operating mode - "LDCN" or "Amplifier"
        bit_pattern: Expected bit values for matching (with 'X' for don't-care)
        is_faulted: True if this is a fault/error condition
        requires_reset: True if hard reset required (e.g., EncoderERR)
        error_class: Error classification - "FAULT", "ERROR", "POWER", "READY", None
        brake_state: Brake output state - "Released", "Engaged", "CN8pin9 High", "CN8pin9 Low"
        fault_relay: Fault relay state - "Open", "Closed"
        orange_led: Orange LED state - "ON", "OFF", "Blink", "Alternate"
        green_led: Green LED state - "ON", "OFF", "Blink", "Alternate"
        red_led: Red LED state - "ON", "OFF", "Blink", "Fast Blink"
    """
    condition: str
    mode: str
    bit_pattern: Dict[str, Any]
    is_faulted: bool
    requires_reset: bool
    error_class: Optional[str]
    brake_state: str
    fault_relay: str
    orange_led: str
    green_led: str
    red_led: str


# =============================================================================
# LDCN Mode Diagnostic Table
# Source: servo_diagnostics.md, LS-231SE datasheet
# =============================================================================

LDCN_DIAGNOSTIC_CONDITIONS = (
    # Normal/Operational States
    DiagnosticCondition(
        condition="No Motor Power after LDCN Init",
        mode="LDCN",
        bit_pattern={'limit2': 0, 'home_source': 1, 'pos_error': 1, 'power': 0,
                    'move_done': 1, 'servo': 0, 'index': 1, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=False,
        requires_reset=False,
        error_class=None,
        brake_state="Released",
        fault_relay="Open",
        orange_led="ON",
        green_led="ON",
        red_led="OFF"
    ),
    DiagnosticCondition(
        condition="AxisOFF",
        mode="LDCN",
        bit_pattern={'limit2': 1, 'home_source': 1, 'pos_error': 1, 'power': 1,
                    'move_done': 0, 'servo': 0, 'index': 'X', 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=False,
        requires_reset=False,
        error_class=None,
        brake_state="CN8pin9 High",
        fault_relay="Open",
        orange_led="OFF",
        green_led="ON",
        red_led="ON"
    ),
    DiagnosticCondition(
        condition="ServoON",
        mode="LDCN",
        bit_pattern={'limit2': 0, 'home_source': 0, 'pos_error': 0, 'power': 1,
                    'move_done': 'X', 'servo': 1, 'index': 'X', 'stop_cmd': 1, 'pic_ae': 1},
        is_faulted=False,
        requires_reset=False,
        error_class=None,
        brake_state="CN8pin9 High",
        fault_relay="Open",
        orange_led="ON",
        green_led="ON",
        red_led="ON"
    ),
    DiagnosticCondition(
        condition="ServoOFF",
        mode="LDCN",
        bit_pattern={'limit2': 0, 'home_source': 0, 'pos_error': 0, 'power': 1,
                    'move_done': 1, 'servo': 0, 'index': 'X', 'stop_cmd': 1, 'pic_ae': 1},
        is_faulted=False,
        requires_reset=False,
        error_class=None,
        brake_state="CN8pin9 Low",
        fault_relay="Open",
        orange_led="OFF",
        green_led="Alternate",
        red_led="ON"
    ),
    DiagnosticCondition(
        condition="Stopped",
        mode="LDCN",
        bit_pattern={'limit2': 0, 'home_source': 0, 'pos_error': 0, 'power': 1,
                    'move_done': 1, 'servo': 0, 'index': 'X', 'stop_cmd': 1, 'pic_ae': 1},
        is_faulted=False,
        requires_reset=False,
        error_class=None,
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="Alternate",
        red_led="Blink"
    ),

    # Error/Fault States
    DiagnosticCondition(
        condition="ErrHALL",
        mode="LDCN",
        bit_pattern={'limit2': 'X', 'home_source': 'X', 'pos_error': 1, 'power': 0,
                    'move_done': 1, 'servo': 0, 'index': 1, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="Blink",
        red_led="Fast Blink"
    ),
    DiagnosticCondition(
        condition="ErrEEPROM",
        mode="LDCN",
        bit_pattern={'limit2': 1, 'home_source': 1, 'pos_error': 1, 'power': 0,
                    'move_done': 0, 'servo': 0, 'index': 0, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=True,
        requires_reset=True,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="Blink",
        red_led="ON"
    ),
    DiagnosticCondition(
        condition="No Motor Power",
        mode="LDCN",
        bit_pattern={'limit2': 1, 'home_source': 0, 'pos_error': 1, 'power': 0,
                    'move_done': 1, 'servo': 0, 'index': 0, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="POWER",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Alternate",
        green_led="OFF",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="Overheat",
        mode="LDCN",
        bit_pattern={'limit2': 0, 'home_source': 0, 'pos_error': 0, 'power': 0,
                    'move_done': 1, 'servo': 0, 'index': 1, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Alternate",
        green_led="Blink",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="Disabled",
        mode="LDCN",
        bit_pattern={'limit2': 1, 'home_source': 0, 'pos_error': 1, 'power': 0,
                    'move_done': 1, 'servo': 0, 'index': 1, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="ON",
        green_led="Blink",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="Master EncoderERROR",
        mode="LDCN",
        bit_pattern={'limit2': 0, 'home_source': 0, 'pos_error': 0, 'power': 0,
                    'move_done': 1, 'servo': 0, 'index': 1, 'stop_cmd': 1, 'pic_ae': 1},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="OFF",
        green_led="Blink",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="Brake or Output Short",
        mode="LDCN",
        bit_pattern={'limit2': 1, 'home_source': 1, 'pos_error': 1, 'power': 0,
                    'move_done': 1, 'servo': 0, 'index': 0, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="ON",
        green_led="ON",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="MotorShort",
        mode="LDCN",
        bit_pattern={'limit2': 1, 'home_source': 0, 'pos_error': 1, 'power': 0,
                    'move_done': 1, 'servo': 0, 'index': 0, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="Blink",
        red_led="ON"
    ),
    DiagnosticCondition(
        condition="Motor PowerDROP",
        mode="LDCN",
        bit_pattern={'limit2': 0, 'home_source': 0, 'pos_error': 1, 'power': 0,
                    'move_done': 'X', 'servo': 0, 'index': 1, 'stop_cmd': 1, 'pic_ae': 1},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="ON",
        green_led="OFF",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="OverLOAD",
        mode="LDCN",
        bit_pattern={'limit2': 1, 'home_source': 0, 'pos_error': 1, 'power': 0,
                    'move_done': 'X', 'servo': 0, 'index': 0, 'stop_cmd': 1, 'pic_ae': 1},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="ON",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="EncoderERR (Reset required)",
        mode="LDCN",
        bit_pattern={'limit2': 'X', 'home_source': 'X', 'pos_error': 1, 'power': 1,
                    'move_done': 1, 'servo': 0, 'index': 'X', 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=True,
        requires_reset=True,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="OFF",
        red_led="OFF"
    ),
    DiagnosticCondition(
        condition="PositionERROR",
        mode="LDCN",
        bit_pattern={'limit2': 0, 'home_source': 1, 'pos_error': 1, 'power': 0,
                    'move_done': 1, 'servo': 0, 'index': 1, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="ON",
        green_led="Blink",
        red_led="ON"
    ),
    DiagnosticCondition(
        condition="Limit2",
        mode="LDCN",
        bit_pattern={'limit2': 1, 'home_source': 'X', 'pos_error': 1, 'power': 1,
                    'move_done': 0, 'servo': 0, 'index': 'X', 'stop_cmd': 0, 'pic_ae': 1},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="Blink",
        red_led="ON"
    ),
    DiagnosticCondition(
        condition="Home Source",
        mode="LDCN",
        bit_pattern={'limit2': 1, 'home_source': 1, 'pos_error': 1, 'power': 0,
                    'move_done': 0, 'servo': 0, 'index': 1, 'stop_cmd': 0, 'pic_ae': 0},
        is_faulted=False,
        requires_reset=False,
        error_class=None,
        brake_state="Released",
        fault_relay="Closed",
        orange_led="OFF",
        green_led="ON",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="Encoder",
        mode="LDCN",
        bit_pattern={'limit2': 'X', 'home_source': 1, 'pos_error': 1, 'power': 1,
                    'move_done': 0, 'servo': 1, 'index': 0, 'stop_cmd': 0, 'pic_ae': 1},
        is_faulted=False,
        requires_reset=False,
        error_class=None,
        brake_state="Released",
        fault_relay="Closed",
        orange_led="OFF",
        green_led="Blink",
        red_led="Blink"
    ),
)

# Lookup dictionary for fast access
_LDCN_DIAGNOSTIC_LOOKUP = {cond.condition: cond for cond in LDCN_DIAGNOSTIC_CONDITIONS}


# =============================================================================
# Amplifier Mode Diagnostic Table
# Source: LS-231SE datasheet lines 2507-2634
# =============================================================================

AMPLIFIER_DIAGNOSTIC_CONDITIONS = (
    DiagnosticCondition(
        condition="Overheat or OverVoltage",
        mode="Amplifier",
        bit_pattern={'limit2': 0, 'home_source': 1, 'pos_error': 0, 'power': 'X'},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Alternate",
        green_led="Blink",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="EncoderERR (Reset required)",
        mode="Amplifier",
        bit_pattern={'limit2': 0, 'home_source': 1, 'pos_error': 1, 'power': 'X'},
        is_faulted=True,
        requires_reset=True,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="OFF",
        red_led="OFF"
    ),
    DiagnosticCondition(
        condition="Triggered Stop",
        mode="Amplifier",
        bit_pattern={'limit2': 'X', 'home_source': 'X', 'pos_error': 0, 'power': 1},
        is_faulted=False,
        requires_reset=False,
        error_class=None,
        brake_state="Engaged",
        fault_relay="Open",
        orange_led="ON",
        green_led="ON",
        red_led="ON"
    ),
    DiagnosticCondition(
        condition="Triggered Output short",
        mode="Amplifier",
        bit_pattern={'limit2': 1, 'home_source': 0, 'pos_error': 0, 'power': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Alternate",
        green_led="Blink",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="EEPROM Checksum Error",
        mode="Amplifier",
        bit_pattern={'limit2': 1, 'home_source': 1, 'pos_error': 0, 'power': 0},
        is_faulted=True,
        requires_reset=True,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="ON",
        green_led="Fast Blink",
        red_led="Fast Blink"
    ),
    DiagnosticCondition(
        condition="Invalid HALL",
        mode="Amplifier",
        bit_pattern={'limit2': 0, 'home_source': 0, 'pos_error': 0, 'power': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="ON",
        green_led="Blink",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="MotorShort or Overheat",
        mode="Amplifier",
        bit_pattern={'limit2': 1, 'home_source': 0, 'pos_error': 1, 'power': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="ON",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="Motor PowerDROP",
        mode="Amplifier",
        bit_pattern={'limit2': 0, 'home_source': 0, 'pos_error': 1, 'power': 1},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="ON",
        green_led="OFF",
        red_led="Blink"
    ),
    DiagnosticCondition(
        condition="No Motor Power",
        mode="Amplifier",
        bit_pattern={'limit2': 1, 'home_source': 1, 'pos_error': 1, 'power': 0},
        is_faulted=True,
        requires_reset=False,
        error_class="POWER",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="Blink",
        green_led="Blink",
        red_led="OFF"
    ),
    DiagnosticCondition(
        condition="PositionERROR",
        mode="Amplifier",
        bit_pattern={'limit2': 0, 'home_source': 1, 'pos_error': 1, 'power': 1},
        is_faulted=True,
        requires_reset=False,
        error_class="FAULT",
        brake_state="Engaged",
        fault_relay="Closed",
        orange_led="ON",
        green_led="Blink",
        red_led="ON"
    ),
)

_AMPLIFIER_DIAGNOSTIC_LOOKUP = {cond.condition: cond for cond in AMPLIFIER_DIAGNOSTIC_CONDITIONS}


# =============================================================================
# Pattern Matching (Layer 1 → Layer 2)
# =============================================================================

def match_diagnostic_condition(
    status_byte: int,
    aux_byte: int,
    stop_cmd: bool = False,
    pic_ae: bool = False,
    mode: str = "LDCN"
) -> Optional[DiagnosticCondition]:
    """
    Match status bytes to diagnostic condition.

    Args:
        status_byte: Status byte value (bits 0-7)
        aux_byte: Auxiliary status byte value (bits 0-6)
        stop_cmd: Stop command bit
        pic_ae: Power driver enable (DE) bit
        mode: "LDCN" or "Amplifier"

    Returns:
        Matched DiagnosticCondition or None if no match found

    Pattern matching handles 'X' (don't care) bits properly.
    """
    # Extract individual bits from status bytes
    bits = {
        'limit2': bool(status_byte & STATUS_LIMIT2),
        'home_source': bool(status_byte & STATUS_HOME_SOURCE),
        'pos_error': bool(status_byte & STATUS_POS_ERROR),
        'power': bool(status_byte & STATUS_POWER),
        'move_done': bool(status_byte & STATUS_MOVE_DONE),
        'servo': bool(aux_byte & AUX_SERVO_ON),
        'index': bool(aux_byte & AUX_INDEX),
        'stop_cmd': stop_cmd,
        'pic_ae': pic_ae
    }

    # Select appropriate table
    conditions = LDCN_DIAGNOSTIC_CONDITIONS if mode == "LDCN" else AMPLIFIER_DIAGNOSTIC_CONDITIONS

    # Match against patterns (handle 'X' as wildcard)
    for condition in conditions:
        if _matches_pattern(bits, condition.bit_pattern):
            return condition

    return None  # No match found


def _matches_pattern(actual_bits: Dict[str, bool], pattern: Dict[str, Any]) -> bool:
    """
    Check if actual bits match pattern.

    Args:
        actual_bits: Dictionary of actual bit values
        pattern: Expected pattern (with 'X' for don't-care)

    Returns:
        True if pattern matches, False otherwise
    """
    for key, expected in pattern.items():
        if expected == 'X':
            continue  # Don't care - any value matches

        actual = actual_bits.get(key, False)
        if actual != bool(expected):
            return False

    return True


# =============================================================================
# Query Functions - Layer 1 (Direct Bit Access)
# =============================================================================

def is_powered(status_byte: int) -> bool:
    """Check if amplifier power enabled (Status Bit 3)."""
    return bool(status_byte & STATUS_POWER)


def is_moving(status_byte: int) -> bool:
    """Check if motion in progress (!Move_done, Bit 0)."""
    return not bool(status_byte & STATUS_MOVE_DONE)


def is_homing(status_byte: int) -> bool:
    """Check if homing in progress (Status Bit 7)."""
    return bool(status_byte & STATUS_HOME_IN_PROG)


def is_servo_on(aux_byte: int) -> bool:
    """Check if servo loop enabled (Aux Bit 2)."""
    return bool(aux_byte & AUX_SERVO_ON)


def is_at_limit(status_byte: int) -> bool:
    """Check if at forward limit switch (Limit2, Bit 6)."""
    return bool(status_byte & STATUS_LIMIT2)


def has_position_error_flag(status_byte: int) -> bool:
    """Check if position error flag set (Bit 4, sticky)."""
    return bool(status_byte & STATUS_POS_ERROR)


def has_current_limit_flag(status_byte: int) -> bool:
    """Check if current limit flag set (Bit 2, sticky)."""
    return bool(status_byte & STATUS_CURRENT_LIMIT)


def has_checksum_error(status_byte: int) -> bool:
    """Check if checksum error in last command (Bit 1)."""
    return bool(status_byte & STATUS_CKSUM_ERROR)


def in_path_mode(aux_byte: int) -> bool:
    """Check if currently executing path (Aux Bit 6)."""
    return bool(aux_byte & AUX_PATH_MODE)


# =============================================================================
# Query Functions - Layer 2 (Condition-Based)
# =============================================================================

def is_faulted(condition: Optional[DiagnosticCondition]) -> bool:
    """Check if condition represents a fault state."""
    return condition.is_faulted if condition else False


def needs_reset(condition: Optional[DiagnosticCondition]) -> bool:
    """Check if condition requires hard reset."""
    return condition.requires_reset if condition else False


def is_brake_released(condition: Optional[DiagnosticCondition]) -> bool:
    """Check if brake is released."""
    if not condition:
        return False
    return condition.brake_state == "Released"


def is_operational(condition: Optional[DiagnosticCondition]) -> bool:
    """
    Check if servo is in operational state (not faulted, ready for motion).

    Returns True for: ServoON, ServoOFF, AxisOFF (normal operational states)
    Returns False for: All fault conditions
    """
    if not condition:
        return False

    # Operational states that can accept commands
    operational_states = ["ServoON", "ServoOFF", "AxisOFF", "Stopped"]
    return not condition.is_faulted and condition.condition in operational_states


def get_error_class(condition: Optional[DiagnosticCondition]) -> Optional[str]:
    """Get error classification: "FAULT", "ERROR", "POWER", "READY", or None."""
    return condition.error_class if condition else None


# =============================================================================
# Comprehensive State Query (All Layers)
# =============================================================================

def get_servo_state(
    status_byte: int,
    aux_byte: int,
    stop_cmd: bool = False,
    pic_ae: bool = False,
    mode: str = "LDCN"
) -> Dict[str, Any]:
    """
    Get comprehensive servo state from status bytes.

    Args:
        status_byte: Status byte value
        aux_byte: Auxiliary status byte value
        stop_cmd: Stop command bit
        pic_ae: Power driver enable bit
        mode: "LDCN" or "Amplifier"

    Returns:
        Dictionary containing:
        - Layer 1 (flags): All individual bit flags
        - Layer 2 (condition): Matched diagnostic condition
        - Layer 3 (metadata): Quick access to key states
    """
    # Layer 1 - All status flags
    flags = {
        'move_done': bool(status_byte & STATUS_MOVE_DONE),
        'cksum_error': bool(status_byte & STATUS_CKSUM_ERROR),
        'current_limit': bool(status_byte & STATUS_CURRENT_LIMIT),
        'power': bool(status_byte & STATUS_POWER),
        'pos_error': bool(status_byte & STATUS_POS_ERROR),
        'home_source': bool(status_byte & STATUS_HOME_SOURCE),
        'limit2': bool(status_byte & STATUS_LIMIT2),
        'home_in_progress': bool(status_byte & STATUS_HOME_IN_PROG),
        'index': bool(aux_byte & AUX_INDEX),
        'pos_wrap': bool(aux_byte & AUX_POS_WRAP),
        'servo_on': bool(aux_byte & AUX_SERVO_ON),
        'accel_done': bool(aux_byte & AUX_ACCEL_DONE),
        'slew_done': bool(aux_byte & AUX_SLEW_DONE),
        'servo_overrun': bool(aux_byte & AUX_SERVO_OVERRUN),
        'path_mode': bool(aux_byte & AUX_PATH_MODE),
    }

    # Layer 2 - Matched condition
    condition = match_diagnostic_condition(status_byte, aux_byte, stop_cmd, pic_ae, mode)

    # Layer 3 - Quick access metadata
    return {
        'status_byte': status_byte,
        'aux_byte': aux_byte,
        'flags': flags,
        'condition': condition,
        'is_faulted': is_faulted(condition),
        'requires_reset': needs_reset(condition),
        'brake_released': is_brake_released(condition),
        'operational': is_operational(condition),
        'error_class': get_error_class(condition),
    }


# =============================================================================
# Formatting Functions
# =============================================================================

def format_status_flags(status_byte: int, aux_byte: int) -> str:
    """
    Format all status byte flags (Layer 1) for display.

    Args:
        status_byte: Status byte value
        aux_byte: Auxiliary status byte value

    Returns:
        Formatted string showing all flag states
    """
    lines = []
    lines.append("Status Byte Flags (0x{:02X}):".format(status_byte))
    lines.append("  Bit 0 (Move Done):      {}".format("YES" if status_byte & STATUS_MOVE_DONE else "NO"))
    lines.append("  Bit 1 (Cksum Error):    {}".format("YES" if status_byte & STATUS_CKSUM_ERROR else "NO"))
    lines.append("  Bit 2 (Current Limit):  {}".format("YES" if status_byte & STATUS_CURRENT_LIMIT else "NO"))
    lines.append("  Bit 3 (Power):          {}".format("ON" if status_byte & STATUS_POWER else "OFF"))
    lines.append("  Bit 4 (Pos Error):      {}".format("YES" if status_byte & STATUS_POS_ERROR else "NO"))
    lines.append("  Bit 5 (Home Source):    {}".format("HIGH" if status_byte & STATUS_HOME_SOURCE else "LOW"))
    lines.append("  Bit 6 (Limit2):         {}".format("ACTIVE" if status_byte & STATUS_LIMIT2 else "INACTIVE"))
    lines.append("  Bit 7 (Homing):         {}".format("YES" if status_byte & STATUS_HOME_IN_PROG else "NO"))

    lines.append("\nAuxiliary Status Flags (0x{:02X}):".format(aux_byte))
    lines.append("  Bit 0 (Index):          {}".format("HIGH" if aux_byte & AUX_INDEX else "LOW"))
    lines.append("  Bit 1 (Pos Wrap):       {}".format("YES" if aux_byte & AUX_POS_WRAP else "NO"))
    lines.append("  Bit 2 (Servo On):       {}".format("YES" if aux_byte & AUX_SERVO_ON else "NO"))
    lines.append("  Bit 3 (Accel Done):     {}".format("YES" if aux_byte & AUX_ACCEL_DONE else "NO"))
    lines.append("  Bit 4 (Slew Done):      {}".format("YES" if aux_byte & AUX_SLEW_DONE else "NO"))
    lines.append("  Bit 5 (Servo Overrun):  {}".format("YES" if aux_byte & AUX_SERVO_OVERRUN else "NO"))
    lines.append("  Bit 6 (Path Mode):      {}".format("YES" if aux_byte & AUX_PATH_MODE else "NO"))

    return "\n".join(lines)


def format_condition(condition: Optional[DiagnosticCondition], include_leds: bool = False) -> str:
    """
    Format diagnostic condition (Layer 2+3) for display.

    Args:
        condition: DiagnosticCondition to format
        include_leds: If True, include LED states in output

    Returns:
        Formatted string with condition details
    """
    if not condition:
        return "Condition: UNKNOWN (no match found)"

    lines = []
    lines.append("Diagnostic Condition: {}".format(condition.condition))
    lines.append("  Mode:           {}".format(condition.mode))
    lines.append("  Faulted:        {}".format("YES" if condition.is_faulted else "NO"))
    lines.append("  Needs Reset:    {}".format("YES" if condition.requires_reset else "NO"))
    lines.append("  Error Class:    {}".format(condition.error_class or "None"))
    lines.append("  Brake:          {}".format(condition.brake_state))
    lines.append("  Fault Relay:    {}".format(condition.fault_relay))

    if include_leds:
        lines.append("\nLED Indicators:")
        lines.append("  Orange LED:     {}".format(condition.orange_led))
        lines.append("  Green LED:      {}".format(condition.green_led))
        lines.append("  Red LED:        {}".format(condition.red_led))

    return "\n".join(lines)


def format_comprehensive_status(state: Dict[str, Any]) -> str:
    """
    Format complete servo status including all layers.

    Args:
        state: State dictionary from get_servo_state()

    Returns:
        Multi-line formatted string with complete status
    """
    lines = []
    lines.append("=" * 60)
    lines.append("LS-231SE SERVO STATUS")
    lines.append("=" * 60)

    # Raw bytes
    lines.append("\nRaw Status:")
    lines.append("  Status Byte:    0x{:02X}".format(state['status_byte']))
    lines.append("  Aux Byte:       0x{:02X}".format(state['aux_byte']))

    # Condition
    lines.append("\n{}".format(format_condition(state['condition'], include_leds=False)))

    # Quick status
    lines.append("\nQuick Status:")
    lines.append("  Operational:    {}".format("YES" if state['operational'] else "NO"))
    lines.append("  Faulted:        {}".format("YES" if state['is_faulted'] else "NO"))
    lines.append("  Brake Released: {}".format("YES" if state['brake_released'] else "NO"))

    # Key flags
    flags = state['flags']
    lines.append("\nKey Flags:")
    lines.append("  Moving:         {}".format("YES" if not flags['move_done'] else "NO"))
    lines.append("  Servo On:       {}".format("YES" if flags['servo_on'] else "NO"))
    lines.append("  Powered:        {}".format("YES" if flags['power'] else "NO"))
    lines.append("  At Limit:       {}".format("YES" if flags['limit2'] else "NO"))
    lines.append("  Homing:         {}".format("YES" if flags['home_in_progress'] else "NO"))

    lines.append("=" * 60)

    return "\n".join(lines)


def format_brief_status(condition: Optional[DiagnosticCondition]) -> str:
    """
    Format brief one-line status.

    Args:
        condition: DiagnosticCondition to format

    Returns:
        Brief status string
    """
    if not condition:
        return "UNKNOWN"

    status_parts = [condition.condition]
    if condition.is_faulted:
        status_parts.append("FAULTED")
    if condition.requires_reset:
        status_parts.append("RESET REQUIRED")

    return " - ".join(status_parts)
