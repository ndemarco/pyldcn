"""
SK-2310g2 Supervisory Controller Status Parsing and Formatting

This module provides functions for encoding/decoding SK-2310g2 specific
data formats, including LS-773 status response parsing and diagnostic
code interpretation.

The SK-2310g2 uses LS-773 protocol format which differs from standard
LDCN format in byte ordering. See docs/SK-2310g2_supervisor.md for
complete details.

Author: NickyDoes
License: GPL v2 or later
"""

from typing import Dict, Any, Optional, NamedTuple


# =============================================================================
# I/O Label Mappings
# =============================================================================

# Digital Input Labels (Bits 0-15)
DIGITAL_INPUT_LABELS = {
    # Byte0 - Application Inputs
    0: ("Input 0", "CN14", "Program Run"),
    1: ("Input 1", "CN14", "Program Stop"),
    2: ("Input 2", "CN6", "Spindle OFF (computed)"),
    3: ("Input 3", "CN6", "Spindle Fault"),
    4: ("Input 4", "CN6", "Spindle At Speed"),
    5: ("Input 5", "CN7", "Air Pressure OK"),
    6: ("Input 6", "CN7", "Tool Length Switch"),
    7: ("Input 7", "CN7", "Tool Changer Closed"),
    # Byte1 - System Status
    8: ("Input 8", "Internal", "At Home (Safe State)"),
    9: ("Input 9", "CN13", "Test Mode (Manual Override)"),
    10: ("Input 10", "CN3", "Servo Fault"),
    11: ("Input 11", "Internal", "Status LED 1"),
    12: ("Input 12", "Internal", "Status LED 2"),
    13: ("Input 13", "Internal", "Status LED 3"),
    14: ("Input 14", "Internal", "Status LED 4"),
    15: ("Input 15", "Internal", "Status LED 5"),
}

# Digital Output Labels (Bits 0-15)
DIGITAL_OUTPUT_LABELS = {
    # Byte0 - Application Outputs
    0: ("Output 0", "CN14", "Program Running Lamp"),
    1: ("Output 1", "CN14", "Program Stopped Lamp"),
    2: ("Output 2", "CN6", "Spindle ON"),
    3: ("Output 3", "CN6", "Spindle Direction (0=CW, 1=CCW)"),
    4: ("Output 4", "CN6", "Spindle DC-brake"),
    5: ("Output 5", "CN7", "Tool Clamp"),
    6: ("Output 6", "CN7", "Spindle Motor Cooling"),
    7: ("Output 7", "CN7", "Tool Cooling"),
    # Byte1 - System Control
    8: ("Output 8", "CN7", "Tool Changer Unlock"),
    9: ("Output 9", "CN9/CN10", "Guard Lock (1=locked, 0=unlocked)"),
    10: ("Output 10", "Internal", "Home Enable"),
    11: ("Output 11", "Internal", "Test Mode Inhibit"),
    12: ("Output 12", "CN3", "Safety Link Bridge"),
    13: ("Output 13", "CN7", "Inverted Output"),
    14: ("Output 14", "Internal", "Reserved (set to 0)"),
    15: ("Output 15", "Internal", "System Lock / Power ON/OFF"),
}

# Analog Input Labels (Channels 0-2)
ANALOG_INPUT_LABELS = {
    0: ("Spindle Load", "CN6.10", "0-10V"),
    1: ("ADC2 (GP)", "CN17.3", "0-5V"),
    2: ("ADC3 (GP)", "CN17.2", "0-5V"),
}

# Analog Output Labels (Channel 0)
ANALOG_OUTPUT_LABELS = {
    0: ("Spindle Speed", "CN6.11", "0-10V"),
}


# =============================================================================
# SK-2310g2 Status Items (LS-773 Protocol)
# =============================================================================

# Format: [(bit_mask, name, byte_size, description), ...]
SK2310G2_STATUS_ITEMS = [
    (0x01, 'digital_inputs', 2, '16-bit digital input state (Byte 0 + Byte 1)'),
    (0x02, 'analog_in_0', 1, 'Analog input channel 0 (0-255, typically spindle load)'),
    (0x04, 'analog_in_1', 1, 'Analog input channel 1 (0-255, general purpose)'),
    (0x08, 'analog_in_2', 1, 'Analog input channel 2 (0-255, general purpose)'),
    (0x10, 'counter_timer', 4, 'Counter/timer value (32-bit, LSB first)'),
    (0x20, 'device_id', 2, 'Device ID and firmware version'),
    (0x40, 'sync_inputs', 2, 'Digital inputs captured with Sync Input command'),
    (0x80, 'sync_counter', 4, 'Counter/timer captured with Sync Input command'),
]


# =============================================================================
# Diagnostic Code Descriptions
# =============================================================================

class DiagnosticState(NamedTuple):
    """Individual state attributes for a diagnostic code.

    Attributes:
        code: Diagnostic code (0x00-0x1F)
        condition: Human-readable description
        guard_1: Guard 1 state - "OPEN", "CLOSED", or None (not applicable)
        guard_2: Guard 2 state - "OPEN", "CLOSED", or None (not applicable)
        safe_zone: Machine in safe zone - True, False, or None (not applicable)
        spindle_stopped: Spindle stopped - True, False, or None (not applicable)
        manual_override: Manual override active - True, False, or None (not applicable)
        power_ready: Ready for power-on - "ON", "OFF", or None (maintains prior state)
        power_enabled: Power outputs active - "ON", "OFF", or None (maintains prior state)
        error_type: Error classification - "FAULT", "STOP", "INIT", or None
        ready_to_power: System ready to accept power command - True, False, or None
    """
    code: int
    condition: str
    guard_1: Optional[str]
    guard_2: Optional[str]
    safe_zone: Optional[bool]
    spindle_stopped: Optional[bool]
    manual_override: Optional[bool]
    power_ready: Optional[str]
    power_enabled: Optional[str]
    error_type: Optional[str]
    ready_to_power: Optional[bool]


# All 32 diagnostic codes with parsed attributes
# Source: docs/SK-2310g2_supervisor.md Diagnostic Code Table
DIAGNOSTIC_CODES = (
    # 0x00-0x13: Error and initialization states
    DiagnosticState(0x00, "Power OFF delay in progress", None, None, None, None, None, "OFF", "ON", "INIT", False),
    DiagnosticState(0x01, "Initializing", None, None, None, None, None, "OFF", "OFF", "INIT", False),
    DiagnosticState(0x02, "Control voltage shorted", None, None, None, None, None, "OFF", "OFF", "FAULT", False),
    DiagnosticState(0x03, "Output shorted", None, None, None, None, None, "OFF", "OFF", "FAULT", False),
    DiagnosticState(0x04, "Control voltage LOW (less than 18V)", None, None, None, None, None, "OFF", "OFF", "FAULT", False),
    DiagnosticState(0x05, "Home/Test switch malfunction (both contacts ON)", None, None, None, None, None, None, None, "FAULT", False),
    DiagnosticState(0x06, "Power UP Home error", None, None, None, None, None, "OFF", "OFF", "FAULT", False),
    DiagnosticState(0x07, "Power UP manual override error", None, None, None, None, None, "OFF", "OFF", "FAULT", False),
    DiagnosticState(0x08, "System LOCKED", None, None, None, None, None, "OFF", "OFF", "FAULT", False),
    DiagnosticState(0x09, "Watchdog Stop", None, None, None, None, None, "OFF", "OFF", "STOP", False),
    DiagnosticState(0x0A, "Safety Link Error", None, None, None, None, None, "OFF", "OFF", "FAULT", False),
    DiagnosticState(0x0B, "Guard Open Stop - Guards open, spindle not stopped", None, None, False, False, None, "OFF", "OFF", "STOP", False),
    DiagnosticState(0x0C, "Guard Open Stop - Guards open, not in safe zone", None, None, False, None, None, "OFF", "OFF", "STOP", False),
    DiagnosticState(0x0D, "Guard Open Stop - Manual override without Enable button held", None, None, None, None, True, "OFF", "OFF", "STOP", False),
    DiagnosticState(0x0E, "Guard contact fault (one or more contacts malfunctioning)", None, None, None, None, None, None, None, "FAULT", False),
    DiagnosticState(0x0F, "Limit Switch Stop", None, None, None, None, None, "OFF", "OFF", "STOP", False),
    DiagnosticState(0x10, "Emergency Stop", None, None, None, None, None, "OFF", "OFF", "STOP", False),
    DiagnosticState(0x11, "Emergency Stop contact fault or Monitor Loop Open", None, None, None, None, None, "OFF", "OFF", "FAULT", False),
    DiagnosticState(0x12, "Busy (≤6s) or Power button short/Monitor Loop Open (>6s)", None, None, None, None, None, "OFF", "OFF", "FAULT", False),
    DiagnosticState(0x13, "Motor Power Supply under-voltage", None, None, None, None, None, "ON", "ON", "FAULT", False),

    # 0x14-0x17: Ready to power states (guards in various positions)
    DiagnosticState(0x14, "Guard-1 Open; Guard-2 Open (ready to power)", "OPEN", "OPEN", None, None, False, "OFF", "OFF", None, True),
    DiagnosticState(0x15, "Guard-1 Closed; Guard-2 Open (ready to power)", "CLOSED", "OPEN", None, None, False, "OFF", "OFF", None, True),
    DiagnosticState(0x16, "Guard-1 Open; Guard-2 Closed (ready to power)", "OPEN", "CLOSED", None, None, False, "OFF", "OFF", None, True),
    DiagnosticState(0x17, "Guard-1 Closed; Guard-2 Closed (ready to power)", "CLOSED", "CLOSED", None, None, False, "OFF", "OFF", None, True),

    # 0x18-0x1B: Manual override with power on
    DiagnosticState(0x18, "Guard-1 Open; Guard-2 Open; Manual override", "OPEN", "OPEN", None, None, True, "ON", "ON", None, False),
    DiagnosticState(0x19, "Guard-1 Closed; Guard-2 Open; Manual override", "CLOSED", "OPEN", None, None, True, "ON", "ON", None, False),
    DiagnosticState(0x1A, "Guard-1 Open; Guard-2 Closed; Manual override", "OPEN", "CLOSED", None, None, True, "ON", "ON", None, False),
    DiagnosticState(0x1B, "Guard-1 Closed; Guard-2 Closed; Manual override", "CLOSED", "CLOSED", None, None, True, "ON", "ON", None, False),

    # 0x1C-0x1F: Safe zone with power on
    DiagnosticState(0x1C, "Guard-1 Open; Guard-2 Open; Safe zone; Spindle stopped", "OPEN", "OPEN", True, True, False, "ON", "ON", None, False),
    DiagnosticState(0x1D, "Guard-1 Closed; Guard-2 Open; Safe zone; Spindle stopped", "CLOSED", "OPEN", True, True, False, "ON", "ON", None, False),
    DiagnosticState(0x1E, "Guard-1 Open; Guard-2 Closed; Safe zone; Spindle stopped", "OPEN", "CLOSED", True, True, False, "ON", "ON", None, False),
    DiagnosticState(0x1F, "Normal operation - All guards closed", "CLOSED", "CLOSED", True, None, False, "ON", "ON", None, False),
)

# Lookup dictionary: code -> DiagnosticState
_DIAGNOSTIC_LOOKUP = {state.code: state for state in DIAGNOSTIC_CODES}


# =============================================================================
# Diagnostic Code Query Functions
# =============================================================================

def get_diagnostic_state(code: int) -> Optional[DiagnosticState]:
    """Get diagnostic state by code.

    Args:
        code: Diagnostic code (0x00-0x1F)

    Returns:
        DiagnosticState object or None if code is invalid
    """
    return _DIAGNOSTIC_LOOKUP.get(code)


def is_power_ready(code: int) -> bool:
    """Check if system is ready to accept power-on command.

    Args:
        code: Diagnostic code (0x00-0x1F)

    Returns:
        True if power_ready == "ON", False otherwise
    """
    state = get_diagnostic_state(code)
    return state is not None and state.power_ready == "ON"


def is_power_enabled(code: int) -> bool:
    """Check if power outputs A/B are active.

    Args:
        code: Diagnostic code (0x00-0x1F)

    Returns:
        True if power_enabled == "ON", False otherwise
    """
    state = get_diagnostic_state(code)
    return state is not None and state.power_enabled == "ON"


def is_guard_open(code: int, guard_num: int) -> Optional[bool]:
    """Check if specific guard is open.

    Args:
        code: Diagnostic code (0x00-0x1F)
        guard_num: Guard number (1 or 2)

    Returns:
        True if guard is open, False if closed, None if not applicable or invalid
    """
    state = get_diagnostic_state(code)
    if state is None:
        return None

    if guard_num == 1:
        return state.guard_1 == "OPEN" if state.guard_1 is not None else None
    elif guard_num == 2:
        return state.guard_2 == "OPEN" if state.guard_2 is not None else None
    else:
        return None


def is_guard_closed(code: int, guard_num: int) -> Optional[bool]:
    """Check if specific guard is closed.

    Args:
        code: Diagnostic code (0x00-0x1F)
        guard_num: Guard number (1 or 2)

    Returns:
        True if guard is closed, False if open, None if not applicable or invalid
    """
    state = get_diagnostic_state(code)
    if state is None:
        return None

    if guard_num == 1:
        return state.guard_1 == "CLOSED" if state.guard_1 is not None else None
    elif guard_num == 2:
        return state.guard_2 == "CLOSED" if state.guard_2 is not None else None
    else:
        return None


def is_manual_override_active(code: int) -> Optional[bool]:
    """Check if manual override is active.

    Args:
        code: Diagnostic code (0x00-0x1F)

    Returns:
        True if manual override active, False if not, None if not applicable
    """
    state = get_diagnostic_state(code)
    return state.manual_override if state is not None else None


def is_safe_zone(code: int) -> Optional[bool]:
    """Check if machine is in safe zone.

    Args:
        code: Diagnostic code (0x00-0x1F)

    Returns:
        True if in safe zone, False if not, None if not applicable
    """
    state = get_diagnostic_state(code)
    return state.safe_zone if state is not None else None


def is_spindle_stopped(code: int) -> Optional[bool]:
    """Check if spindle is stopped.

    Args:
        code: Diagnostic code (0x00-0x1F)

    Returns:
        True if spindle stopped, False if not, None if not applicable
    """
    state = get_diagnostic_state(code)
    return state.spindle_stopped if state is not None else None


def get_error_type(code: int) -> Optional[str]:
    """Get error classification for diagnostic code.

    Args:
        code: Diagnostic code (0x00-0x1F)

    Returns:
        "FAULT", "STOP", "INIT", or None (normal operation)
    """
    state = get_diagnostic_state(code)
    return state.error_type if state is not None else None


def get_power_state(code: int) -> Dict[str, Any]:
    """Get comprehensive power state from diagnostic code.

    Handles all special cases per SK-2310g2 manual and power_control.md:
    - 0x00: Power OFF delay (transitional, not truly ON)
    - 0x05, 0x0E: Maintain prior state (not deterministic)
    - 0x14-0x17: Ready to power (stable OFF state)
    - 0x13, 0x18-0x1F: Power ON

    Args:
        code: Diagnostic code (0x00-0x1F)

    Returns:
        Dictionary with:
            power_on: bool - Motor power actually ON (stable state)
            ready_to_power: bool - System ready to accept power ON command
            transitioning: bool - In power OFF delay (0x00)
            description: str - Human-readable state
    """
    state = get_diagnostic_state(code)
    if state is None:
        return {
            'power_on': False,
            'ready_to_power': False,
            'transitioning': False,
            'description': 'UNKNOWN'
        }

    # Special case: 0x00 is power OFF delay (transitioning)
    # Has power_enabled="ON" temporarily but error_type="INIT"
    # This is NOT a stable ON state - it's transitioning to OFF
    if code == 0x00:
        return {
            'power_on': False,
            'ready_to_power': False,
            'transitioning': True,
            'description': 'POWERING OFF (delay in progress)'
        }

    # Power is ON if outputs are active (stable ON state)
    if state.power_enabled == "ON":
        return {
            'power_on': True,
            'ready_to_power': True,
            'transitioning': False,
            'description': 'POWER ON'
        }

    # Ready to power: system ready to accept power command (stable OFF state)
    if state.ready_to_power:
        return {
            'power_on': False,
            'ready_to_power': True,
            'transitioning': False,
            'description': 'READY TO POWER'
        }

    # All other states are power OFF (fault/stop conditions)
    return {
        'power_on': False,
        'ready_to_power': False,
        'transitioning': False,
        'description': 'POWER OFF'
    }


# =============================================================================
# LS-773 Status Response Parsing
# =============================================================================

def parse_ls773_status(response: bytes, status_mask: int = 0x00) -> Dict[str, Any]:
    """
    Parse LS-773 format status response from SK-2310g2.

    Parses only the fields corresponding to bits set in status_mask.
    The response contains fields in the order defined by SK2310G2_STATUS_ITEMS.

    Args:
        response: Raw response bytes from CMD_READ_STATUS or NOP
        status_mask: Status item bitmask indicating which fields are present

    Returns:
        Dictionary containing all parsed status fields

    Response Format:
        [status_byte(1)] [status_items based on mask] [checksum(1)]

    Example with status_mask=0x01 (digital inputs only):
        [status(1)] [byte0(1)] [byte1(1)] [checksum(1)] = 4 bytes
    """
    if len(response) < 1:
        return {}

    idx = 0
    status_byte = response[idx]; idx += 1

    # Initialize all fields with defaults
    byte0 = 0
    byte1 = 0
    analog_in_0 = 0
    analog_in_1 = 0
    analog_in_2 = 0
    counter_timer = 0
    device_id = 0
    version = 0
    sync_byte0 = 0
    sync_byte1 = 0
    sync_counter = 0

    # Parse fields based on status_mask
    # Process bits in order from SK2310G2_STATUS_ITEMS

    # Bit 0 (0x01): digital_inputs (2 bytes - byte0, byte1)
    if status_mask & 0x01 and idx + 2 <= len(response):
        byte0 = response[idx]; idx += 1
        byte1 = response[idx]; idx += 1

    # Bit 1 (0x02): analog_in_0 (1 byte)
    if status_mask & 0x02 and idx + 1 <= len(response):
        analog_in_0 = response[idx]; idx += 1

    # Bit 2 (0x04): analog_in_1 (1 byte)
    if status_mask & 0x04 and idx + 1 <= len(response):
        analog_in_1 = response[idx]; idx += 1

    # Bit 3 (0x08): analog_in_2 (1 byte)
    if status_mask & 0x08 and idx + 1 <= len(response):
        analog_in_2 = response[idx]; idx += 1

    # Bit 4 (0x10): counter_timer (4 bytes, LSB first)
    if status_mask & 0x10 and idx + 4 <= len(response):
        counter_timer = int.from_bytes(response[idx:idx+4], 'little', signed=False); idx += 4

    # Bit 5 (0x20): device_id (2 bytes)
    if status_mask & 0x20 and idx + 2 <= len(response):
        device_id_raw = int.from_bytes(response[idx:idx+2], 'little'); idx += 2
        device_id = device_id_raw & 0xFF
        version = (device_id_raw >> 8) & 0xFF

    # Bit 6 (0x40): sync_inputs (2 bytes)
    if status_mask & 0x40 and idx + 2 <= len(response):
        sync_byte0 = response[idx]; idx += 1
        sync_byte1 = response[idx]; idx += 1

    # Bit 7 (0x80): sync_counter (4 bytes, LSB first)
    if status_mask & 0x80 and idx + 4 <= len(response):
        sync_counter = int.from_bytes(response[idx:idx+4], 'little', signed=False); idx += 4

    # Extract diagnostic code from byte1 bits [7:3] (SK-2310g2 manual page 20)
    diagnostic = (byte1 >> 3) & 0x1F

    # Decode Byte0 digital inputs
    input1 = bool(byte0 & 0x01)
    input2 = bool(byte0 & 0x02)
    spindle_stopped = bool(byte0 & 0x04)
    spindle_fault = bool(byte0 & 0x08)
    input3 = bool(byte0 & 0x10)
    input4 = bool(byte0 & 0x20)
    input5 = bool(byte0 & 0x40)
    input6 = bool(byte0 & 0x80)

    # Decode Byte1 internal status
    safe_state = bool(byte1 & 0x01)
    manual_override = bool(byte1 & 0x02)
    servo_fault = bool(byte1 & 0x04)
    # Bits [7:3] are diagnostic code (already extracted)

    # Get comprehensive power state using get_power_state()
    # Handles special cases: 0x00 (transitional), 0x05/0x0E (maintain prior), etc.
    power_info = get_power_state(diagnostic)

    return {
        # Raw bytes
        'status': status_byte,
        'byte0': byte0,
        'byte1': byte1,

        # Status items (mapped from status_mask bits)
        'analog_in_0': analog_in_0,
        'analog_in_1': analog_in_1,
        'analog_in_2': analog_in_2,
        'counter_timer': counter_timer,
        'device_id': device_id,
        'version': version,
        'sync_byte0': sync_byte0,
        'sync_byte1': sync_byte1,
        'sync_counter': sync_counter,

        # SK-2310g2 specific fields
        'diagnostic': diagnostic,
        'power_state': power_info['power_on'],  # Backward compatibility
        'power_on': power_info['power_on'],
        'ready_to_power': power_info['ready_to_power'],
        'power_transitioning': power_info['transitioning'],
        'power_description': power_info['description'],

        # Byte0 - Digital inputs
        'input1': input1,
        'input2': input2,
        'spindle_stopped': spindle_stopped,
        'spindle_fault': spindle_fault,
        'input3': input3,
        'input4': input4,
        'input5': input5,
        'input6': input6,

        # Byte1 - Internal status
        'safe_state': safe_state,
        'manual_override': manual_override,
        'servo_fault': servo_fault,
    }


# =============================================================================
# Status Formatting
# =============================================================================

def format_led_pattern(diagnostic: int) -> str:
    """
    Format diagnostic code as LED pattern string.

    Args:
        diagnostic: 5-bit diagnostic code (0x00-0x1F)

    Returns:
        String showing LED pattern in 5-4-3-2-1 order
        Example: "🟢⚫🟢🟢⚫" for diagnostic 0x16
    """
    led5 = '🟢' if (diagnostic & 0x10) else '⚫'
    led4 = '🟢' if (diagnostic & 0x08) else '⚫'
    led3 = '🟢' if (diagnostic & 0x04) else '⚫'
    led2 = '🟢' if (diagnostic & 0x02) else '⚫'
    led1 = '🟢' if (diagnostic & 0x01) else '⚫'
    return f"{led5}{led4}{led3}{led2}{led1}"


def format_status(status: Dict[str, Any]) -> str:
    """
    Format SK-2310g2 status dictionary for readable display.

    Args:
        status: Status dictionary from parse_ls773_status()

    Returns:
        Multi-line formatted string with all status information
    """
    lines = []
    lines.append("=" * 60)
    lines.append("SK-2310g2 SUPERVISORY CONTROLLER STATUS")
    lines.append("=" * 60)

    # Device info
    lines.append(f"\nDevice:")
    lines.append(f"  Device ID:      {status.get('device_id', 0)}")
    lines.append(f"  Version:        {status.get('version', 0)}")

    # Diagnostic code with LED display
    diagnostic = status.get('diagnostic', 0)
    led_pattern = format_led_pattern(diagnostic)
    diag_state = get_diagnostic_state(diagnostic)
    condition = diag_state.condition if diag_state else "Unknown condition"

    lines.append(f"\nDiagnostic Code:")
    lines.append(f"  Code:           0x{diagnostic:02X} ({diagnostic:05b}b)")
    lines.append(f"  LED Display:    {led_pattern}  (5-4-3-2-1)")
    lines.append(f"  Condition:      {condition}")

    # Raw values
    lines.append(f"\nRaw Status:")
    lines.append(f"  Status byte:    0x{status.get('status', 0):02X}")
    lines.append(f"  Byte0:          0x{status.get('byte0', 0):02X}")
    lines.append(f"  Byte1:          0x{status.get('byte1', 0):02X}")

    # Digital inputs - Byte0 (bits 0-7)
    byte0 = status.get('byte0', 0)
    lines.append(f"\nDigital Inputs (Byte0):")
    for bit in range(8):
        value = (byte0 >> bit) & 1
        state = "HIGH" if value else "LOW "
        _, _, function = DIGITAL_INPUT_LABELS.get(bit, ("", "", "Unknown"))
        lines.append(f"  Input {bit}:       {state}  - {function}")

    # Digital inputs - Byte1 (bits 8-15)
    byte1_val = status.get('byte1', 0)
    lines.append(f"\nDigital Inputs (Byte1):")
    for bit in range(8, 16):
        value = (byte1_val >> (bit - 8)) & 1
        state = "HIGH" if value else "LOW "
        _, _, function = DIGITAL_INPUT_LABELS.get(bit, ("", "", "Unknown"))
        lines.append(f"  Input {bit}:      {state}  - {function}")

    # Analog inputs - using corrected 8-bit byte-based decoding
    analog_raw = status.get('analog_inputs', 0)
    ad_value = status.get('ad_value', 0)
    lines.append(f"\nAnalog Inputs:")
    lines.append(f"  Raw:            analog=0x{analog_raw:04X} ad_value=0x{ad_value:02X}")
    # Channel 0: Low byte (8-bit, 0-255)
    ch0_raw = analog_raw & 0xFF
    ch0_percent = (ch0_raw / 255.0) * 100.0
    lines.append(f"  Channel 0:      {ch0_raw:4d} (0x{ch0_raw:02X}) = {ch0_percent:5.1f}% - Spindle Load")
    # Channel 1: High byte (8-bit, 0-255)
    ch1_raw = (analog_raw >> 8) & 0xFF
    ch1_percent = (ch1_raw / 255.0) * 100.0
    lines.append(f"  Channel 1:      {ch1_raw:4d} (0x{ch1_raw:02X}) = {ch1_percent:5.1f}% - ADC2 (GP)")
    # Channel 2: ad_value field (8-bit, 0-255)
    ch2_raw = ad_value & 0xFF
    ch2_percent = (ch2_raw / 255.0) * 100.0
    lines.append(f"  Channel 2:      {ch2_raw:4d} (0x{ch2_raw:02X}) = {ch2_percent:5.1f}% - ADC3 (GP)")

    # Power state
    lines.append(f"\nPower:")
    lines.append(f"  Power state:    {'ON' if status.get('power_state') else 'OFF'}")

    return "\n".join(lines)


def format_digital_inputs(byte0: int, byte1: int) -> str:
    """
    Format 16-bit digital inputs with labels.

    Args:
        byte0: Input byte 0 (bits 0-7)
        byte1: Input byte 1 (bits 8-15)

    Returns:
        Formatted string showing all inputs with labels
    """
    inputs = (byte1 << 8) | byte0

    lines = []
    lines.append(f"Digital Inputs (Byte1:Byte0 = 0x{inputs:04X}):")
    lines.append(f"{'Bit':<4} {'Val':<4} {'Name':<12} {'Connector':<10} {'Function':<35}")
    lines.append("-" * 75)

    for bit in range(16):
        value = (inputs >> bit) & 1
        name, connector, function = DIGITAL_INPUT_LABELS.get(bit, ("Unknown", "?", "?"))
        lines.append(f"{bit:<4} {value:<4} {name:<12} {connector:<10} {function:<35}")

    return "\n".join(lines)


def format_digital_outputs(byte0: int, byte1: int) -> str:
    """
    Format 16-bit digital outputs with labels.

    Args:
        byte0: Output byte 0 (bits 0-7)
        byte1: Output byte 1 (bits 8-15)

    Returns:
        Formatted string showing all outputs with labels
    """
    outputs = (byte1 << 8) | byte0

    lines = []
    lines.append(f"Digital Outputs (Byte1:Byte0 = 0x{outputs:04X}):")
    lines.append(f"{'Bit':<4} {'Val':<4} {'Name':<12} {'Connector':<10} {'Function':<35}")
    lines.append("-" * 75)

    for bit in range(16):
        value = (outputs >> bit) & 1
        name, connector, function = DIGITAL_OUTPUT_LABELS.get(bit, ("Unknown", "?", "?"))
        lines.append(f"{bit:<4} {value:<4} {name:<12} {connector:<10} {function:<35}")

    return "\n".join(lines)


def format_analog_value(adc_value: int, voltage_range: str) -> tuple:
    """
    Format analog value as hex, decimal, voltage, and percentage.

    Args:
        adc_value: 10-bit ADC value (0-1023)
        voltage_range: "0-10V" or "0-5V"

    Returns:
        (hex_str, dec_str, voltage_str, percent_str)
    """
    max_voltage = 10.0 if voltage_range == "0-10V" else 5.0
    voltage = (adc_value / 1023.0) * max_voltage
    percent = (adc_value / 1023.0) * 100.0

    hex_str = f"0x{adc_value:03X}"
    dec_str = f"{adc_value:4d}"
    voltage_str = f"{voltage:5.2f}V"
    percent_str = f"{percent:5.1f}%"

    return hex_str, dec_str, voltage_str, percent_str


def format_analog_inputs(analog_values: Dict[int, int]) -> str:
    """
    Format analog inputs with hex, decimal, voltage, and percentage.

    Args:
        analog_values: Dictionary mapping channel to ADC value

    Returns:
        Formatted string showing all analog inputs
    """
    lines = []
    lines.append("Analog Inputs:")
    lines.append(f"{'Ch':<4} {'Hex':<7} {'Dec':<6} {'Voltage':<9} {'%':<7} {'Label':<18} {'Connector':<10} {'Range':<6}")
    lines.append("-" * 75)

    for ch in range(3):
        adc_value = analog_values.get(ch, 0)
        label, connector, voltage_range = ANALOG_INPUT_LABELS.get(ch, ("Unknown", "?", "0-5V"))
        hex_str, dec_str, voltage_str, percent_str = format_analog_value(adc_value, voltage_range)
        lines.append(f"{ch:<4} {hex_str:<7} {dec_str:<6} {voltage_str:<9} {percent_str:<7} {label:<18} {connector:<10} {voltage_range:<6}")

    return "\n".join(lines)


def format_analog_output(adc_value: int) -> str:
    """
    Format analog output with hex, decimal, voltage, and percentage.

    Args:
        adc_value: 10-bit DAC value (0-1023)

    Returns:
        Formatted string showing analog output
    """
    lines = []
    lines.append("Analog Output:")
    lines.append(f"{'Ch':<4} {'Hex':<7} {'Dec':<6} {'Voltage':<9} {'%':<7} {'Label':<18} {'Connector':<10} {'Range':<6}")
    lines.append("-" * 75)

    label, connector, voltage_range = ANALOG_OUTPUT_LABELS.get(0, ("Unknown", "?", "0-10V"))
    hex_str, dec_str, voltage_str, percent_str = format_analog_value(adc_value, voltage_range)
    lines.append(f"{0:<4} {hex_str:<7} {dec_str:<6} {voltage_str:<9} {percent_str:<7} {label:<18} {connector:<10} {voltage_range:<6}")

    return "\n".join(lines)


def format_io_report(status: Dict[str, Any]) -> str:
    """
    Format comprehensive I/O report from status dictionary.

    Args:
        status: Status dictionary from parse_ls773_status()

    Returns:
        Multi-line formatted string with complete I/O state
    """
    lines = []
    lines.append("=" * 80)
    lines.append("SK-2310g2 I/O STATUS REPORT")
    lines.append("=" * 80)

    # Device info and diagnostic
    lines.append(f"\nDevice: ID={status.get('device_id', 0)} Version={status.get('version', 0)}")
    diagnostic = status.get('diagnostic', 0)
    led_pattern = format_led_pattern(diagnostic)
    diag_state = get_diagnostic_state(diagnostic)
    condition = diag_state.condition if diag_state else "Unknown condition"
    lines.append(f"Diagnostic: 0x{diagnostic:02X} {led_pattern} - {condition}")

    # Digital Inputs
    byte0 = status.get('byte0', 0)
    byte1 = status.get('byte1', 0)
    lines.append(f"\n{format_digital_inputs(byte0, byte1)}")

    # Digital Outputs (if available)
    if 'digital_outputs' in status:
        out_byte0 = status['digital_outputs'] & 0xFF
        out_byte1 = (status['digital_outputs'] >> 8) & 0xFF
        lines.append(f"\n{format_digital_outputs(out_byte0, out_byte1)}")

    # Analog Inputs
    # Note: Actual encoding TBD - using analog_inputs field as placeholder
    analog_raw = status.get('analog_inputs', 0)
    # Placeholder: Assume analog_inputs is a 16-bit value encoding multiple channels
    # This will need to be updated based on actual hardware response
    analog_values = {
        0: (analog_raw >> 0) & 0x3FF,   # Channel 0: bits 0-9
        1: (analog_raw >> 10) & 0x1F,   # Channel 1: bits 10-14 (scaled)
        2: 0,  # Channel 2: TBD
    }
    lines.append(f"\n{format_analog_inputs(analog_values)}")

    # Analog Output (if available)
    if 'analog_output' in status:
        lines.append(f"\n{format_analog_output(status['analog_output'])}")

    # Safety summary
    lines.append(f"\nSafety Status:")
    lines.append(f"  Power:          {'ON' if status.get('power_state') else 'OFF'}")
    lines.append(f"  At Home:        {'Yes' if status.get('safe_state') else 'No'}")
    lines.append(f"  Manual Override: {'Active' if status.get('manual_override') else 'Inactive'}")
    lines.append(f"  Servo Fault:    {'FAULT' if status.get('servo_fault') else 'OK'}")

    lines.append("\n" + "=" * 80)

    return "\n".join(lines)


# =============================================================================
# Output Encoding (Future expansion)
# =============================================================================

def encode_output_byte0(outputs: Dict[str, bool]) -> int:
    """
    Encode output states into Byte0 format for SET_OUTPUTS command.

    Args:
        outputs: Dictionary with output names and states

    Returns:
        Byte0 value for SET_OUTPUTS command

    Example:
        >>> encode_output_byte0({'output1': True, 'output2': False})
        0x01
    """
    # Placeholder for future implementation
    # See CMD_SET_OUTPUTS in docs/SK-2310g2_supervisor.md
    byte0 = 0

    if outputs.get('output1', False):
        byte0 |= 0x01
    if outputs.get('output2', False):
        byte0 |= 0x02
    if outputs.get('output3', False):
        byte0 |= 0x04
    if outputs.get('output4', False):
        byte0 |= 0x08
    if outputs.get('output5', False):
        byte0 |= 0x10
    if outputs.get('output6', False):
        byte0 |= 0x20
    if outputs.get('output7', False):
        byte0 |= 0x40
    if outputs.get('output8', False):
        byte0 |= 0x80

    return byte0


def encode_output_byte1(power_a: bool = False, power_b: bool = False) -> int:
    """
    Encode power outputs into Byte1 format for SET_OUTPUTS command.

    Args:
        power_a: Power A output state
        power_b: Power B output state

    Returns:
        Byte1 value for SET_OUTPUTS command
    """
    byte1 = 0

    if power_a:
        byte1 |= 0x01
    if power_b:
        byte1 |= 0x02

    return byte1
