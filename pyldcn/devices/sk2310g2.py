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

from typing import Dict, Any


# =============================================================================
# Diagnostic Code Descriptions
# =============================================================================

DIAGNOSTIC_CODES = {
    0x00: "Power OFF delay in progress",
    0x01: "Initializing",
    0x02: "Control voltage shorted",
    0x03: "Output shorted",
    0x04: "Control voltage LOW (less than 18V)",
    0x05: "Home/Test switch malfunction",
    0x06: "Power UP Home error",
    0x07: "Power UP manual override error",
    0x08: "System LOCKED",
    0x09: "Watchdog Stop",
    0x0A: "Safety Link Error",
    0x0B: "Guard Open Stop - spindle not stopped",
    0x0C: "Guard Open Stop - not in safe zone",
    0x0D: "Guard Open Stop - manual override w/o Enable",
    0x0E: "Guard contact fault",
    0x0F: "Limit Switch Stop",
    0x10: "Emergency Stop",
    0x11: "E-Stop contact fault or Monitor Loop Open",
    0x12: "Busy or Power button short/Monitor Loop Open",
    0x13: "Motor Power Supply under-voltage",
    0x14: "Guard-1 Open; Guard-2 Open (ready to power)",
    0x15: "Guard-1 Closed; Guard-2 Open (ready to power)",
    0x16: "Guard-1 Open; Guard-2 Closed (ready to power)",
    0x17: "Guard-1 Closed; Guard-2 Closed (ready to power)",
    0x18: "Guard-1 Open; Guard-2 Open; Manual override",
    0x19: "Guard-1 Closed; Guard-2 Open; Manual override",
    0x1A: "Guard-1 Open; Guard-2 Closed; Manual override",
    0x1B: "Guard-1 Closed; Guard-2 Closed; Manual override",
    0x1C: "Guard-1 Open; Guard-2 Open; Safe zone; Spindle stopped",
    0x1D: "Guard-1 Closed; Guard-2 Open; Safe zone; Spindle stopped",
    0x1E: "Guard-1 Open; Guard-2 Closed; Safe zone; Spindle stopped",
    0x1F: "Normal operation - All guards closed",
}


# =============================================================================
# LS-773 Status Response Parsing
# =============================================================================

def parse_ls773_status(response: bytes) -> Dict[str, Any]:
    """
    Parse LS-773 format status response from SK-2310g2.

    The SK-2310g2 uses LS-773 format which differs from standard LDCN:
    - Byte0 and Byte1 come immediately after status byte (indices 1-2)
    - Standard LDCN places them near the end (indices 18-19)

    Args:
        response: Raw response bytes from CMD_READ_STATUS with [0xFF, 0xFF]

    Returns:
        Dictionary containing all parsed status fields

    LS-773 Response Format (22 bytes total):
        [status(1)] [byte0(1)] [byte1(1)] [position(4)] [ad(1)] [velocity(2)]
        [aux(1)] [home(4)] [dev_id(2)] [pos_err(2)] [pathbuf(1)] [analog(2)]
        [checksum(1)]
    """
    if len(response) < 22:
        return {}

    idx = 0
    status_byte = response[idx]; idx += 1

    # SK-2310g2 specific: Byte0 and Byte1 come FIRST (LS-773 format)
    byte0 = response[idx]; idx += 1  # Digital inputs
    byte1 = response[idx]; idx += 1  # Internal status + diagnostic

    # Then standard LDCN motion fields
    position = int.from_bytes(response[idx:idx+4], 'little', signed=True); idx += 4
    ad_value = response[idx]; idx += 1
    velocity = int.from_bytes(response[idx:idx+2], 'little', signed=True); idx += 2
    auxiliary = response[idx]; idx += 1
    home = int.from_bytes(response[idx:idx+4], 'little', signed=True); idx += 4
    device_id_raw = int.from_bytes(response[idx:idx+2], 'little'); idx += 2
    position_error = int.from_bytes(response[idx:idx+2], 'little', signed=True); idx += 2
    path_buffer = response[idx]; idx += 1
    analog_inputs = int.from_bytes(response[idx:idx+2], 'little'); idx += 2

    # Extract device ID and version
    device_id = device_id_raw & 0xFF
    version = (device_id_raw >> 8) & 0xFF

    # Extract diagnostic code from byte1 bits [7:3]
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

    # Power state from status byte
    STATUS_POWER_ON = 0x01
    power_state = bool(status_byte & STATUS_POWER_ON)

    return {
        # Raw bytes
        'status': status_byte,
        'byte0': byte0,
        'byte1': byte1,

        # LDCN motion controller fields
        'position': position,
        'ad_value': ad_value,
        'velocity': velocity,
        'auxiliary': auxiliary,
        'home': home,
        'device_id': device_id,
        'version': version,
        'position_error': position_error,
        'path_buffer': path_buffer,
        'analog_inputs': analog_inputs,

        # SK-2310g2 specific fields
        'diagnostic': diagnostic,
        'power_state': power_state,

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
    condition = DIAGNOSTIC_CODES.get(diagnostic, "Unknown condition")

    lines.append(f"\nDiagnostic Code:")
    lines.append(f"  Code:           0x{diagnostic:02X} ({diagnostic:05b}b)")
    lines.append(f"  LED Display:    {led_pattern}  (5-4-3-2-1)")
    lines.append(f"  Condition:      {condition}")

    # Raw values
    lines.append(f"\nRaw Status:")
    lines.append(f"  Status byte:    0x{status.get('status', 0):02X}")
    lines.append(f"  Byte0:          0x{status.get('byte0', 0):02X}")
    lines.append(f"  Byte1:          0x{status.get('byte1', 0):02X}")

    # Digital inputs
    lines.append(f"\nDigital Inputs (Byte0):")
    lines.append(f"  Input 1:        {status.get('input1', False)}")
    lines.append(f"  Input 2:        {status.get('input2', False)}")
    lines.append(f"  Spindle stopped: {status.get('spindle_stopped', False)}")
    lines.append(f"  Spindle fault:  {status.get('spindle_fault', False)}")
    lines.append(f"  Input 3:        {status.get('input3', False)}")
    lines.append(f"  Input 4:        {status.get('input4', False)}")
    lines.append(f"  Input 5:        {status.get('input5', False)}")
    lines.append(f"  Input 6:        {status.get('input6', False)}")

    # Internal status
    lines.append(f"\nInternal Status (Byte1):")
    lines.append(f"  Safe state:     {status.get('safe_state', False)}")
    lines.append(f"  Manual override: {status.get('manual_override', False)}")
    lines.append(f"  Servo fault:    {status.get('servo_fault', False)}")

    # Power state
    lines.append(f"\nPower:")
    lines.append(f"  Power state:    {status.get('power_state', False)}")

    # LDCN motion controller fields (for reference)
    lines.append(f"\nLDCN Motion Fields:")
    lines.append(f"  Position:       {status.get('position', 0)}")
    lines.append(f"  Velocity:       {status.get('velocity', 0)}")
    lines.append(f"  Home:           {status.get('home', 0)}")

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
