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

    Note: The SK-2310g2 is an I/O controller, not a motion controller. The
    position, velocity, and home fields are vestigial (inherited from LS-773
    protocol) and have no meaningful values. Use byte0/byte1 for I/O state.

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

    # LDCN motion fields (vestigial - not used for I/O controller, kept for protocol compatibility)
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

    # Power state from status byte (bit 3)
    STATUS_POWER_ON = 0x08
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
    Format SK-2310g2 status dictionary for readable display (compact).

    Args:
        status: Status dictionary from parse_ls773_status()

    Returns:
        Multi-line formatted string with all status information
    """
    lines = []

    # Header line
    diagnostic = status.get('diagnostic', 0)
    led_pattern = format_led_pattern(diagnostic)
    condition = DIAGNOSTIC_CODES.get(diagnostic, "Unknown condition")
    pwr = "ON" if status.get('power_state') else "OFF"
    lines.append(f"SK-2310g2 Status | Pwr:{pwr} | Diag:0x{diagnostic:02X} {led_pattern} {condition}")

    # Raw bytes
    byte0 = status.get('byte0', 0)
    byte1_val = status.get('byte1', 0)
    status_byte = status.get('status', 0)
    lines.append(f"Raw: Status=0x{status_byte:02X} Byte0=0x{byte0:02X} Byte1=0x{byte1_val:02X}")

    # Digital inputs - Byte0 (bits 0-7)
    lines.append("Byte0 Inputs:")
    for bit in range(8):
        value = (byte0 >> bit) & 1
        state = "1" if value else "0"
        _, _, function = DIGITAL_INPUT_LABELS.get(bit, ("", "", "Unknown"))
        lines.append(f"  {bit}:{state} {function}")

    # Digital inputs - Byte1 (bits 8-15)
    lines.append("Byte1 Inputs:")
    for bit in range(8, 16):
        value = (byte1_val >> (bit - 8)) & 1
        state = "1" if value else "0"
        _, _, function = DIGITAL_INPUT_LABELS.get(bit, ("", "", "Unknown"))
        lines.append(f"  {bit}:{state} {function}")

    # Analog inputs
    analog_raw = status.get('analog_inputs', 0)
    # Try byte-swapped interpretation
    # The raw value is 16-bit, try splitting by bytes instead of bit fields
    ch0_raw = analog_raw & 0xFF  # Low byte (8-bit)
    ch1_raw = (analog_raw >> 8) & 0xFF  # High byte (8-bit)
    ch0_pct = (ch0_raw / 255.0) * 100.0  # 8-bit scale
    ch1_pct = (ch1_raw / 255.0) * 100.0  # 8-bit scale
    lines.append("Analog Inputs:")
    lines.append(f"  Raw=0x{analog_raw:04X}")
    lines.append(f"  0: {ch0_raw:4d} (0x{ch0_raw:02X}) {ch0_pct:5.1f}% - Spindle Load")
    lines.append(f"  1: {ch1_raw:4d} (0x{ch1_raw:02X}) {ch1_pct:5.1f}% - ADC2 (GP)")

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
    condition = DIAGNOSTIC_CODES.get(diagnostic, "Unknown condition")
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
