"""
SK-2310g2 Supervisory Controller Formatting and Constants

This module provides formatting utilities and I/O label mappings for
the SK-2310g2 supervisory I/O controller.

Status parsing has been moved to pyldcn/devices/status/sk2310g2_status.py
for better organization and reuse.

Author: NickyDoes
License: GPL v2 or later
"""

from typing import Dict, Any

# Import diagnostic codes for formatting functions
from .status.sk2310g2_status import DIAGNOSTIC_CODES


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
# NOTE: Status parsing moved to pyldcn/devices/status/sk2310g2_status.py
# =============================================================================
# SK2310G2_STATUS_ITEMS, DIAGNOSTIC_CODES, and parse_ls773_status()
# have been moved to the SK2310g2Status class for better organization.
# Import from .status.sk2310g2_status if needed.


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
