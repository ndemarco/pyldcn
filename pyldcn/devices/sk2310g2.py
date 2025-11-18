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
# Digital Input Bit Mappings (Page 19 of SK-2310g2 Manual)
# =============================================================================
# Format: 16-bit word = Byte1:Byte0 (MSB:LSB)
# CN6/CN7/CN14 physical inputs → Inputs/Byte0/Bits 0-7
# Internal status signals → Inputs/Byte1/Bits 0-7

# Byte0 - Application Inputs
INPUT_PROGRAM_RUN = 0  # Bit 0: CN14 - Program run signal
INPUT_PROGRAM_STOP = 1  # Bit 1: CN14 - Program stop signal
INPUT_SPINDLE_OFF = 2  # Bit 2: CN6 - Spindle OFF (computed, see Note 1)
INPUT_SPINDLE_FAULT = 3  # Bit 3: CN6 - Spindle fault status
INPUT_SPINDLE_AT_SPEED = 4  # Bit 4: CN6 - Spindle at speed status
INPUT_AIR_PRESSURE = 5  # Bit 5: CN7 - Air pressure OK
INPUT_TOOL_LENGTH_SWITCH = 6  # Bit 6: CN7 - Tool length measurement switch
INPUT_TOOL_CHANGER_CLOSED = 7  # Bit 7: CN7 - Tool changer cover closed

# Byte1 - System Status Inputs
INPUT_AT_HOME = 8  # Bit 0: System at home/safe zone
INPUT_TEST_MODE = 9  # Bit 1: Test mode active
INPUT_SERVO_FAULT = 10  # Bit 2: CN3 Safety Bus - Servo fault signal
INPUT_STATUS_LED1 = 11  # Bit 3: LED1 status
INPUT_STATUS_LED2 = 12  # Bit 4: LED2 status
INPUT_STATUS_LED3 = 13  # Bit 5: LED3 status
INPUT_STATUS_LED4 = 14  # Bit 6: LED4 status
INPUT_STATUS_LED5 = 15  # Bit 7: LED5 status

# Note 1: INPUT_SPINDLE_OFF computation (from manual page 19)
# Spindle OFF = 1 when:
#   - Spindle ON output (Outputs/Byte0/Bit2) = 0 AND
#   - Spindle Stopped input (CN6 pin2) = HIGH


# =============================================================================
# Digital Output Bit Mappings (Page 19 of SK-2310g2 Manual)
# =============================================================================
# Format: 16-bit word = Byte1:Byte0 (MSB:LSB)

# Byte0 - Application Outputs
OUTPUT_PROGRAM_RUNNING_LAMP = 0  # Bit 0: CN14 - Program running lamp
OUTPUT_PROGRAM_STOPPED_LAMP = 1  # Bit 1: CN14 - Program stopped lamp
OUTPUT_SPINDLE_ON = 2  # Bit 2: CN6/CN16 - Spindle ON (see Note 3, Note 4)
OUTPUT_SPINDLE_DIRECTION = 3  # Bit 3: CN6 - Spindle direction (0=CW, 1=CCW)
OUTPUT_SPINDLE_DC_BRAKE = 4  # Bit 4: CN6 - Spindle DC-braking or PWM
OUTPUT_TOOL_CLAMP = 5  # Bit 5: CN7 - Tool clamp solenoid
OUTPUT_SOLENOID_VALVE_2 = 6  # Bit 6: CN7 - Solenoid valve 2
OUTPUT_SOLENOID_VALVE_3 = 7  # Bit 7: CN7 - Solenoid valve 3

# Byte1 - System Control Outputs
OUTPUT_TOOL_CHANGER_UNLOCK = 8  # Bit 0: CN7 - Tool changer cover unlock
OUTPUT_GUARD_LOCK = 9  # Bit 1: CN9/CN10 - Guard lock control
OUTPUT_HOME_ENABLE = 10  # Bit 2: See automation modes / Home enable
OUTPUT_TEST_MODE_INHIBIT = 11  # Bit 3: Test mode inhibit control
OUTPUT_SAFETY_LINK_BRIDGE = (
    12  # Bit 4: CN3 Safety Bus - Safety Link Bridge (see Note 3)
)
OUTPUT_INVERTED_13 = 13  # Bit 5: CN7 - Inverted output (HIGH when bit=0)
OUTPUT_COVERS_LOCK_UNLOCK = 14  # Bit 6: Reserved / Covers lock/unlock (see Note 5)
OUTPUT_SYSTEM_LOCK = 15  # Bit 7: System lock / Power ON/OFF (see Note 6)

# Note 3: CRITICAL SAFETY CONSTRAINT (from manual page 19)
# OUTPUT_SPINDLE_ON (Bit 2) and OUTPUT_SAFETY_LINK_BRIDGE (Bit 12) cannot be used simultaneously.
# If one of them is turned on (set to 1), the other one should NOT be activated.
# To activate either output, the other one MUST be turned off (set to 0) first.
# The firmware will THROW AN ERROR if both are set to 1 simultaneously.
#
# Note 4: See "Sample application – Spindle control Option 1" and "Option 2" for details
# on J16/J20/J10 jumper configurations controlling spindle behavior.
#
# Note 5: J10-2 and J19 must be installed (short) to use OUTPUT_COVERS_LOCK_UNLOCK.
#
# Note 6: J21 must be installed (short) to enable software power control.


# =============================================================================
# Analog I/O Mappings (Page 19 of SK-2310g2 Manual)
# =============================================================================

# Analog Output (DAC)
ANALOG_OUT_SPINDLE_SPEED = 0  # CN6.11: 0-10V spindle speed command

# Analog Inputs (ADC)
ANALOG_IN_SPINDLE_LOAD = 0  # CN6.10: 0-10V spindle load feedback
ANALOG_IN_ADC2 = 1  # CN17.3: 0-5V general purpose
ANALOG_IN_ADC3 = 2  # CN17.2: 0-5V general purpose


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


# =============================================================================
# SK-2310g2 Device Class
# =============================================================================

import time
from typing import Optional, Dict, Any, List

# Import from parent package
from pyldcn.network import LDCNNetwork

# Import IOController base class
from .io import IOController, CMD_SET_OUTPUTS

# Import SK-2310g2 specific status management
from .status import SK2310g2Status


class SK2310g2(IOController):
    """
    SK-2310g2 I/O Controller

    Generic LDCN I/O controller device used in this application as a
    supervisory controller with safety and spindle control functions.

    """

    # Device constants
    EXPECTED_VERSION = 0x34

    def __init__(self, network: LDCNNetwork, address: int):
        """
        Initialize I/O controller.

        Args:
            network: Parent LDCNNetwork object
            address: Device address (1-127)

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        super().__init__(network, address)
        self.device_type = "SK-2310g2"

        # Status subsystem (replaces self.status_mask)
        self._status = SK2310g2Status(self)

        # Cached status values
        self.diagnostic_code: Optional[int] = None
        self.status_byte: Optional[int] = None
        self.power_state: Optional[bool] = None
        self.estop_state: Optional[bool] = None
        self.digital_inputs: Optional[int] = None
        self.digital_outputs: Optional[int] = None

    # -------------------------------------------------------------------------
    # IOController Abstract Method Implementations
    # -------------------------------------------------------------------------

    def get_pwm_channels(self) -> List[int]:
        """
        PWM available on channel 4 (OUTPUT_SPINDLE_DC_BRAKE).

        Returns:
            List containing [4]
        """
        return [4]

    def _validate_outputs(self, byte0: int, byte1: int) -> None:
        """
        Validate SK-2310g2 output constraints.

        Note 3 Safety Constraint:
        OUTPUT_SPINDLE_ON (bit 2) and OUTPUT_SAFETY_LINK_BRIDGE (bit 12)
        cannot be active simultaneously.

        Args:
            byte0: Output byte 0 (bits 0-7)
            byte1: Output byte 1 (bits 8-15)

        Raises:
            LDCNError: If safety constraint is violated
        """
        from pyldcn.network import LDCNError

        spindle_on = bool(byte0 & (1 << 2))  # OUTPUT_SPINDLE_ON = bit 2
        safety_bridge = bool(byte1 & (1 << 4))  # OUTPUT_SAFETY_LINK_BRIDGE = bit 12 (byte1 bit 4)

        if spindle_on and safety_bridge:
            raise LDCNError(
                "Safety constraint violated: OUTPUT_SPINDLE_ON (Bit 2) and "
                "OUTPUT_SAFETY_LINK_BRIDGE (Bit 12) cannot be active simultaneously. "
                "See Note 3 on page 19 of SK-2310g2 manual. "
                f"Current outputs: byte0=0x{byte0:02X}, byte1=0x{byte1:02X}"
            )

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------

    def define_status(self, status_mask: int) -> None:
        """
        Configure which status items are returned in NOP responses.

        Sends DEFINE_STATUS command to configure persistent status reporting.
        Delegates to status subsystem for state tracking.

        Args:
            status_mask: Bitmask of status items to include (see SK2310g2Status.status_items)

        Example:
            device.define_status(0x01)  # Only digital inputs
            device.define_status(0xFF)  # All status items
        """
        self._status.configure(status_mask)

    def configure(self) -> None:
        """
        Configure I/O controller for full status reporting.

        Sends DEFINE_STATUS with 0xFF (all 8 status items).
        See SK2310g2Status.status_items for item definitions.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        # Request all status items: digital_inputs, analog_in_0-2, counter_timer,
        # device_id, sync_inputs, sync_counter (bits 0-7, i.e., 0xFF)
        self._status.configure(0xFF)
        time.sleep(1.0)

    def hard_reset(self) -> None:
        """
        Send HARD_RESET command to device.

        Resets device to power-on defaults:
        - Clears DEFINE_STATUS configuration (returns to 0x00)
        - Resets all device state

        Updates status subsystem's mask to match device default (0x00).
        """
        from pyldcn.protocol import CMD_HARD_RESET

        # Send HARD_RESET command
        self.send_command(CMD_HARD_RESET, [])

        # Reset status subsystem's mask to match device default
        self._status.status_mask = 0x00

    # -------------------------------------------------------------------------
    # Status Reading
    # -------------------------------------------------------------------------

    def read_status(self) -> Dict:
        """
        Read complete I/O controller status.

        Returns complete status including SK-2310g2 specific digital inputs
        and system state (Byte0/Byte1 from supervisory controller documentation).

        Returns:
            {
                'status': status_byte,
                'device_id': int,
                'version': int,
                'byte0': int,              # SK-2310g2 digital inputs
                'byte1': int,              # SK-2310g2 internal status
                'analog_in_0/1/2': int,    # Analog input channels
                'diagnostic': int,         # Extracted from byte1 bits [7:3]
                'power_state': bool,
                # Decoded Byte0 fields:
                'input1': bool,
                'input2': bool,
                'spindle_stopped': bool,
                'spindle_fault': bool,
                'input3': bool,
                'input4': bool,
                'input5': bool,
                'input6': bool,
                # Decoded Byte1 fields:
                'safe_state': bool,
                'manual_override': bool,
                'servo_fault': bool,
            }
        """
        # Request all status items (0xFF = bits 0-7)
        status = self._status.read(0xFF)

        if not status:
            return {}

        # Update instance variables
        self.diagnostic_code = status.get("diagnostic", 0)
        self.status_byte = status.get("status", 0)
        self.power_state = status.get("power_state", False)

        return status

    def read_diagnostic(self) -> int:
        """
        Read diagnostic code (LED display value).

        Returns:
            Diagnostic code (0x00-0xFF)

        See page 20 of manual for diagnostic code table.
        Use decode_diagnostic() to get human-readable description.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        status = self.read_status()
        return status.get("diagnostic", 0)
    def decode_diagnostic(self, diag_code: Optional[int] = None) -> Dict[str, Any]:
        """
        Decode diagnostic code to human-readable system state.

        Uses the diagnostic table from page 20 of the SK-2310g2 manual.

        Args:
            diag_code: Diagnostic code to decode (if None, reads current)

        Returns:
            Dictionary with:
            - 'code': Diagnostic code (0x00-0xFF)
            - 'description': Human-readable description
            - 'power_enable': True if Power Enable is ON (TODO: Determine if this is a state - flashing power LED?)
            - 'power_relays': True if Power A & B relays are ON
            - 'motor_power_on': True if motor power is actually ON
            - 'byte1_bits': Dict of Byte1 status bits (bits 7,6,5,4,3)
            - 'led_states': List of LED numbers that should be lit
            - 'details': Additional state details

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if diag_code is None:
            diag_code = self.read_diagnostic()

        # Extract Byte1 bits from diagnostic code (bits 7,6,5,4,3)
        bit7 = (diag_code >> 7) & 0x01  # Power Enable
        bit6 = (diag_code >> 6) & 0x01
        bit5 = (diag_code >> 5) & 0x01
        bit4 = (diag_code >> 4) & 0x01
        bit3 = (diag_code >> 3) & 0x01

        # Diagnostic code lookup table from page 20
        DIAGNOSTIC_TABLE = {
            0x01: {
                "desc": "Initializing",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 2, 3, 4],
                "details": "System initialization in progress",
            },
            0x02: {
                "desc": "Control voltage shorted",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 2, 3, 5],
                "details": "Control voltage short circuit detected",
            },
            0x03: {
                "desc": "Output shorted",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 2, 3],
                "details": "Output short circuit detected",
            },
            0x04: {
                "desc": "Control voltage low (<18V)",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 2, 4, 5],
                "details": "Control voltage below 18V",
            },
            0x05: {
                "desc": "Safe / Manual switch malfunction (both contacts ON)",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 2, 4],
                "details": "Both Safe or Manual Mode switch contacts are ON",
            },
            0x06: {
                "desc": "Power-up Safe error",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 2, 5],
                "details": "Machine Safe error detected at power-up",
            },
            0x07: {
                "desc": "Power-up Manual Override Mode error",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 2],
                "details": "Manual Override Mode switch error detected at power-up",
            },
            0x08: {
                "desc": "System Locked",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 3, 4, 5],
                "details": "System locked via software command",
            },
            0x09: {
                "desc": "Watchdog Timeout",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 3, 4],
                "details": "Watchdog timer expired",
            },
            0x0A: {
                "desc": "Safety Link Error",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 3, 5],
                "details": "Safety Link daisy-chain broken. Check each device status",
            },
            0x0B: {
                "desc": "Guard Open Stop (Spindle not stopped)",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 3],
                "details": "Guard open while spindle was moving",
            },
            0x0C: {
                "desc": "Guard Open Stop (machine not safe)",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 4, 5],
                "details": "Guard open and machine not safe",
            },
            0x0D: {
                "desc": "Guard Open Stop (Manual mode without Acknowledge)",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 4],
                "details": "Guard open in Manual mode without Acknowledge pressed",
            },
            0x0E: {
                "desc": "Guard contact Fault",
                "power_enable": False,
                "power_ab": False,
                "leds": [1, 5],
                "details": "One or more guard contacts malfunctioning",
            },
            0x0F: {
                "desc": "Limit Switch Stop",
                "power_enable": False,
                "power_ab": False,
                "leds": [1],
                "details": "Limit switch activated",
            },
            0x10: {
                "desc": "Emergency Stop",
                "power_enable": False,
                "power_ab": False,
                "leds": [2, 3, 4, 5],
                "details": "Emergency stop button pressed",
            },
            0x11: {
                "desc": "Emergency Stop contact malfunction or Monitor Loop open",
                "power_enable": False,
                "power_ab": False,
                "leds": [2, 3, 4],
                "details": "E-stop contact failure or relay monitor loop open",
            },
            0x12: {
                "desc": "Power ON button busy (>6sec) or Monitor Loop open",
                "power_enable": False,
                "power_ab": False,
                "leds": [2, 3, 5],
                "details": "Power button held >6sec or safety relay fault",
            },
            0x13: {
                "desc": "Motor Power Supply under-voltage",
                "power_enable": True,
                "power_ab": True,
                "leds": [2, 3],
                "details": "Motor power supply (UM) voltage too low",
            },
            0x14: {
                "desc": "Guards open (ready to power)",
                "power_enable": False,
                "power_ab": False,
                "leds": [2, 4, 5],
                "details": "Both guards open, Power LED flashing",
            },
            0x15: {
                "desc": "Guard 1 closed, Guard 2 open (ready to power)",
                "power_enable": False,
                "power_ab": False,
                "leds": [2, 4],
                "details": "Guard 1 closed, Guard 2 open, Power LED flashing",
            },
            0x16: {
                "desc": "Guard 1 open, Guard 2 closed (ready to power)",
                "power_enable": False,
                "power_ab": False,
                "leds": [2, 5],
                "details": "Guard 1 open, Guard 2 closed, Power LED flashing",
            },
            0x17: {
                "desc": "Both Guards closed (ready to power)",
                "power_enable": False,
                "power_ab": False,
                "leds": [2],
                "details": "Both guards closed, Power LED flashing",
            },
            0x18: {
                "desc": "Both guards open, Manual mode enabled",
                "power_enable": True,
                "power_ab": True,
                "leds": [3, 4, 5],
                "details": "Manual mode active with guards open",
            },
            0x19: {
                "desc": "Guard 1 closed, Guard 2 open, Manual mode enabled",
                "power_enable": True,
                "power_ab": True,
                "leds": [3, 4],
                "details": "Manual mode: Guard 1 closed, Guard 2 open",
            },
            0x1A: {
                "desc": "Guard 1 open, Guard 2 closed, Manual mode enabled",
                "power_enable": True,
                "power_ab": True,
                "leds": [3, 5],
                "details": "Manual mode: Guard 1 open, Guard 2 closed",
            },
            0x1B: {
                "desc": "Both guards closed, Manual Mode",
                "power_enable": True,
                "power_ab": True,
                "leds": [3],
                "details": "Manual mode active with both guards closed",
            },
            0x1C: {
                "desc": "Machine safe, spindle stopped, guards open",
                "power_enable": True,
                "power_ab": True,
                "leds": [4, 5],
                "details": "Machine safe, spindle stopped, guards open",
            },
            0x1D: {
                "desc": "Machine safe, spindle stopped, Guard 1 closed, Guard 2 open",
                "power_enable": True,
                "power_ab": True,
                "leds": [4],
                "details": "Machine safe with Guard 1 closed, Guard 2 open",
            },
            0x1E: {
                "desc": "Machine safe, spindle stopped, Guard 1 open, Guard 2 closed",
                "power_enable": True,
                "power_ab": True,
                "leds": [5],
                "details": "Machine safe with Guard 1 open, Guard 2 closed",
            },
            0x1F: {
                "desc": "Normal operation - Guards closed",
                "power_enable": True,
                "power_ab": True,
                "leds": [],
                "details": "All systems ready, guards closed, powered",
            },
            0x00: {
                "desc": "Power OFF delay in progress",
                "power_enable": False,
                "power_ab": True,
                "leds": [1, 2, 3, 4, 5],
                "details": "Power OFF command delay (per J2 setting)",
            },
        }

        # Lookup diagnostic info
        diag_info = DIAGNOSTIC_TABLE.get(
            diag_code,
            {
                "desc": f"Unknown diagnostic code 0x{diag_code:02X}",
                "power_enable": bit7 == 1,
                "power_ab": (bit7 == 1 and bit6 == 0) or diag_code == 0x00,
                "leds": [],
                "details": "See manual page 20 for this code",
            },
        )

        # Motor power is ON when BOTH Power Enable AND Power A&B relays are ON
        motor_power_on = diag_info["power_enable"] and diag_info["power_ab"]

        return {
            "code": diag_code,
            "code_hex": f"0x{diag_code:02X}",
            "code_binary": f"{diag_code:08b}",
            "description": diag_info["desc"],
            "details": diag_info["details"],
            "power_enable": diag_info["power_enable"],
            "power_relays_on": diag_info["power_ab"],
            "motor_power_on": motor_power_on,
            "byte1_bits": {
                "bit7_power_enable": bool(bit7),
                "bit6": bool(bit6),
                "bit5": bool(bit5),
                "bit4": bool(bit4),
                "bit3": bool(bit3),
            },
            "led_states": diag_info["leds"],
        }

    def get_system_state(self) -> Dict[str, Any]:
        """
        Get comprehensive system state combining diagnostic and I/O status.

        Returns:
            Dictionary with complete system state:
            - 'diagnostic': Decoded diagnostic information
            - 'motor_power_on': True if motor power relays are energized
            - 'spindle_status': Spindle state (off/fault/at_speed)
            - 'safety_status': Safety system state
            - 'io_status': Digital I/O states
            - 'guard_status': Door/guard states
            - 'system_ready': True if system is operational

        This provides a complete snapshot of machine state.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        # Read diagnostic
        diagnostic = self.decode_diagnostic()

        # Read digital inputs
        io_states = self.read_input_states()

        # Read spindle status
        spindle = self.read_spindle_status()

        # Determine overall system readiness
        system_ready = (
            diagnostic["motor_power_on"]
            and not io_states["servo_fault"]
            and not spindle["fault"]
        )

        return {
            "diagnostic": diagnostic,
            "motor_power_on": diagnostic["motor_power_on"],
            "spindle_status": {
                "off": spindle["off"],
                "fault": spindle["fault"],
                "at_speed": spindle["at_speed"],
                "load_voltage": spindle.get("load_voltage"),
            },
            "safety_status": {
                "at_home": io_states["at_home"],
                "test_mode": io_states["test_mode"],
                "servo_fault": io_states["servo_fault"],
            },
            "io_status": {
                "air_pressure_ok": io_states["air_pressure_ok"],
                "tool_length_switch": io_states["tool_length_switch"],
                "tool_changer_closed": io_states["tool_changer_closed"],
            },
            "system_ready": system_ready,
        }

    # -------------------------------------------------------------------------
    # SK-2310g2 Specific Methods
    # -------------------------------------------------------------------------

    # Compatibility property for 16-bit output interface
    @property
    def digital_outputs(self) -> Optional[int]:
        """Get current 16-bit output state."""
        if self._output_byte0 == 0 and self._output_byte1 == 0:
            return None
        return (self._output_byte1 << 8) | self._output_byte0

    @digital_outputs.setter
    def digital_outputs(self, value: Optional[int]) -> None:
        """Set 16-bit output state (for compatibility)."""
        if value is None:
            self._output_byte0 = 0x00
            self._output_byte1 = 0x00
        else:
            self._output_byte0 = value & 0xFF
            self._output_byte1 = (value >> 8) & 0xFF

    def set_outputs_16bit(self, outputs: int, validate_safety: bool = True) -> None:
        """
        Set outputs using 16-bit value (compatibility method).

        Args:
            outputs: 16-bit output state
            validate_safety: If True, validate constraints (ignored, always validated)
        """
        byte0 = outputs & 0xFF
        byte1 = (outputs >> 8) & 0xFF
        self.set_outputs(byte0, byte1)

    def read_power_state(self) -> bool:
        """
        Read power button state from status bit 3.

        Returns:
            True if power ON, False if power OFF

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        status = self.read_status()
        return status.get("power_state", False)

    # -------------------------------------------------------------------------
    # Power and Safety Monitoring
    # -------------------------------------------------------------------------

    def power_on(self, timeout: Optional[float] = None, verbose: bool = True) -> bool:
        """
        Turn on motor power using software control or physical button.

        Attempts software power-on by toggling bit 15 (System Lock/Power ON/OFF).
        If software control fails, prompts user to press physical power button.

        Sequence:
        1. Check if already powered on
        2. Attempt software power-on (toggle bit 15 for 100ms)
        3. Check if power state changed to ON (diagnostic 0x13 or 0x18-0x1F)
        4. If not, prompt user to press physical button
        5. Wait for power ON with timeout

        Args:
            timeout: Maximum wait time for power-on (seconds, None = infinite)
            verbose: If True, print status updates

        Returns:
            True if power successfully turned on, False if timeout

        Note:
            Software control requires J21 shorted (pins 1-2).
            With J21 open (default), only physical button works.
        """
        # Check if already powered on
        if self.read_power_state():
            if verbose:
                print("✓ Power already ON")
            return True

        if verbose:
            print("Attempting software power-on (bit 15 toggle)...")

        # Attempt software power-on: toggle bit 15
        try:
            # Get current outputs (or start with 0 if unknown)
            current_outputs = (
                self.digital_outputs if self.digital_outputs is not None else 0
            )

            # Set bit 15 HIGH
            outputs_high = current_outputs | (1 << OUTPUT_SYSTEM_LOCK)
            byte0 = outputs_high & 0xFF
            byte1 = (outputs_high >> 8) & 0xFF
            self.set_outputs(byte0, byte1)
            time.sleep(0.1)  # Hold for 100ms

            # Set bit 15 LOW
            outputs_low = current_outputs & ~(1 << OUTPUT_SYSTEM_LOCK)
            byte0 = outputs_low & 0xFF
            byte1 = (outputs_low >> 8) & 0xFF
            self.set_outputs(byte0, byte1)
            time.sleep(0.2)  # Wait for state change

            # Check if power turned on
            if self.read_power_state():
                if verbose:
                    print("✓ Software power-on successful!")
                return True

            if verbose:
                print("  Software control failed (J21 may be open)")

        except Exception as e:
            if verbose:
                print(f"  Software control failed: {e}")

        # Fall back to physical button
        self.request_power_on()
        return self.wait_for_power_button(
            timeout=timeout, poll_rate=0.1, verbose=verbose
        )

    def request_power_on(
        self, message: str = "Please press the POWER button to continue..."
    ) -> None:
        """
        Display operator notification requesting power button press.

        This is a helper method to provide clear feedback to the operator
        before calling wait_for_power_button().

        Args:
            message: Custom message to display (default: standard request)

        Note: With J21 open (default), power can ONLY be enabled via physical button.
              To enable software control, short J21 pins 1-2 on the SK2310g2.
        """
        print(f"\n{'=' * 60}")
        print(f"POWER REQUIRED")
        print(f"{'=' * 60}")
        print(f"{message}")
        print(f"Waiting for power state change...")
        print(f"{'=' * 60}\n")

    def wait_for_power_button(
        self,
        timeout: Optional[float] = None,
        poll_rate: float = 0.1,
        verbose: bool = False,
    ) -> bool:
        """
        Wait for power button press detection.

        Continuously monitors power state until transition from OFF to ON.

        Args:
            timeout: Maximum wait time (None = infinite)
            poll_rate: Status polling rate (seconds)
            verbose: If True, print status updates while waiting

        Returns:
            True if power button pressed, False if timeout

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        start_time = time.time()
        last_status_time = 0.0

        while True:
            power_state = self.read_power_state()

            if power_state:
                if verbose:
                    print("\n✓ Power ON detected!")
                return True

            # Print periodic status if verbose
            if verbose:
                current_time = time.time()
                if current_time - last_status_time >= 2.0:  # Every 2 seconds
                    elapsed = current_time - start_time
                    print(f"  Waiting... ({elapsed:.1f}s elapsed)")
                    last_status_time = current_time

            if timeout is not None:
                elapsed = time.time() - start_time
                if elapsed >= timeout:
                    if verbose:
                        print(f"\n✗ Timeout after {elapsed:.1f}s")
                    return False

            time.sleep(poll_rate)

    def read_estop_state(self) -> bool:
        """
        Read emergency stop state (dual line monitoring).

        Returns:
            True if E-stop is OK, False if E-stop is active

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD - depends on I/O channel mapping
        """
        # TBD: Read appropriate digital inputs
        raise NotImplementedError(
            "E-stop monitoring not yet implemented - I/O mapping TBD"
        )

    def read_guard_state(self) -> bool:
        """
        Read work zone guard state (guarded area contacts).

        Returns:
            True if guards closed (safe), False if any guard open

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD - depends on I/O channel mapping
        """
        # TBD: Read appropriate digital inputs
        raise NotImplementedError(
            "Guard monitoring not yet implemented - I/O mapping TBD"
        )

    def read_safe_zone_state(self) -> bool:
        """
        Read safe zone sensor state (dual sensor interface).

        Returns:
            True if safe zone clear, False if zone occupied

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD - depends on I/O channel mapping
        """
        # TBD: Read appropriate digital inputs
        raise NotImplementedError(
            "Safe zone monitoring not yet implemented - I/O mapping TBD"
        )

    # -------------------------------------------------------------------------
    # Digital Input Reading
    # -------------------------------------------------------------------------

    def read_digital_inputs(self) -> int:
        """
        Read all 16 digital input states.

        Returns:
            16-bit digital input value

        Input mapping (SK-2310g2) from page 19:
        - Byte 0 (bits 0-7): Application inputs
          - Bit 0: Program run
          - Bit 1: Program stop
          - Bit 2: Spindle OFF (computed)
          - Bit 3: Spindle fault
          - Bit 4: Spindle at speed
          - Bit 5: Air pressure OK
          - Bit 6: Tool length measurement switch
          - Bit 7: Tool changer closed

        - Byte 1 (bits 8-15): System status
          - Bit 8: At home/safe zone
          - Bit 9: Test mode active
          - Bit 10: Servo fault
          - Bits 11-15: Status LEDs

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        status = self.read_status()
        return status.get("digital_inputs", 0)

    def get_input_bit(self, bit: int) -> bool:
        """
        Read a single digital input bit.

        Args:
            bit: Input bit number (0-15)

        Returns:
            True if input is HIGH, False if LOW

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        inputs = self.read_digital_inputs()
        return self._get_bit(inputs, bit)

    def read_input_states(self) -> Dict[str, bool]:
        """
        Read all digital inputs and return as named dictionary.

        Returns:
            Dictionary mapping input names to boolean states

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        inputs = self.read_digital_inputs()

        return {
            # Application inputs (Byte0)
            "program_run": self._get_bit(inputs, INPUT_PROGRAM_RUN),
            "program_stop": self._get_bit(inputs, INPUT_PROGRAM_STOP),
            "spindle_off": self._get_bit(inputs, INPUT_SPINDLE_OFF),
            "spindle_fault": self._get_bit(inputs, INPUT_SPINDLE_FAULT),
            "spindle_at_speed": self._get_bit(inputs, INPUT_SPINDLE_AT_SPEED),
            "air_pressure_ok": self._get_bit(inputs, INPUT_AIR_PRESSURE),
            "tool_length_switch": self._get_bit(inputs, INPUT_TOOL_LENGTH_SWITCH),
            "tool_changer_closed": self._get_bit(inputs, INPUT_TOOL_CHANGER_CLOSED),
            # System status (Byte1)
            "at_home": self._get_bit(inputs, INPUT_AT_HOME),
            "test_mode": self._get_bit(inputs, INPUT_TEST_MODE),
            "servo_fault": self._get_bit(inputs, INPUT_SERVO_FAULT),
        }

    # -------------------------------------------------------------------------
    # Analog I/O
    # -------------------------------------------------------------------------

    def read_analog_inputs(self) -> Dict[int, float]:
        """
        Read all analog input values (3 channels).

        Returns:
            Dictionary of {channel: voltage} pairs

        Channels:
        - 0: CN6.10 - Spindle load feedback (0-10V)
        - 1: CN17.3 - ADC2 general purpose (0-5V)
        - 2: CN17.2 - ADC3 general purpose (0-5V)

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Status response format needs to be determined from hardware testing
        """
        status = self.read_status()
        # TBD: Extract analog values from status response
        # This depends on the actual response format which needs hardware verification
        return {
            0: 0.0,  # Spindle load
            1: 0.0,  # ADC2
            2: 0.0,  # ADC3
        }

    def set_analog_output(self, channel: int, voltage: float) -> None:
        """
        Set analog output voltage.

        Args:
            channel: Output channel (0 = CN6.11 spindle speed)
            voltage: Output voltage (0.0 - 10.0V)

        Raises:
            ValueError: If voltage out of range or invalid channel

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Command format needs to be determined from hardware testing
        """
        if channel != 0:
            raise ValueError(
                f"Invalid analog output channel: {channel}. Only channel 0 (spindle speed) is supported."
            )

        if not 0.0 <= voltage <= 10.0:
            raise ValueError(f"Voltage {voltage}V out of range. Must be 0.0-10.0V")

        # TBD: Implement actual analog output command
        # This may use CMD_SET_PWM_IO or a different command
        # Need to verify with hardware
        raise NotImplementedError(
            "Analog output command format TBD - needs hardware verification"
        )

    # -------------------------------------------------------------------------
    # Spindle Control (Connected LS2315 via CN6 → CN7)
    # -------------------------------------------------------------------------

    def set_spindle_speed_voltage(self, voltage: float) -> None:
        """
        Set spindle speed via 0-10V analog output to LS2315.

        The LS2315 interprets the voltage as speed command:
        - 0V = stopped
        - 10V = maximum RPM (50K/60K/100K depending on DIP switch)

        Args:
            voltage: Speed command voltage (0.0 - 10.0V)

        Raises:
            ValueError: If voltage out of range

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.set_analog_output(ANALOG_OUT_SPINDLE_SPEED, voltage)

    def set_spindle_speed_percent(self, speed_percent: float) -> None:
        """
        Set spindle speed as percentage of maximum.

        Args:
            speed_percent: Speed as percentage (0.0 - 100.0)

        Raises:
            ValueError: If speed_percent out of range

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if not 0.0 <= speed_percent <= 100.0:
            raise ValueError(f"Speed {speed_percent}% out of range. Must be 0.0-100.0%")

        voltage = (speed_percent / 100.0) * 10.0
        self.set_spindle_speed_voltage(voltage)

    def enable_spindle(self, direction: str = "forward") -> None:
        """
        Enable spindle via OUTPUT_SPINDLE_ON and set direction.

        This sets:
        - OUTPUT_SPINDLE_ON (Bit 2) = 1  (enables LS2315)
        - OUTPUT_SPINDLE_DIRECTION (Bit 3) = 0 for CW, 1 for CCW

        Hardware behavior (from manual page 7, CN6 description):
        - CN6.5 (Spindle ON) goes HIGH → LS2315 CN7.5 SpindleENABLE
        - CN6.6 (Direction) controls → LS2315 CN7.6 SpindleREVERSE

        Jumper Configuration (Option 1 - this machine):
        - J16 2-3 short: Spindle ON disabled when guards are open
        - J20 open: Spindle must be stopped before guard unlock in Manual mode
        - J10.3 open: Spindle operation NOT enabled in Manual mode

        Args:
            direction: 'forward' (CW) or 'reverse' (CCW)

        Raises:
            ValueError: If invalid direction
            LDCNError: If safety constraint violated (Note 3)

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if direction not in ("forward", "reverse"):
            raise ValueError(
                f"Invalid direction: {direction}. Must be 'forward' or 'reverse'"
            )

        # Get current output state
        byte0, byte1 = self.get_output_state()

        # Set spindle enable and direction bits (both in byte0)
        byte0 = self._set_bit(byte0, OUTPUT_SPINDLE_ON, True)
        byte0 = self._set_bit(byte0, OUTPUT_SPINDLE_DIRECTION, direction == "reverse")

        # This will validate Note 3 constraint automatically
        self.set_outputs(byte0, byte1)

    def disable_spindle(self) -> None:
        """
        Disable spindle by clearing OUTPUT_SPINDLE_ON.

        This clears:
        - OUTPUT_SPINDLE_ON (Bit 2) = 0  (disables LS2315)

        The LS2315 will decelerate according to its internal ramp settings.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        # Get current output state
        byte0, byte1 = self.get_output_state()

        # Clear spindle enable bit
        byte0 = self._set_bit(byte0, OUTPUT_SPINDLE_ON, False)

        self.set_outputs(byte0, byte1)

    def read_spindle_status(self) -> Dict[str, Any]:
        """
        Read spindle status from digital inputs.

        Returns dictionary with:
        - 'off': Spindle OFF (computed from Spindle ON output and Stopped input)
        - 'fault': Spindle fault active
        - 'at_speed': Spindle at commanded speed
        - 'load': Spindle load (0-10V analog feedback, if available)

        Note 1 from manual page 19:
        Spindle OFF = 1 when:
          - Spindle ON output (Outputs/Byte0/Bit2) = 0 AND
          - Spindle Stopped input (CN6 pin2) = HIGH

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        inputs = self.read_digital_inputs()

        status = {
            "off": self._get_bit(inputs, INPUT_SPINDLE_OFF),
            "fault": self._get_bit(inputs, INPUT_SPINDLE_FAULT),
            "at_speed": self._get_bit(inputs, INPUT_SPINDLE_AT_SPEED),
        }

        # Add analog load feedback if available
        try:
            analog = self.read_analog_inputs()
            status["load_voltage"] = analog.get(ANALOG_IN_SPINDLE_LOAD, 0.0)
        except Exception:
            status["load_voltage"] = None

        return status

    # -------------------------------------------------------------------------
    # Guard Lock Control (Schmersal AZM170-02ZK-2321 Door Switch)
    # -------------------------------------------------------------------------

    def lock_guard(self) -> None:
        """
        Engage guard lock.

        Sets OUTPUT_GUARD_LOCK (Bit 9) = 1 to lock the Schmersal safety
        switch. This prevents the door from being opened.

        The Schmersal AZM170-02ZK-2321 integrated lock is controlled via
        CN9.3-4 (Guard1 Unlock solenoid +/-).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.set_output_bit(OUTPUT_GUARD_LOCK, True)

    def unlock_guard(self) -> None:
        """
        Release guard lock.

        Sets OUTPUT_GUARD_LOCK (Bit 9) = 0 to unlock the Schmersal safety
        switch, allowing the door to open if unlock conditions are met:

        Unlock conditions (from manual page 7, CN9 description):
        - Spindle stopped AND Power is OFF, OR
        - Spindle stopped AND machine in Safety Zone, OR
        - Manual mode with Acknowledge (J20 setting dependent)

        Current jumper config (Option 1):
        - J20 open: Spindle must be stopped for unlock in Manual mode

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.set_output_bit(OUTPUT_GUARD_LOCK, False)

    def read_guard_lock_state(self) -> Dict[str, bool]:
        """
        Read guard/door safety switch state.

        Returns:
            Dictionary with:
            - 'closed': True if both A and B contacts indicate door closed
            - 'locked': True if guard lock output is active

        The Schmersal AZM170-02ZK-2321 provides dual redundant contacts
        (A and B channels) connected to CN9.1-2 and CN9.5-6.

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  This method needs guard contact input bit mapping - TBD
        """
        # TBD: Need to determine which input bits correspond to guard contacts
        # Manual shows CN9.1-2 (Guard1 A) and CN9.5-6 (Guard1 B)
        # but doesn't specify the input bit mapping in Inputs/Byte0 or Byte1

        outputs = self.digital_outputs if self.digital_outputs is not None else 0
        locked = self._get_bit(outputs, OUTPUT_GUARD_LOCK)

        return {
            "closed": False,  # TBD - need input bit mapping
            "locked": locked,
        }

    # -------------------------------------------------------------------------
    # Tool Changer & Pneumatic Control
    # -------------------------------------------------------------------------

    def read_air_pressure_ok(self) -> bool:
        """
        Read air pressure OK status from INPUT_AIR_PRESSURE (Bit 5).

        Connected to CN7.1 (Input 5) - indicates sufficient air pressure
        for pneumatic tool changer operations.

        Returns:
            True if air pressure is sufficient, False otherwise

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        return self.get_input_bit(INPUT_AIR_PRESSURE)

    def read_tool_length_switch(self) -> bool:
        """
        Read tool length measurement switch state.

        Connected to CN7.3 (Input 6) - indicates tool length probe contact.

        Returns:
            True if switch is activated, False otherwise

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        return self.get_input_bit(INPUT_TOOL_LENGTH_SWITCH)

    def read_tool_changer_closed(self) -> bool:
        """
        Read tool changer cover closed state.

        Connected to CN7.5 (Input 7).

        Returns:
            True if tool changer cover is closed, False otherwise

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        return self.get_input_bit(INPUT_TOOL_CHANGER_CLOSED)

    def set_tool_clamp(self, state: bool) -> None:
        """
        Control tool clamp solenoid.

        Args:
            state: True to clamp tool, False to release

        Connected to CN7.7 (Output 5).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.set_output_bit(OUTPUT_TOOL_CLAMP, state)

    def set_solenoid_valve_2(self, state: bool) -> None:
        """
        Control solenoid valve 2.

        Args:
            state: True to energize, False to de-energize

        Connected to CN7.9 (Output 6).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.set_output_bit(OUTPUT_SOLENOID_VALVE_2, state)

    def set_solenoid_valve_3(self, state: bool) -> None:
        """
        Control solenoid valve 3.

        Args:
            state: True to energize, False to de-energize

        Connected to CN7.11 (Output 7).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.set_output_bit(OUTPUT_SOLENOID_VALVE_3, state)

    def unlock_tool_changer(self) -> None:
        """
        Unlock tool changer cover.

        Sets OUTPUT_TOOL_CHANGER_UNLOCK (Bit 8) = 1.
        Connected to CN7.13 (Output 8).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.set_output_bit(OUTPUT_TOOL_CHANGER_UNLOCK, True)

    def lock_tool_changer(self) -> None:
        """
        Lock tool changer cover.

        Sets OUTPUT_TOOL_CHANGER_UNLOCK (Bit 8) = 0.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.set_output_bit(OUTPUT_TOOL_CHANGER_UNLOCK, False)
