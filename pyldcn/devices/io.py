"""
SK-2310g2 I/O Controller Device

Generic LDCN I/O controller used for supervisory control with safety
and spindle control functions.

Author: NickyDoes
License: GPL v2 or later
"""

import time
from typing import Optional, Dict, Any

# Import from parent package
from pyldcn.network import (
    LDCNDevice, LDCNNetwork,
    CMD_READ_STATUS,
    STATUS_POWER_ON,
)



# =============================================================================
# I/O Controller Commands
# =============================================================================

CMD_SET_PWM_IO = 0x04          # Set PWM duty cycle
CMD_SYNCH_OUTPUT = 0x05        # Apply staged outputs
CMD_SET_OUTPUTS = 0x06         # Set all output states
CMD_SET_SYNCH_OUTPUT = 0x07    # Stage outputs for sync
CMD_SET_TIMER_MODE = 0x08      # Configure counter/timer
CMD_SYNCH_INPUT = 0x0C         # Capture inputs atomically


# =============================================================================
# Digital Input Bit Mappings (Page 19 of SK-2310g2 Manual)
# =============================================================================
# Format: 16-bit word = Byte1:Byte0 (MSB:LSB)
# CN6/CN7/CN14 physical inputs → Inputs/Byte0/Bits 0-7
# Internal status signals → Inputs/Byte1/Bits 0-7

# Byte0 - Application Inputs
INPUT_PROGRAM_RUN = 0          # Bit 0: CN14 - Program run signal
INPUT_PROGRAM_STOP = 1         # Bit 1: CN14 - Program stop signal
INPUT_SPINDLE_OFF = 2          # Bit 2: CN6 - Spindle OFF (computed, see Note 1)
INPUT_SPINDLE_FAULT = 3        # Bit 3: CN6 - Spindle fault status
INPUT_SPINDLE_AT_SPEED = 4     # Bit 4: CN6 - Spindle at speed status
INPUT_AIR_PRESSURE = 5         # Bit 5: CN7 - Air pressure OK
INPUT_TOOL_LENGTH_SWITCH = 6   # Bit 6: CN7 - Tool length measurement switch
INPUT_TOOL_CHANGER_CLOSED = 7  # Bit 7: CN7 - Tool changer cover closed

# Byte1 - System Status Inputs
INPUT_AT_HOME = 8              # Bit 0: System at home/safe zone
INPUT_TEST_MODE = 9            # Bit 1: Test mode active
INPUT_SERVO_FAULT = 10         # Bit 2: CN3 Safety Bus - Servo fault signal
INPUT_STATUS_LED1 = 11         # Bit 3: LED1 status
INPUT_STATUS_LED2 = 12         # Bit 4: LED2 status
INPUT_STATUS_LED3 = 13         # Bit 5: LED3 status
INPUT_STATUS_LED4 = 14         # Bit 6: LED4 status
INPUT_STATUS_LED5 = 15         # Bit 7: LED5 status

# Note 1: INPUT_SPINDLE_OFF computation (from manual page 19)
# Spindle OFF = 1 when:
#   - Spindle ON output (Outputs/Byte0/Bit2) = 0 AND
#   - Spindle Stopped input (CN6 pin2) = HIGH


# =============================================================================
# Digital Output Bit Mappings (Page 19 of SK-2310g2 Manual)
# =============================================================================
# Format: 16-bit word = Byte1:Byte0 (MSB:LSB)

# Byte0 - Application Outputs
OUTPUT_PROGRAM_RUNNING_LAMP = 0    # Bit 0: CN14 - Program running lamp
OUTPUT_PROGRAM_STOPPED_LAMP = 1    # Bit 1: CN14 - Program stopped lamp
OUTPUT_SPINDLE_ON = 2              # Bit 2: CN6/CN16 - Spindle ON (see Note 3, Note 4)
OUTPUT_SPINDLE_DIRECTION = 3       # Bit 3: CN6 - Spindle direction (0=CW, 1=CCW)
OUTPUT_SPINDLE_DC_BRAKE = 4        # Bit 4: CN6 - Spindle DC-braking or PWM
OUTPUT_TOOL_CLAMP = 5              # Bit 5: CN7 - Tool clamp solenoid
OUTPUT_SOLENOID_VALVE_2 = 6        # Bit 6: CN7 - Solenoid valve 2
OUTPUT_SOLENOID_VALVE_3 = 7        # Bit 7: CN7 - Solenoid valve 3

# Byte1 - System Control Outputs
OUTPUT_TOOL_CHANGER_UNLOCK = 8     # Bit 0: CN7 - Tool changer cover unlock
OUTPUT_GUARD_LOCK = 9              # Bit 1: CN9/CN10 - Guard lock control
OUTPUT_HOME_ENABLE = 10            # Bit 2: See automation modes / Home enable
OUTPUT_TEST_MODE_INHIBIT = 11      # Bit 3: Test mode inhibit control
OUTPUT_SAFETY_LINK_BRIDGE = 12     # Bit 4: CN3 Safety Bus - Safety Link Bridge (see Note 3)
OUTPUT_INVERTED_13 = 13            # Bit 5: CN7 - Inverted output (HIGH when bit=0)
OUTPUT_COVERS_LOCK_UNLOCK = 14     # Bit 6: Reserved / Covers lock/unlock (see Note 5)
OUTPUT_SYSTEM_LOCK = 15            # Bit 7: System lock / Power ON/OFF (see Note 6)

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
ANALOG_OUT_SPINDLE_SPEED = 0   # CN6.11: 0-10V spindle speed command

# Analog Inputs (ADC)
ANALOG_IN_SPINDLE_LOAD = 0     # CN6.10: 0-10V spindle load feedback
ANALOG_IN_ADC2 = 1             # CN17.3: 0-5V general purpose
ANALOG_IN_ADC3 = 2             # CN17.2: 0-5V general purpose


class SK2310g2(LDCNDevice):
    """
    SK-2310g2 I/O Controller

    Generic LDCN I/O controller device used in this application as a
    supervisory controller with safety and spindle control functions.

    Hardware Capabilities:
    - Dual mechanical relay power control
    - Spindle control with spindle enable mechanical relay (CN6)
    - Dual line emergency stop monitoring
    - Dual work zone "covers" contacts (guarded area monitoring)
    - Dual safe zone sensor interface
    - 3 analog inputs (0-5V or 0-10V)
    - 1 analog output (CN6.11 is 0-10V spindle speed control)
    - Digital I/O (16 inputs, 16 outputs)

    Hardware Wiring (This Machine):
    ===================================

    SPINDLE CONTROL (CN6 → LS2315 CN7 pin-for-pin):
    - CN6.2:  Spindle Stopped (from LS2315 CN7.2/8)
    - CN6.3:  Spindle Fault (from LS2315 CN7.3)
    - CN6.4:  Spindle At Speed (from LS2315 CN7.4)
    - CN6.5:  Spindle ON enable (to LS2315 CN7.5)
    - CN6.6:  Spindle Direction/Reverse (to LS2315 CN7.6)
    - CN6.7:  Spindle DC-braking/PWM (to LS2315 CN7.7)
    - CN6.10: Spindle LOAD analog feedback 0-10V (from LS2315 CN7.10)
    - CN6.11: Spindle SPEED command 0-10V (to LS2315 CN7.11)

    LS2315 also connects to Safety Bus via CN4/CN5 but is NOT an LDCN device.
    See LS-2315-High-Performance-Spindle-Drive.pdf for full pinout details.

    DOOR SAFETY SWITCH (CN9 → Schmersal AZM170-02ZK-2321):
    - CN9.1-2:  Cover1 A contacts (dual-channel safety monitoring)
    - CN9.5-6:  Cover1 B contacts (dual-channel safety monitoring)
    - CN9.3-4:  Cover1 Unlock solenoid (+/-) - releases lock in Schmersal switch

    The Schmersal AZM170-02ZK-2321 is a safety door switch with:
      • Dual redundant safety contacts (A and B channels)
      • Integrated electromagnetic lock solenoid
      • Lock release via CN9.3-4 when unlock conditions are met:
        - Spindle stopped AND Power is OFF, OR
        - Spindle stopped AND machine in Safety Zone, OR
        - Test Mode with Acknowledge (J20 setting dependent)

    Jumper J16 2-3 short ensures Spindle ON is disabled when guards are open.
    Jumper J20 open requires spindle stopped before guard unlock in Test Mode.
    See pages 11-12 of SK-2310g2 manual for complete jumper configurations.

    TOOL CHANGER & PNEUMATIC CONTROL (CN7):
    - CN7.1 (Input 5):  Air Pressure OK sensor
    - CN7.3 (Input 6):  Tool Length Measurement Switch
    - CN7.5 (Input 7):  Tool Changer Cover Closed
    - CN7.7 (Output 5): Tool Clamp Solenoid
    - CN7.9 (Output 6): Solenoid Valve 2
    - CN7.11 (Output 7): Solenoid Valve 3
    - CN7.13 (Output 8): Tool Changer Unlock

    Additional Attributes:
        diagnostic_code: Last diagnostic code (LED display, see page 20)
        power_state: Power button state
        estop_state: Emergency stop state
        digital_inputs: 16-bit digital input state
        digital_outputs: 16-bit digital output state

    Safety Notes:
    =============
    ⚠️  OUTPUT_SPINDLE_ON (Bit 2) and OUTPUT_SAFETY_LINK_BRIDGE (Bit 12)
        CANNOT be active simultaneously (Note 3, page 19). Software will
        validate this constraint and raise LDCNError if violated.

    ⚠️  Current configuration uses Spindle Control Option 1:
        J16 2-3 short, J20 open, J10.3 open
        This provides maximum safety - spindle disabled when covers open.
    """

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

        # Cached status values
        self.diagnostic_code: Optional[int] = None
        self.status_byte: Optional[int] = None
        self.power_state: Optional[bool] = None
        self.estop_state: Optional[bool] = None
        self.digital_inputs: Optional[int] = None
        self.digital_outputs: Optional[int] = None

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------

    def configure(self) -> None:
        """
        Configure I/O controller for full status reporting.

        Sends DEFINE_STATUS with 0xFFFF (all status data).

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        self.define_status(0xFFFF)
        time.sleep(1.0)

    # -------------------------------------------------------------------------
    # Status Reading
    # -------------------------------------------------------------------------

    def read_status(self) -> Dict:
        """
        Read complete I/O controller status.

        Returns:
            {
                'status': status_byte,
                'diagnostic': diagnostic_code,
                'power_state': bool,
                'digital_inputs': int (16-bit),
                ...
            }

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        # Configure for full status if not already done
        response = self.send_command(CMD_READ_STATUS, [0xFF, 0xFF])

        if len(response) < 3:
            return {}

        # For I/O controller with full status, diagnostic is at index 1
        status_byte = response[0]
        diagnostic = response[1]

        self.diagnostic_code = diagnostic
        self.status_byte = status_byte
        self.power_state = bool(status_byte & STATUS_POWER_ON)

        result = {
            'status': status_byte,
            'diagnostic': diagnostic,
            'power_state': self.power_state,
        }

        # Parse additional data if available
        # (Implementation depends on actual response format - TBD from hardware)

        return result

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
        return status.get('diagnostic', 0)

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
                'desc': 'Initializing',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,2,3,4],
                'details': 'System initialization in progress'
            },
            0x02: {
                'desc': 'Control voltage shorted',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,2,3,5],
                'details': 'Control voltage short circuit detected'
            },
            0x03: {
                'desc': 'Output shorted',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,2,3],
                'details': 'Output short circuit detected'
            },
            0x04: {
                'desc': 'Control voltage low (<18V)',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,2,4,5],
                'details': 'Control voltage below 18V'
            },
            0x05: {
                'desc': 'Safe / Manual switch malfunction (both contacts ON)',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,2,4],
                'details': 'Both Safe or Manual Mode switch contacts are ON'
            },
            0x06: {
                'desc': 'Power-up Safe error',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,2,5],
                'details': 'Machine Safe error detected at power-up'
            },
            0x07: {
                'desc': 'Power-up Manual Override Mode error',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,2],
                'details': 'Manual Override Mode switch error detected at power-up'
            },
            0x08: {
                'desc': 'System Locked',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,3,4,5],
                'details': 'System locked via software command'
            },
            0x09: {
                'desc': 'Watchdog Timeout',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,3,4],
                'details': 'Watchdog timer expired'
            },
            0x0A: {
                'desc': 'Safety Link Error',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,3,5],
                'details': 'Safety Link daisy-chain broken. Check each device status'
            },
            0x0B: {
                'desc': 'Guard Open Stop (Spindle not stopped)',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,3],
                'details': 'Guard open while spindle was moving'
            },
            0x0C: {
                'desc': 'Guard Open Stop (machine not safe)',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,4,5],
                'details': 'Guard open and machine not safe'
            },
            0x0D: {
                'desc': 'Guard Open Stop (Manual mode without Acknowledge)',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,4],
                'details': 'Guard open in Manual mode without Acknowledge pressed'
            },
            0x0E: {
                'desc': 'Guard contact Fault',
                'power_enable': False,
                'power_ab': False,
                'leds': [1,5],
                'details': 'One or more guard contacts malfunctioning'
            },
            0x0F: {
                'desc': 'Limit Switch Stop',
                'power_enable': False,
                'power_ab': False,
                'leds': [1],
                'details': 'Limit switch activated'
            },
            0x10: {
                'desc': 'Emergency Stop',
                'power_enable': False,
                'power_ab': False,
                'leds': [2,3,4,5],
                'details': 'Emergency stop button pressed'
            },
            0x11: {
                'desc': 'Emergency Stop contact malfunction or Monitor Loop open',
                'power_enable': False,
                'power_ab': False,
                'leds': [2,3,4],
                'details': 'E-stop contact failure or relay monitor loop open'
            },
            0x12: {
                'desc': 'Power ON button busy (>6sec) or Monitor Loop open',
                'power_enable': False,
                'power_ab': False,
                'leds': [2,3,5],
                'details': 'Power button held >6sec or safety relay fault'
            },
            0x13: {
                'desc': 'Motor Power Supply under-voltage',
                'power_enable': True,
                'power_ab': True,
                'leds': [2,3],
                'details': 'Motor power supply (UM) voltage too low'
            },
            0x14: {
                'desc': 'Guards open (ready to power)',
                'power_enable': False,
                'power_ab': False,
                'leds': [2,4,5],
                'details': 'Both guards open, Power LED flashing'
            },
            0x15: {
                'desc': 'Guard 1 closed, Guard 2 open (ready to power)',
                'power_enable': False,
                'power_ab': False,
                'leds': [2,4],
                'details': 'Guard 1 closed, Guard 2 open, Power LED flashing'
            },
            0x16: {
                'desc': 'Guard 1 open, Guard 2 closed (ready to power)',
                'power_enable': False,
                'power_ab': False,
                'leds': [2,5],
                'details': 'Guard 1 open, Guard 2 closed, Power LED flashing'
            },
            0x17: {
                'desc': 'Both Guards closed (ready to power)',
                'power_enable': False,
                'power_ab': False,
                'leds': [2],
                'details': 'Both guards closed, Power LED flashing'
            },
            0x18: {
                'desc': 'Both guards open, Manual mode enabled',
                'power_enable': True,
                'power_ab': True,
                'leds': [3,4,5],
                'details': 'Manual mode active with guards open'
            },
            0x19: {
                'desc': 'Guard 1 closed, Guard 2 open, Manual mode enabled',
                'power_enable': True,
                'power_ab': True,
                'leds': [3,4],
                'details': 'Manual mode: Guard 1 closed, Guard 2 open'
            },
            0x1A: {
                'desc': 'Guard 1 open, Guard 2 closed, Manual mode enabled',
                'power_enable': True,
                'power_ab': True,
                'leds': [3,5],
                'details': 'Manual mode: Guard 1 open, Guard 2 closed'
        },
            0x1B: {
                'desc': 'Both guards closed, Manual Mode',
                'power_enable': True,
                'power_ab': True,
                'leds': [3],
                'details': 'Manual mode active with both guards closed'
            },
            0x1C: {
                'desc': 'Machine safe, spindle stopped, guards open',
                'power_enable': True,
                'power_ab': True,
                'leds': [4,5],
                'details': 'Machine safe, spindle stopped, guards open'
            },
            0x1D: {
                'desc': 'Machine safe, spindle stopped, Guard 1 closed, Guard 2 open',
                'power_enable': True,
                'power_ab': True,
                'leds': [4],
                'details': 'Machine safe with Guard 1 closed, Guard 2 open'
            },
            0x1E: {
                'desc': 'Machine safe, spindle stopped, Guard 1 open, Guard 2 closed',
                'power_enable': True,
                'power_ab': True,
                'leds': [5],
                'details': 'Machine safe with Guard 1 open, Guard 2 closed'
            },
            0x1F: {
                'desc': 'Normal operation - Guards closed',
                'power_enable': True,
                'power_ab': True,
                'leds': [],
                'details': 'All systems ready, guards closed, powered'
            },
            0x00: {
                'desc': 'Power OFF delay in progress',
                'power_enable': False,
                'power_ab': True,
                'leds': [1,2,3,4,5],
                'details': 'Power OFF command delay (per J2 setting)'
            },
        }

        # Lookup diagnostic info
        diag_info = DIAGNOSTIC_TABLE.get(diag_code, {
            'desc': f'Unknown diagnostic code 0x{diag_code:02X}',
            'power_enable': bit7 == 1,
            'power_ab': (bit7 == 1 and bit6 == 0) or diag_code == 0x00,
            'leds': [],
            'details': 'See manual page 20 for this code'
        })

        # Motor power is ON when BOTH Power Enable AND Power A&B relays are ON
        motor_power_on = diag_info['power_enable'] and diag_info['power_ab']

        return {
            'code': diag_code,
            'code_hex': f'0x{diag_code:02X}',
            'code_binary': f'{diag_code:08b}',
            'description': diag_info['desc'],
            'details': diag_info['details'],
            'power_enable': diag_info['power_enable'],
            'power_relays_on': diag_info['power_ab'],
            'motor_power_on': motor_power_on,
            'byte1_bits': {
                'bit7_power_enable': bool(bit7),
                'bit6': bool(bit6),
                'bit5': bool(bit5),
                'bit4': bool(bit4),
                'bit3': bool(bit3),
            },
            'led_states': diag_info['leds'],
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
            diagnostic['motor_power_on'] and
            not io_states['servo_fault'] and
            not spindle['fault']
        )

        return {
            'diagnostic': diagnostic,
            'motor_power_on': diagnostic['motor_power_on'],
            'spindle_status': {
                'off': spindle['off'],
                'fault': spindle['fault'],
                'at_speed': spindle['at_speed'],
                'load_voltage': spindle.get('load_voltage'),
            },
            'safety_status': {
                'at_home': io_states['at_home'],
                'test_mode': io_states['test_mode'],
                'servo_fault': io_states['servo_fault'],
            },
            'io_status': {
                'air_pressure_ok': io_states['air_pressure_ok'],
                'tool_length_switch': io_states['tool_length_switch'],
                'tool_changer_closed': io_states['tool_changer_closed'],
            },
            'system_ready': system_ready,
        }

    # -------------------------------------------------------------------------
    # Bit Manipulation Helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def _get_bit(value: int, bit: int) -> bool:
        """
        Extract a single bit from an integer value.

        Args:
            value: Integer value
            bit: Bit position (0-15)

        Returns:
            True if bit is set, False otherwise
        """
        return bool((value >> bit) & 0x01)

    @staticmethod
    def _set_bit(value: int, bit: int, state: bool) -> int:
        """
        Set a single bit in an integer value.

        Args:
            value: Current integer value
            bit: Bit position (0-15)
            state: True to set bit, False to clear bit

        Returns:
            Modified integer value
        """
        if state:
            return value | (1 << bit)
        else:
            return value & ~(1 << bit)

    # -------------------------------------------------------------------------
    # Digital I/O Control
    # -------------------------------------------------------------------------

    def _validate_spindle_safety_constraint(self, outputs: int) -> None:
        """
        Validate Note 3 safety constraint from page 19 of manual.

        Spindle ON (Bit 2) and Safety Link Bridge (Bit 12) cannot be
        active simultaneously. This is a hardware safety requirement.

        Args:
            outputs: Proposed 16-bit output value

        Raises:
            LDCNError: If both bits are set to 1 simultaneously

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        from pyldcn.network import LDCNError

        spindle_on = self._get_bit(outputs, OUTPUT_SPINDLE_ON)
        safety_bridge = self._get_bit(outputs, OUTPUT_SAFETY_LINK_BRIDGE)

        if spindle_on and safety_bridge:
            raise LDCNError(
                "Safety constraint violated: OUTPUT_SPINDLE_ON (Bit 2) and "
                "OUTPUT_SAFETY_LINK_BRIDGE (Bit 12) cannot be active simultaneously. "
                "See Note 3 on page 19 of SK-2310g2 manual. "
                f"Current outputs: 0x{outputs:04X}"
            )

    def set_outputs(self, outputs: int, validate_safety: bool = True) -> None:
        """
        Set all 16 digital outputs immediately.

        Args:
            outputs: 16-bit output state (bit 0 = Output 0, ..., bit 15 = Output 15)
            validate_safety: If True, validate Note 3 constraint (default: True)

        Output mapping (SK-2310g2) from page 19:
        - Byte 0 (bits 0-7): Application outputs
          - Bit 0: Program running lamp
          - Bit 1: Program stopped lamp
          - Bit 2: Spindle ON (⚠️ Note 3 constraint)
          - Bit 3: Spindle direction (0=CW, 1=CCW)
          - Bit 4: Spindle DC-braking/PWM
          - Bit 5: Tool clamp
          - Bit 6: Solenoid valve 2
          - Bit 7: Solenoid valve 3

        - Byte 1 (bits 8-15): System control outputs
          - Bit 8: Tool changer unlock
          - Bit 9: Guard lock
          - Bit 10: Home enable
          - Bit 11: Test mode inhibit
          - Bit 12: Safety Link Bridge (⚠️ Note 3 constraint)
          - Bit 13: Inverted output 13
          - Bit 14: Reserved / Covers lock/unlock
          - Bit 15: System lock / Power ON/OFF

        Example:
            # Enable spindle forward
            device.set_outputs(0x0004)  # Bit 2 = Spindle ON

            # Enable spindle reverse
            device.set_outputs(0x000C)  # Bit 2 = ON, Bit 3 = reverse

        Raises:
            LDCNError: If safety constraint is violated

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if validate_safety:
            self._validate_spindle_safety_constraint(outputs)

        byte0 = outputs & 0xFF
        byte1 = (outputs >> 8) & 0xFF
        self.send_command(CMD_SET_OUTPUTS, [byte0, byte1])
        self.digital_outputs = outputs

    def set_output_bit(self, bit: int, state: bool) -> None:
        """
        Set a single output bit while preserving other outputs.

        Args:
            bit: Output bit number (0-15)
            state: True to set bit, False to clear bit

        Raises:
            LDCNError: If safety constraint would be violated

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if self.digital_outputs is None:
            # Read current state if not cached
            status = self.read_status()
            self.digital_outputs = status.get('digital_outputs', 0)

        new_outputs = self._set_bit(self.digital_outputs, bit, state)
        self.set_outputs(new_outputs)

    def read_power_state(self) -> bool:
        """
        Read power button state from status bit 3.

        Returns:
            True if power ON, False if power OFF

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        status = self.read_status()
        return status.get('power_state', False)

    # -------------------------------------------------------------------------
    # Power and Safety Monitoring
    # -------------------------------------------------------------------------

    def request_power_on(self, message: str = "Please press the POWER button to continue...") -> None:
        """
        Display operator notification requesting power button press.

        This is a helper method to provide clear feedback to the operator
        before calling wait_for_power_button().

        Args:
            message: Custom message to display (default: standard request)

        Note: With J21 open (default), power can ONLY be enabled via physical button.
              To enable software control, short J21 pins 1-2 on the SK2310g2.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        print(f"\n{'='*60}")
        print(f"POWER REQUIRED")
        print(f"{'='*60}")
        print(f"{message}")
        print(f"Waiting for power state change...")
        print(f"{'='*60}\n")

    def wait_for_power_button(self, timeout: Optional[float] = None, poll_rate: float = 0.1, verbose: bool = False) -> bool:
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
        raise NotImplementedError("E-stop monitoring not yet implemented - I/O mapping TBD")

    def read_guard_state(self) -> bool:
        """
        Read work zone guard state (guarded area contacts).

        Returns:
            True if guards closed (safe), False if any guard open

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD - depends on I/O channel mapping
        """
        # TBD: Read appropriate digital inputs
        raise NotImplementedError("Guard monitoring not yet implemented - I/O mapping TBD")

    def read_safe_zone_state(self) -> bool:
        """
        Read safe zone sensor state (dual sensor interface).

        Returns:
            True if safe zone clear, False if zone occupied

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD - depends on I/O channel mapping
        """
        # TBD: Read appropriate digital inputs
        raise NotImplementedError("Safe zone monitoring not yet implemented - I/O mapping TBD")

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
        return status.get('digital_inputs', 0)

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
            'program_run': self._get_bit(inputs, INPUT_PROGRAM_RUN),
            'program_stop': self._get_bit(inputs, INPUT_PROGRAM_STOP),
            'spindle_off': self._get_bit(inputs, INPUT_SPINDLE_OFF),
            'spindle_fault': self._get_bit(inputs, INPUT_SPINDLE_FAULT),
            'spindle_at_speed': self._get_bit(inputs, INPUT_SPINDLE_AT_SPEED),
            'air_pressure_ok': self._get_bit(inputs, INPUT_AIR_PRESSURE),
            'tool_length_switch': self._get_bit(inputs, INPUT_TOOL_LENGTH_SWITCH),
            'tool_changer_closed': self._get_bit(inputs, INPUT_TOOL_CHANGER_CLOSED),

            # System status (Byte1)
            'at_home': self._get_bit(inputs, INPUT_AT_HOME),
            'test_mode': self._get_bit(inputs, INPUT_TEST_MODE),
            'servo_fault': self._get_bit(inputs, INPUT_SERVO_FAULT),
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
            raise ValueError(f"Invalid analog output channel: {channel}. Only channel 0 (spindle speed) is supported.")

        if not 0.0 <= voltage <= 10.0:
            raise ValueError(f"Voltage {voltage}V out of range. Must be 0.0-10.0V")

        # TBD: Implement actual analog output command
        # This may use CMD_SET_PWM_IO or a different command
        # Need to verify with hardware
        raise NotImplementedError("Analog output command format TBD - needs hardware verification")

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

    def enable_spindle(self, direction: str = 'forward') -> None:
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
        if direction not in ('forward', 'reverse'):
            raise ValueError(f"Invalid direction: {direction}. Must be 'forward' or 'reverse'")

        # Read current outputs
        if self.digital_outputs is None:
            status = self.read_status()
            self.digital_outputs = status.get('digital_outputs', 0)

        # Set spindle enable and direction bits
        new_outputs = self._set_bit(self.digital_outputs, OUTPUT_SPINDLE_ON, True)
        new_outputs = self._set_bit(new_outputs, OUTPUT_SPINDLE_DIRECTION, direction == 'reverse')

        # This will validate Note 3 constraint automatically
        self.set_outputs(new_outputs)

    def disable_spindle(self) -> None:
        """
        Disable spindle by clearing OUTPUT_SPINDLE_ON.

        This clears:
        - OUTPUT_SPINDLE_ON (Bit 2) = 0  (disables LS2315)

        The LS2315 will decelerate according to its internal ramp settings.

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        if self.digital_outputs is None:
            status = self.read_status()
            self.digital_outputs = status.get('digital_outputs', 0)

        new_outputs = self._set_bit(self.digital_outputs, OUTPUT_SPINDLE_ON, False)
        self.set_outputs(new_outputs)

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
            'off': self._get_bit(inputs, INPUT_SPINDLE_OFF),
            'fault': self._get_bit(inputs, INPUT_SPINDLE_FAULT),
            'at_speed': self._get_bit(inputs, INPUT_SPINDLE_AT_SPEED),
        }

        # Add analog load feedback if available
        try:
            analog = self.read_analog_inputs()
            status['load_voltage'] = analog.get(ANALOG_IN_SPINDLE_LOAD, 0.0)
        except Exception:
            status['load_voltage'] = None

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
            'closed': False,  # TBD - need input bit mapping
            'locked': locked,
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


