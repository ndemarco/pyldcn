"""
SK-2310g2 I/O Controller Device

Generic LDCN I/O controller used for supervisory control with safety
and spindle control functions.

Author: NickyDoes
License: GPL v2 or later
"""

import time
from typing import Optional, Dict

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


class SK2310g2(LDCNDevice):
    """
    SK-2310g2 I/O Controller

    Generic LDCN I/O controller device used in this application as a
    supervisory controller with safety and spindle control functions.

    Hardware Capabilities:
    - Dual mechanical relay power control
    - Spindle control with spindle enable mechanical relay
    - Dual line emergency stop monitoring
    - Dual work zone "covers" contacts (guarded area monitoring)
    - Dual safe zone sensor interface
    - 3 analog inputs
    - 1 analog output (CN6.11 is 0-10V spindle speed control)
    - Digital I/O (16 inputs, 16 outputs)

    Additional Attributes:
        diagnostic_code: Last diagnostic code (LED display)
        power_state: Power button state
        estop_state: Emergency stop state
        digital_inputs: 16-bit digital input state
        digital_outputs: 16-bit digital output state
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

        # For I/O controller with full status, status byte is at index 1
        diagnostic = response[0]
        status_byte = response[1]

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

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        status = self.read_status()
        return status.get('diagnostic', 0)

    # -------------------------------------------------------------------------
    # Digital I/O Control
    # -------------------------------------------------------------------------

    def set_outputs(self, outputs: int) -> None:
        """
        Set all 16 digital outputs immediately.

        Args:
            outputs: 16-bit output state (bit 0 = Output 0, ..., bit 15 = Output 15)

        Output mapping (SK-2310g2):
        - Byte 0 (bits 0-7): Outputs 0-7
        - Byte 1 (bits 8-15): Outputs 8-15
          - Bit 15 (0x8000): System Lock / Power ON/OFF

        Example:
            # Enable power relay (Output 15)
            device.set_outputs(0x8000)

            # Turn on outputs 0, 2, and 15
            device.set_outputs(0x8005)  # 0x8000 | 0x0004 | 0x0001

        🔴 UNVERIFIED - Not yet tested on hardware
        """
        byte0 = outputs & 0xFF
        byte1 = (outputs >> 8) & 0xFF
        self.send_command(CMD_SET_OUTPUTS, [byte0, byte1])
        self.digital_outputs = outputs

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

    def read_cover_state(self) -> bool:
        """
        Read work zone cover state (guarded area contacts).

        Returns:
            True if covers closed (safe), False if any cover open

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD - depends on I/O channel mapping
        """
        # TBD: Read appropriate digital inputs
        raise NotImplementedError("Cover monitoring not yet implemented - I/O mapping TBD")

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
    # Digital I/O (Stubs - Implementation TBD)
    # -------------------------------------------------------------------------

    def read_digital_inputs(self) -> int:
        """
        Read all digital input states.

        Returns:
            16-bit digital input value

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Digital I/O not yet implemented")

    def set_digital_outputs(self, outputs: int) -> None:
        """
        Set all digital output states.

        Args:
            outputs: 16-bit digital output value

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Digital I/O not yet implemented")

    # -------------------------------------------------------------------------
    # Analog I/O (Stubs - Implementation TBD)
    # -------------------------------------------------------------------------

    def read_analog_inputs(self) -> Dict[int, int]:
        """
        Read all analog input values (3 channels).

        Returns:
            Dictionary of {channel: value} pairs

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Analog I/O not yet implemented")

    def set_analog_output(self, voltage: float) -> None:
        """
        Set analog output voltage (CN6.11 spindle speed control).

        Args:
            voltage: Output voltage (0.0 - 10.0V)

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Analog output not yet implemented")

    # -------------------------------------------------------------------------
    # Spindle Control (Stubs - Implementation TBD)
    # -------------------------------------------------------------------------

    def set_spindle_speed(self, speed_percent: float) -> None:
        """
        Set spindle speed via analog output.

        Args:
            speed_percent: Speed as percentage (0.0 - 100.0)

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        voltage = (speed_percent / 100.0) * 10.0
        self.set_analog_output(voltage)

    def enable_spindle(self) -> None:
        """
        Enable spindle via mechanical relay.

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Spindle control not yet implemented")

    def disable_spindle(self) -> None:
        """
        Disable spindle via mechanical relay.

        🔴 UNVERIFIED - Not yet tested on hardware
        ⚠️  Implementation TBD
        """
        raise NotImplementedError("Spindle control not yet implemented")


