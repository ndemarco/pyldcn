#!/usr/bin/env python3
"""
Monitor LS-231SE servo status bytes and I/O diagnostics in real-time.

Status byte decoding is based on docs/servo_status_reporting.md and the
I/O/diagnostic pin meaning table from docs/servo_diagnostics.md.
"""

from __future__ import annotations

from typing import Optional, Dict

from pyldcn import InitMode
from pyldcn.network import LDCNNetwork
from pyldcn.devices import LS231SE
from pyldcn.devices import servo_diagnostics as diag


# Configuration constants
PORT = "/dev/ttyUSB0"
INIT_MODE = InitMode.AUTO
STATUS_MASK = 0x0001 | 0x0004 | 0x0008 | 0x0040  # pos, vel, aux, pos_err


def format_inputs(flags: Dict[str, bool], servo: LS231SE) -> str:
    """Format a comprehensive summary of digital inputs with physical signal names."""
    # Get mapped inputs with physical names and label-style states
    inputs = servo.io.read_mapped_inputs()

    parts = []

    # Show mapped physical signals with their label-style states
    parts.append(f"{inputs['bit5']['label']}={inputs['bit5']['state']}")
    parts.append(f"{inputs['bit6']['label']}={inputs['bit6']['state']}")
    parts.append(f"{inputs['index']['label']}={inputs['index']['state']}")

    # Add key status flags
    if flags.get('power'):
        parts.append("Power=ON")
    if flags.get('servo_on'):
        parts.append("ServoOn=YES")

    return " | ".join(parts)


def format_outputs(
    condition: Optional[diag.DiagnosticCondition], servo: LS231SE
) -> str:
    """Format a comprehensive summary of digital outputs."""
    parts = []

    # Brake and fault relay from diagnostic condition
    if condition:
        brake_short = condition.brake_state.replace("Released", "RLS").replace("Engaged", "ENG")
        parts.append(f"Brake={brake_short}")

        relay_short = "CLS" if condition.fault_relay == "Closed" else "OPN"
        parts.append(f"FaultRelay={relay_short}")

    # Operating mode
    mode = servo.state.current_mode
    parts.append(f"Mode={mode}")

    return " | ".join(parts)


def main() -> None:
    try:
        with LDCNNetwork(PORT) as network:
            print(f"Opening {PORT}...")
            print(f"Initializing network (mode={INIT_MODE.name})...")
            num_devices, _ = network.initialize(mode=INIT_MODE)
            print(f"Found {num_devices} device(s).")

            device = network.find_device_by_type("LS-231SE")
            if not device or not isinstance(device, LS231SE):
                print("No LS-231SE device found on the network.")
                return

            servo: LS231SE = device

            # Configure status reporting
            print("Configuring status reporting...")
            servo.configure_status(STATUS_MASK)

            print("Resetting position counter...")
            servo.motion.reset_position()

            print(f"Monitoring LS-231SE at address {servo.address} (Ctrl+C to stop)\n")

            while True:
                # Get comprehensive state with all context applied
                state = servo.get_state()

                flags = state["flags"]
                condition = state["condition"]
                status_byte = state["status_byte"]
                aux_byte = state["aux_byte"]

                position = state.get("position")
                velocity = state.get("velocity")
                pos_error = state.get("pos_error")

                motion = (
                    f"Pos={position if position is not None else 'N/A':>8}"
                    f"  Vel={velocity if velocity is not None else 'N/A':>6}"
                    f"  Err={pos_error if pos_error is not None else 'N/A':>6}"
                )

                # Build status summary with I/O prominence
                summary_lines = [
                    "=" * 80,
                    f"Status: 0x{status_byte:02X}  Aux: 0x{aux_byte:02X}  {motion}",
                    "",
                    f"INPUTS:  {format_inputs(flags, servo)}",
                    f"OUTPUTS: {format_outputs(condition, servo)}",
                    "",
                ]

                # Add any active alerts
                alerts = []
                if flags.get("move_done"):
                    alerts.append("MoveDone")
                if flags.get("home_in_progress"):
                    alerts.append("Homing")
                if flags.get("pos_error"):
                    alerts.append("PosError")
                if flags.get("current_limit"):
                    alerts.append("CurrentLimit")
                if flags.get("cksum_error"):
                    alerts.append("ChecksumErr")
                if flags.get("servo_overrun"):
                    alerts.append("ServoOverrun")
                if flags.get("pos_wrap"):
                    alerts.append("PosWrap")

                if alerts:
                    summary_lines.append(f"ALERTS:  {', '.join(alerts)}")
                else:
                    summary_lines.append("ALERTS:  None")

                # Add LED status if available
                if condition:
                    leds = f"LEDs: O={condition.orange_led} G={condition.green_led} R={condition.red_led}"
                    summary_lines.append(leds)

                summary_lines.append("=" * 80)

                print("\n".join(summary_lines))
                _ = input("\nPress Enter to refresh, Ctrl+C to stop...")

    except KeyboardInterrupt:
        print("\nMonitoring stopped.")


if __name__ == "__main__":
    main()
