#!/usr/bin/env python3
"""
Monitor LS-231SE servo status bytes and I/O diagnostics in real-time.

Status byte decoding is based on docs/servo_status_reporting.md and the
I/O/diagnostic pin meaning table from docs/servo_diagnostics.md.
"""

from __future__ import annotations

import argparse
import time
from itertools import cycle
from typing import Iterable, Optional, Dict

from pyldcn import InitMode
from pyldcn.network import LDCNNetwork
from pyldcn.devices import LS231SE
from pyldcn.devices import servo_diagnostics as diag


INIT_MODE_MAP = {
    "auto": InitMode.AUTO,
    "soft": InitMode.SOFT,
    "full": InitMode.FULL,
    "validate": InitMode.VALIDATE,
    "readdress": InitMode.READDRESS,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor LS-231SE status bytes, auxiliary status, and key I/O states."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port connected to the LDCN network (default: %(default)s)",
    )
    parser.add_argument(
        "--mode",
        choices=INIT_MODE_MAP.keys(),
        default="auto",
        help="Initialization mode to use before monitoring (default: %(default)s)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.25,
        help="Polling interval in seconds (default: %(default)s)",
    )
    return parser.parse_args()


def find_servo(devices: Iterable[object]) -> Optional[LS231SE]:
    for device in devices:
        if isinstance(device, LS231SE):
            return device
    return None


def format_inputs(flags: Dict[str, bool], servo: LS231SE) -> str:
    """Format a comprehensive summary of digital inputs."""
    selection = servo.state.home_selection
    homesel_str = f"{selection[0]}{selection[1]}"

    # Get actual input states from IO subsystem
    inputs = servo.io.read_inputs()

    parts = [
        f"HomeSEL={homesel_str}",
        f"Home={'HIGH' if inputs['home_source'] else 'LOW'}",
        f"Limit2={'ACT' if inputs['limit2'] else 'OK'}",
        f"Index={'HIGH' if inputs['index'] else 'LOW'}",
    ]

    # Add flags from status decoding
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
    args = parse_args()
    init_mode = INIT_MODE_MAP[args.mode]

    try:
        with LDCNNetwork(args.port) as network:
            print(f"Opening {args.port}...")
            print(f"Initializing network (mode={args.mode})...")
            num_devices, _ = network.initialize(mode=init_mode)
            print(f"Found {num_devices} device(s).")

            servo = find_servo(network.devices)
            if not servo:
                print("No LS-231SE device found on the network.")
                return

            print("Resetting position counter (CMD_RESET_POS)...")
            servo.motion.reset_position()

            print(f"Monitoring LS-231SE at address {servo.address} (Ctrl+C to stop)\n")

            while True:
                status = servo.read_status()
                status_byte = status.get("status", 0)
                aux_byte = status.get("aux_status", servo.state.aux_status or 0)
                state = diag.get_servo_state(
                    status_byte,
                    aux_byte,
                    stop_cmd=servo.state.stop_cmd,
                    pic_ae=servo.state.pic_ae,
                )

                flags = state["flags"]
                condition = state["condition"]
                position = status.get("position")
                velocity = status.get("velocity")
                pos_error = status.get("pos_error")
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
