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
from typing import Iterable, Optional

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


def format_condition(condition: Optional[diag.DiagnosticCondition]) -> str:
    if not condition:
        return "Condition: UNKNOWN"
    prefix = "FAULT" if condition.is_faulted else "OK"
    return f"{condition.condition} ({prefix})"


def format_io(flags: dict, condition: Optional[diag.DiagnosticCondition]) -> str:
    """Only show I/O states when something notable is happening."""
    parts = []

    if flags["home_source"]:
        parts.append("Home=HIGH")
    if flags["limit2"]:
        parts.append("Limit2=ACTIVE")
    if not flags["index"]:
        parts.append("Index=LOW")

    brake = condition.brake_state if condition else None
    if brake and brake not in {"Released", "CN8pin9 Low", "Unknown"}:
        parts.append(f"Brake={brake}")

    if not parts:
        return "IO nominal"
    return "IO: " + ", ".join(parts)


def main() -> None:
    args = parse_args()
    spinner = cycle("⣾⣽⣻⢿⡿⣟⣯⣷")
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

                alerts = []
                if flags["power"]:
                    alerts.append("Power")
                if flags["servo_on"]:
                    alerts.append("Servo")
                if flags["limit2"]:
                    alerts.append("Limit2")
                if flags["home_in_progress"]:
                    alerts.append("Homing")
                if flags["pos_error"]:
                    alerts.append("PosErr")
                if flags["current_limit"]:
                    alerts.append("CurrentLimit")
                if flags["cksum_error"]:
                    alerts.append("Checksum")
                if flags["servo_overrun"]:
                    alerts.append("ServoOverrun")
                if flags["pos_wrap"]:
                    alerts.append("PosWrap")
                if condition and condition.fault_relay == "Closed":
                    alerts.append("FaultRelay")

                alert_text = ", ".join(alerts) if alerts else "None"
                summary = (
                    # f"\r{next(spinner)} "
                    f"Stat=0x{status_byte:02X} Aux=0x{aux_byte:02X}  "
                    # f"{format_condition(condition)}  "
                    f"{motion}  "
                    f"Active[{alert_text}]  "
                    f"{format_io(flags, condition)}      "
                )
                print(summary)
                # print(summary, end="", flush=True)
                time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nMonitoring stopped.")


if __name__ == "__main__":
    main()
