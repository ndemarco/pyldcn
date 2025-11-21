#!/usr/bin/env python3
"""
Monitor LS-231SE servo status bytes and I/O diagnostics in real-time.

Status byte decoding is based on docs/servo_status_reporting.md and the
I/O/diagnostic pin meaning table from docs/servo_diagnostics.md.
"""

from __future__ import annotations

import struct
from typing import Optional, Dict, Any

from pyldcn import InitMode
from pyldcn.network import LDCNNetwork
from pyldcn.devices import LS231SE
from pyldcn.devices import servo_diagnostics as diag
from pyldcn.protocol import CMD_NOP


# Configuration constants
PORT = "/dev/ttyUSB0"
INIT_MODE = InitMode.AUTO
STATUS_MASK = 0x0001 | 0x0004 | 0x0008 | 0x0040  # pos, vel, aux, pos_err


def read_status_raw(servo: LS231SE) -> Dict[str, Any]:
    """
    Read status directly via READ_STATUS command with all bits (0xFF).

    Returns raw bytes and parsed motion data for comparison with class methods.
    """
    from pyldcn.protocol import CMD_READ_STATUS

    # Read status with all available items (bits 0-7)
    response = servo.send_command(CMD_READ_STATUS, [0xFF, 0x00])

    # Parse response - status byte is always first
    status_byte = response[0]
    idx = 1

    # Parse based on what we requested (0xFF = bits 0-7)
    # Bit 0: Position (4 bytes)
    position = struct.unpack('<i', bytes(response[idx:idx+4]))[0] if len(response) > idx else None
    idx += 4

    # Bit 1: A/D Value (1 byte)
    ad_value = response[idx] if len(response) > idx else None
    idx += 1

    # Bit 2: Velocity (2 bytes)
    velocity = struct.unpack('<h', bytes(response[idx:idx+2]))[0] if len(response) > idx+1 else None
    idx += 2

    # Bit 3: Aux status (1 byte)
    aux_byte = response[idx] if len(response) > idx else 0
    idx += 1

    # Bit 4: Home position (4 bytes)
    home_pos = struct.unpack('<i', bytes(response[idx:idx+4]))[0] if len(response) > idx+3 else None
    idx += 4

    # Bit 5: Device ID/Version (2 bytes)
    dev_id = response[idx] if len(response) > idx else None
    version = response[idx+1] if len(response) > idx+1 else None
    idx += 2

    # Bit 6: Position error (2 bytes)
    pos_error = struct.unpack('<h', bytes(response[idx:idx+2]))[0] if len(response) > idx+1 else None
    idx += 2

    # Bit 7: Path count (1 byte)
    path_count = response[idx] if len(response) > idx else None

    return {
        'status_byte': status_byte,
        'aux_byte': aux_byte,
        'position': position,
        'velocity': velocity,
        'pos_error': pos_error,
        'ad_value': ad_value,
        'home_pos': home_pos,
        'dev_id': dev_id,
        'version': version,
        'path_count': path_count,
    }


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

                # Also read status directly for comparison
                raw = read_status_raw(servo)

                motion = (
                    f"Pos={position if position is not None else 'N/A':>8}"
                    f"  Vel={velocity if velocity is not None else 'N/A':>6}"
                    f"  Err={pos_error if pos_error is not None else 'N/A':>6}"
                )

                # Build status summary with I/O prominence
                summary_lines = [
                    "=" * 80,
                    f"Status: 0x{status_byte:02X} (Raw: 0x{raw['status_byte']:02X})  Aux: 0x{aux_byte:02X} (Raw: 0x{raw['aux_byte']:02X})  {motion}",
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

                # Show LED states from diagnostic condition
                if condition:
                    summary_lines.append(f"LEDs: Orange={condition.orange_led} Green={condition.green_led} Red={condition.red_led}")

                # Show bits in both LSB format and Logosol diagnostic order
                # Convert bytes to binary strings (LSB format: bit7..bit0)
                status_lsb = f"{status_byte:08b}"
                aux_lsb = f"{aux_byte:08b}"
                raw_status_lsb = f"{raw['status_byte']:08b}"
                raw_aux_lsb = f"{raw['aux_byte']:08b}"

                summary_lines.append("")
                summary_lines.append(f"Status byte:  {status_lsb}  (Raw: {raw_status_lsb})")
                summary_lines.append(f"Aux byte:     {aux_lsb}  (Raw: {raw_aux_lsb})")
                summary_lines.append("")

                # Show bits in Logosol manual order: s6 s5 s4 s3 s0 a2 a0 sc0
                # s = status byte, a = aux byte, sc = stop_cmd
                s6 = '●' if status_byte & 0x40 else '○'   # bit 6 - limit2
                s5 = '●' if status_byte & 0x20 else '○'   # bit 5 - home_source
                s4 = '●' if status_byte & 0x10 else '○'   # bit 4 - pos_error
                s3 = '●' if status_byte & 0x08 else '○'   # bit 3 - power
                s0 = '●' if status_byte & 0x01 else '○'   # bit 0 - move_done
                a2 = '●' if aux_byte & 0x04 else '○'      # bit 2 - servo_on
                a0 = '●' if aux_byte & 0x01 else '○'      # bit 0 - index
                sc0 = '●' if servo.state.stop_cmd else '○'  # stop_cmd

                # Raw bits in same order
                rs6 = '●' if raw['status_byte'] & 0x40 else '○'
                rs5 = '●' if raw['status_byte'] & 0x20 else '○'
                rs4 = '●' if raw['status_byte'] & 0x10 else '○'
                rs3 = '●' if raw['status_byte'] & 0x08 else '○'
                rs0 = '●' if raw['status_byte'] & 0x01 else '○'
                ra2 = '●' if raw['aux_byte'] & 0x04 else '○'
                ra0 = '●' if raw['aux_byte'] & 0x01 else '○'

                summary_lines.append(f"Diag order: {s6} {s5} {s4} {s3} {s0} {a2} {a0} {sc0}  [s6 s5 s4 s3 s0 a2 a0 sc0]")
                summary_lines.append(f"   (Raw):   {rs6} {rs5} {rs4} {rs3} {rs0} {ra2} {ra0} ─")

                summary_lines.append("=" * 80)

                print("\n".join(summary_lines))
                _ = input("\nPress Enter to refresh, Ctrl+C to stop...")

    except KeyboardInterrupt:
        print("\nMonitoring stopped.")


if __name__ == "__main__":
    main()
