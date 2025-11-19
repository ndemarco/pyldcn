#!/usr/bin/env python3
"""
Quick LS-231SE initialization helper.

Edit the constants below to experiment with different defaults without
touching the rest of the script. All device interaction uses the LS231SE
API—never call send_command directly here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

from pyldcn import InitMode
from pyldcn.devices import LS231SE
from pyldcn.network import LDCNNetwork

# -----------------------------------------------------------------------------
# User-tunable defaults
# -----------------------------------------------------------------------------

PORT = "/dev/ttyUSB0"
INIT_MODE = InitMode.AUTO

# Configure which status items every NOP/command should return.
STATUS_MASK = 0x0001 | 0x0004 | 0x0008 | 0x0040  # position, velocity, aux, pos_error


@dataclass
class GainSettings:
    kp: int = 0
    kd: int = 0
    ki: int = 0
    il: int = 0
    ol: int = 0
    cl: int = 0
    el: int = 0
    sr: int = 0
    db: int = 0


GAINS = GainSettings()

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------


def find_servo(devices: Iterable[object]) -> Optional[LS231SE]:
    for device in devices:
        if isinstance(device, LS231SE):
            return device
    return None


def apply_initialization(servo: LS231SE) -> None:
    """Perform repeatable LS-231SE setup steps using helper APIs only."""
    print(f"Configuring status mask: 0x{STATUS_MASK:04X}")
    servo.configure_status(STATUS_MASK)

    print("Loading PID gains:", GAINS)
    servo.set_gains(
        kp=GAINS.kp,
        kd=GAINS.kd,
        ki=GAINS.ki,
        il=GAINS.il,
        ol=GAINS.ol,
        cl=GAINS.cl,
        el=GAINS.el,
        sr=GAINS.sr,
        db=GAINS.db,
    )

    print("Resetting position counter to zero.")
    servo.motion.reset_position()

    print("Clearing sticky bits (CMD_CLEAR_BITS).")
    servo.clear_faults()

    print("Stage 2: Loading trapezoidal trajectory (pos=0, accel=1000).")
    servo.motion.move_to_counts(position=0, velocity=0, accel=1000)

    confirm = (
        input(
            "Stage 3: Close servo loop via STOP_MOTOR (AMP_ENABLE|STOP_ABRUPT)? [y/N]: "
        )
        .strip()
        .lower()
    )
    if confirm == "y":
        print(
            "Enabling amplifier using STOP_MOTOR bits "
            "AMP_ENABLE=0x01 and STOP_ABRUPT=0x04 "
            "(see pyldcn/devices/servo_motion.pyyy"
            "y)."
        )
        servo.motion.enable()
    else:
        print("Skipping servo enable stage (operator declined).")

    servo.clear_faults()

    status = servo.read_status()
    print(
        "Status byte: 0x{0:02X}, aux: 0x{1:02X}".format(
            status.get("status", 0),
            status.get("aux_status", 0),
        )
    )


def main() -> None:
    with LDCNNetwork(PORT) as network:
        print(f"Opening {PORT} and initializing in {INIT_MODE.name} mode...")
        num_devices, _ = network.initialize(mode=INIT_MODE)
        print(f"Found {num_devices} device(s).")

        servo = find_servo(network.devices)
        if not servo:
            raise RuntimeError("No LS-231SE detected—connect one and rerun.")

        print(f"Applying initialization to LS-231SE at address {servo.address}")
        apply_initialization(servo)
        print("Initialization complete.")


if __name__ == "__main__":
    main()
