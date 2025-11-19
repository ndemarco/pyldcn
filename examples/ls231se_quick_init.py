#!/usr/bin/env python3
"""
LS-231SE quick initialization and monitoring launcher.

This script relies solely on pyldcn abstractions: it initializes the network,
locates the first LS-231SE via find_device_by_type(), applies configuration,
executes the requested motion/STOP_MOTOR sequence, then launches the
monitor_ls231se_status.py helper for live diagnostics.
"""

from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass

from pyldcn import InitMode
from pyldcn.network import LDCNNetwork

# -----------------------------------------------------------------------------
# Tunables
# -----------------------------------------------------------------------------

PORT = "/dev/ttyUSB0"
INIT_MODE = InitMode.AUTO
STATUS_MASK = 0x0001 | 0x0004 | 0x0008 | 0x0040  # pos, vel, aux, pos_err


@dataclass
class GainSettings:
    kp: int = 10
    kd: int = 1000
    ki: int = 20
    il: int = 40
    ol: int = 255
    cl: int = 129
    el: int = 2000
    sr: int = 1
    db: int = 0


GAINS = GainSettings()


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------


def apply_initialization(network: LDCNNetwork) -> None:
    """Configure the first LS-231SE found on the network."""
    servo = network.find_device_by_type("LS-231SE")
    if servo is None:
        raise RuntimeError("No LS-231SE detected on the network.")

    servo.configure_status(STATUS_MASK)

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

    servo.motion.reset_position()
    servo.clear_faults()

    # Load trajectory: target=0, velocity=0, accel=1. move_to_counts sets the
    # trapezoidal, start-now bits for us.
    servo.motion.move_to_counts(position=0, velocity=0, accel=1)

    # Close the servo loop via STOP_MOTOR (AMP_ENABLE | STOP_ABRUPT).
    servo.motion.enable()

    # Give hardware a moment to settle before we switch to the monitor script.
    time.sleep(0.1)


def launch_monitor() -> None:
    """Spawn the LS-231SE monitor script on the configured port."""
    subprocess.run(
        ["python3", "examples/monitor_ls231se_status.py", "--port", PORT],
        check=False,
    )


def main() -> None:
    with LDCNNetwork(PORT) as network:
        print(f"Opening {PORT} and initializing in {INIT_MODE.name} mode...")
        num_devices, _ = network.initialize(mode=INIT_MODE)
        print(f"Found {num_devices} device(s).")
        apply_initialization(network)
        print("Initialization sequence complete.")

    print("\nLaunching monitor_ls231se_status.py (Ctrl+C to stop)...\n")
    launch_monitor()


if __name__ == "__main__":
    main()
