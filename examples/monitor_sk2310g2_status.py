#!/usr/bin/env python3
"""
Monitor SK-2310g2 diagnostic status in real-time.

Continuously polls the SK-2310g2 and displays diagnostic code, LED pattern,
and condition description in one line.
"""

import time
from itertools import cycle
from pyldcn.network import LDCNNetwork
from pyldcn.devices.sk2310g2 import SK2310g2, format_led_pattern, DIAGNOSTIC_CODES


def main():
    # Initialize network
    network = LDCNNetwork("/dev/ttyUSB0")

    # Auto-initialize devices
    print("Initializing devices...")
    num_devices, _ = network.initialize()
    print(f"Found {num_devices} device(s)")

    devices = network.devices

    # Find SK-2310g2
    sk2310 = None
    for device in devices:
        if isinstance(device, SK2310g2):
            sk2310 = device
            break

    if not sk2310:
        print("No SK-2310g2 found on network")
        return

    print(f"Found SK-2310g2 at address {sk2310.address}")
    print("Monitoring status (Ctrl+C to stop)...")
    print()

    spinner = cycle("⣾⣽⣻⢿⡿⣟⣯⣷")
    try:
        while True:
            # Read status
            status = sk2310.read_status()
            diagnostic = status.get("diagnostic", 0)

            # Format output
            led_pattern = format_led_pattern(diagnostic)
            condition = DIAGNOSTIC_CODES.get(diagnostic, "Unknown condition")

            # Display in one line (overwrite previous)
            print(
                f"\r{next(spinner)} Code: 0x{diagnostic:02X}  LED: {led_pattern}  {condition}                    ",
                end="",
                flush=True,
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped")


if __name__ == "__main__":
    main()
