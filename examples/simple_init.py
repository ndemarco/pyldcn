#!/usr/bin/env python3
"""
Simple LDCN device initialization example.

Initializes all LDCN devices on /dev/ttyUSB0 using auto init mode
and prints which devices were found.

Usage:
    python3 simple_init.py
"""

from pyldcn import LDCNNetwork, InitMode, LS231SE
from pyldcn.devices import servo_diagnostics as diag

# Create network and initialize with auto mode
network = LDCNNetwork("/dev/ttyUSB0")
network.open()

# Initialize network (auto mode detects and recovers from any state)
num_devices, device_info = network.initialize(mode=InitMode.AUTO)

# Print results
print(f"Found {num_devices} devices:")
for dev in device_info:
    if dev["responding"]:
        print(
            f"  Address {dev['address']}: ID=0x{dev['device_id']:02X}, Version=0x{dev['version']:02X}"
        )

# Print device objects
print("\nDevice objects:")
for device in network.devices:
    print(f"  {device}")

# If an LS-231SE is present, read status and decode primary flags for reference
for device in network.devices:
    if isinstance(device, LS231SE):
        status = device.read_status()
        status_byte = status.get("status", 0)
        aux_byte = status.get("aux_status", device.state.aux_status or 0)
        state = diag.get_servo_state(status_byte, aux_byte)

        print(
            "\nLS-231SE status decode:"
            f"\n  Status byte 0x{status_byte:02X} ->"
            f" move_done={state['flags']['move_done']},"
            f" power={state['flags']['power']},"
            f" pos_error={state['flags']['pos_error']},"
            f" home_source={state['flags']['home_source']},"
            f" limit2={state['flags']['limit2']}"
            f"\n  Aux byte 0x{aux_byte:02X} ->"
            f" servo_on={state['flags']['servo_on']},"
            f" accel_done={state['flags']['accel_done']},"
            f" slew_done={state['flags']['slew_done']}"
        )
        break

network.close()
