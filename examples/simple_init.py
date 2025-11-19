#!/usr/bin/env python3
"""
Simple LDCN device initialization example.

Initializes all LDCN devices on /dev/ttyUSB0 using auto init mode
and prints which devices were found.

Usage:
    python3 simple_init.py
"""

from pyldcn import LDCNNetwork, InitMode

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

network.close()
