#!/usr/bin/env python3
"""
Test SK-2310g2 Status Reporting API

Simple script to test status reading from SK-2310g2 at address 6.

This script:
1. Performs soft initialization (preserves device state)
2. Gets the SK-2310g2 device at address 6
3. Configures full status reporting (all 8 status items)
4. Reads and displays formatted status

Usage:
    python3 test_status.py
"""

from pyldcn import LDCNNetwork, InitMode
from pyldcn.devices.sk2310g2 import format_status

# Start network
network = LDCNNetwork("/dev/ttyUSB0")
network.open()

print("Initializing network (soft mode - no reset)...")
num_devices, device_info = network.initialize(mode=InitMode.FULL)
print(f"Found {num_devices} devices\n")

# Get SK-2310g2 by device type
supervisor = network.find_device_by_type("SK-2310g2")
if supervisor is None:
    print("ERROR: SK-2310g2 not found on network")
    network.close()
    exit(1)
print(f"Connected to {supervisor.device_type} at address {supervisor.address}")

# Configure full status reporting (0xFF = all 8 status items)
print("\nConfiguring full status reporting...")
supervisor.configure()

# Read status
print("\nReading status...")
status = supervisor.read_status()

# Display formatted status
print(format_status(status))

network.close()
