#!/usr/bin/env python3
"""
util.py - LDCN Utility Functions

Utility functions for device list management and configuration.

Author: LinuxCNC Community
License: GPL v2 or later
Date: 2025-10-29
"""

import json
from typing import List, Dict
from datetime import datetime


def save_device_list(devices: List, port: str, baud_rate: int, filename: str) -> None:
    """
    Save discovered device list to JSON file.

    This saves hardware facts from network discovery only.
    Does not include axis configuration (tuning, homing, limits).

    File format:
    {
        "file_version": "1.0",
        "discovered_at": "2025-10-29T12:34:56",
        "port": "/dev/ttyUSB0",
        "baud_rate": 125000,
        "num_devices": 6,
        "devices": [
            {
                "address": 1,
                "device_id": 0,
                "device_type": "LS-231SE",
                "version": 21
            },
            ...
        ]
    }

    Args:
        devices: List of LDCNDevice objects
        port: Serial port path
        baud_rate: Current baud rate
        filename: Path to save file (e.g., 'device_list.json')

    Example:
        from pyldcn.util import save_device_list
        save_device_list(network.devices, network.port, network.baud_rate, 'device_list.json')

    🔴 UNVERIFIED - Not yet tested on hardware
    """
    from pyldcn.network import LDCNError  # Import here to avoid circular dependency

    if not devices:
        raise LDCNError("No devices to save. Run initialize() first.")

    device_list_data = []
    for device in devices:
        device_data = {
            "address": device.address,
            "device_id": device.model_id if device.model_id is not None else 0,
            "device_type": device.device_type,
            "version": device.version if device.version is not None else 0
        }
        device_list_data.append(device_data)

    config = {
        "file_version": "1.0",
        "discovered_at": datetime.now().isoformat(),
        "port": port,
        "baud_rate": baud_rate,
        "num_devices": len(devices),
        "devices": device_list_data
    }

    with open(filename, 'w') as f:
        json.dump(config, f, indent=2)


def load_device_list(filename: str) -> List[Dict]:
    """
    Load device list from JSON file.

    Validates file format and returns device information.
    Does not create device objects or open serial port.

    Args:
        filename: Path to device list file

    Returns:
        List of device info dictionaries

    Raises:
        LDCNError: If file format invalid or unsupported version

    Example:
        from pyldcn.util import load_device_list
        device_list = load_device_list('device_list.json')
        print(f"Loaded {len(device_list)} devices")

    🔴 UNVERIFIED - Not yet tested on hardware
    """
    from pyldcn.network import LDCNError  # Import here to avoid circular dependency

    try:
        with open(filename, 'r') as f:
            config = json.load(f)
    except FileNotFoundError:
        raise LDCNError(f"Device list file not found: {filename}")
    except json.JSONDecodeError as e:
        raise LDCNError(f"Invalid JSON in device list file: {e}")

    # Validate file version
    file_version = config.get('file_version')
    if file_version != "1.0":
        raise LDCNError(f"Unsupported device list file version: {file_version}")

    # Validate required fields
    required_fields = ['file_version', 'discovered_at', 'port', 'baud_rate', 'num_devices', 'devices']
    for field in required_fields:
        if field not in config:
            raise LDCNError(f"Missing required field in device list: {field}")

    devices = config['devices']
    if len(devices) != config['num_devices']:
        raise LDCNError(f"Device count mismatch: expected {config['num_devices']}, got {len(devices)}")

    # Validate device entries
    for i, device in enumerate(devices):
        required_device_fields = ['address', 'device_id', 'device_type', 'version']
        for field in required_device_fields:
            if field not in device:
                raise LDCNError(f"Device {i} missing required field: {field}")

    return devices
