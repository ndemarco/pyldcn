#!/usr/bin/env python3
"""
Unified power control test for SK-2310g2

Tests power on/off using diagnostic code API.
"""

import time
from pyldcn import LDCNNetwork, InitMode
from pyldcn.devices.sk2310g2 import SK2310g2, DIAGNOSTIC_CODES


def get_power_state_description(diagnostic: int) -> str:
    """Get human-readable power state from diagnostic code.

    Power states based on diagnostic codes:
    - POWER ON: 0x13, 0x18-0x1F
    - READY TO POWER: 0x14-0x17
    - POWER OFF: All other states
    """
    # Power ON states
    if diagnostic == 0x13 or (0x18 <= diagnostic <= 0x1F):
        return "POWER ON"

    # Ready to power states
    if 0x14 <= diagnostic <= 0x17:
        return "READY TO POWER (OFF)"

    # All other states are power OFF
    return "POWER OFF"


def format_diagnostic_details(diagnostic: int) -> str:
    """Format detailed diagnostic information."""
    condition = DIAGNOSTIC_CODES.get(diagnostic, "Unknown diagnostic code")

    lines = []
    lines.append(f"Condition: {condition}")

    # Determine power state details
    power_state = get_power_state_description(diagnostic)
    lines.append(f"Power State: {power_state}")

    # Add specific details based on diagnostic code ranges
    if diagnostic == 0x00:
        lines.append("Status: Power OFF delay in progress")
    elif diagnostic == 0x01:
        lines.append("Status: System initializing")
    elif 0x02 <= diagnostic <= 0x0F:
        lines.append("Status: FAULT - System error detected")
    elif diagnostic == 0x10:
        lines.append("Status: STOP - Emergency stop active")
    elif 0x11 <= diagnostic <= 0x13:
        lines.append("Status: FAULT - Power system fault")
    elif 0x14 <= diagnostic <= 0x17:
        lines.append("Status: Ready to accept power command")
    elif 0x18 <= diagnostic <= 0x1B:
        lines.append("Status: Manual override mode active")
    elif 0x1C <= diagnostic <= 0x1E:
        lines.append("Status: Safe zone, guards open")
    elif diagnostic == 0x1F:
        lines.append("Status: Normal operation")

    return "\n  ".join(lines)


def monitor_power_state(
    device: SK2310g2, duration: float = 10.0, verbose: bool = False
):
    """Monitor and display power state changes.

    Args:
        device: SK2310g2 device to monitor
        duration: Monitoring duration in seconds
        verbose: If True, show detailed diagnostic info on changes
    """
    start_time = time.time()
    last_diagnostic = None

    print(f"\nMonitoring power state for {duration}s...")
    if verbose:
        print(f"{'Time':<8} {'Diag':<6} {'Power State':<20}")
        print("=" * 80)
    else:
        print(f"{'Time':<8} {'Diag':<6} {'State':<25} {'Description':<50}")
        print("-" * 95)

    while time.time() - start_time < duration:
        status = device.read_status()
        diagnostic = status.get("diagnostic", 0xFF)

        # Only print when diagnostic changes
        if diagnostic != last_diagnostic:
            elapsed = time.time() - start_time
            desc = DIAGNOSTIC_CODES.get(diagnostic, "Unknown")
            state = get_power_state_description(diagnostic)

            if verbose:
                print(f"\n{elapsed:7.2f}s  0x{diagnostic:02X}   {state}")
                print(f"  {format_diagnostic_details(diagnostic)}")
            else:
                print(f"{elapsed:7.2f}s  0x{diagnostic:02X}   {state:<25} {desc}")

            last_diagnostic = diagnostic

        time.sleep(0.1)

    if verbose:
        print("=" * 80)
    else:
        print("-" * 95)


def test_power_on(device: SK2310g2):
    """Test power-on sequence."""
    print("\n" + "=" * 80)
    print("POWER ON SEQUENCE")
    print("=" * 80)

    success = device.power_on(timeout=60.0, verbose=True)

    if success:
        print("\n✓ Power ON successful!")

        # Read final state
        status = device.read_status()
        diagnostic = status.get("diagnostic", 0xFF)
        print(f"\nFinal Status:")
        print(f"  Diagnostic: 0x{diagnostic:02X}")
        print(f"  Power State: {get_power_state_description(diagnostic)}")
        print(f"  {format_diagnostic_details(diagnostic)}")
    else:
        print("\n✗ Power ON failed or timed out")


def test_power_off(device: SK2310g2):
    """Test power-off sequence."""
    print("\n" + "=" * 80)
    print("POWER OFF SEQUENCE")
    print("=" * 80)

    # Get current outputs
    current_outputs = (
        device.digital_outputs if device.digital_outputs is not None else 0
    )
    print(f"Current outputs: 0x{current_outputs:04X}")

    # Set bit 15 HIGH for 1s
    print("\nSetting bit 15 HIGH for 1 second...")
    outputs_high = current_outputs | (1 << 15)
    byte0 = outputs_high & 0xFF
    byte1 = (outputs_high >> 8) & 0xFF
    device.set_outputs(byte0, byte1)
    time.sleep(1.0)

    # Set bit 15 LOW
    print("Setting bit 15 LOW...")
    outputs_low = current_outputs & ~(1 << 15)
    byte0 = outputs_low & 0xFF
    byte1 = (outputs_low >> 8) & 0xFF
    device.set_outputs(byte0, byte1)

    print("\nExpecting: 0x00 (Power OFF delay) for ~4s, then power OFF state")

    # Monitor for power-off with verbose details
    monitor_power_state(device, duration=10.0, verbose=True)

    # Final state
    status = device.read_status()
    diagnostic = status.get("diagnostic", 0xFF)
    power_state = status.get("power_state", False)

    print(f"\nFinal Status:")
    print(f"  Diagnostic: 0x{diagnostic:02X}")
    print(f"  Power State: {get_power_state_description(diagnostic)}")
    print(f"  {format_diagnostic_details(diagnostic)}")

    if not power_state:
        print("\n✓ Power OFF successful!")
    else:
        print("\n✗ Power is still ON")


def main():
    """Main test function."""
    print("=" * 80)
    print("SK-2310g2 Power Control Test")
    print("=" * 80)

    # Connect to network
    print("\nConnecting to /dev/ttyUSB0...")
    network = LDCNNetwork("/dev/ttyUSB0")

    try:
        network.open()

        # Initialize with auto mode
        print("Initializing network...")
        network.initialize(mode=InitMode.AUTO)
        print(f"Found {len(network.devices)} devices")

        # List discovered devices
        print("\nDevices found:")
        for dev in network.devices:
            print(
                f"  Address {dev.address}: {dev.device_type} (class: {type(dev).__name__})"
            )

        # Find SK-2310g2 supervisor
        device = network.find_supervisor()
        if device is None:
            print("\nERROR: No SK-2310g2 found on network")
            return

        print(f"\nUsing SK-2310g2 at address {device.address}")

        # Configure status reporting
        print("\nConfiguring status reporting...")
        device.define_status(0x01)  # Digital inputs (includes diagnostic)
        time.sleep(0.1)

        # Check current power state
        print("\n" + "=" * 80)
        print("CURRENT POWER STATE")
        print("=" * 80)

        status = device.read_status()
        diagnostic = status.get("diagnostic", 0xFF)
        power_state = status.get("power_state", False)
        state_desc = get_power_state_description(diagnostic)

        print(f"  Diagnostic:   0x{diagnostic:02X}")
        print(f"  Power State:  {state_desc}")
        print(f"\n  {format_diagnostic_details(diagnostic)}")

        # Take action based on current state
        if power_state:
            # Power is ON - offer to turn OFF
            print("\n→ Power is currently ON")
            response = input("\nTurn power OFF? (y/n): ")
            if response.lower() == "y":
                test_power_off(device)
        else:
            # Power is OFF - offer to turn ON
            print("\n→ Power is currently OFF")
            response = input("\nTurn power ON? (y/n): ")
            if response.lower() == "y":
                test_power_on(device)

    finally:
        network.close()


if __name__ == "__main__":
    main()
