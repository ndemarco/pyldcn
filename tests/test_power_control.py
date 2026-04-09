#!/usr/bin/env python3
"""
Unified power control test for SK-2310g2

Tests power on/off using structured diagnostic code API.
"""

import time
from pyldcn import LDCNNetwork, InitMode
from pyldcn.devices.io import SK2310g2
from pyldcn.devices.sk2310g2 import get_diagnostic_state


def get_power_state_description(diagnostic: int) -> str:
    """Get human-readable power state from diagnostic code.

    Uses diagnostic state attributes:
    - POWER ON: power_enabled == "ON" and not in transition
    - READY TO POWER: ready_to_power == True
    - POWER OFF: All other states
    """
    state = get_diagnostic_state(diagnostic)
    if state is None:
        return "UNKNOWN"

    # Special case: Power OFF delay (0x00) is transitioning, not truly ON
    # It has power_enabled="ON" temporarily but error_type="INIT"
    if state.error_type == "INIT" and state.power_enabled == "ON":
        return "POWER OFF"

    # Power is ON if outputs are active
    if state.power_enabled == "ON":
        return "POWER ON"

    # Ready to power: system ready to accept power command
    if state.ready_to_power:
        return "READY TO POWER (OFF)"

    # All other states are power OFF
    return "POWER OFF"


def format_diagnostic_details(diagnostic: int) -> str:
    """Format detailed diagnostic information."""
    state = get_diagnostic_state(diagnostic)
    if state is None:
        return "Unknown diagnostic code"

    lines = []
    lines.append(f"Condition: {state.condition}")

    # Guard states
    if state.guard_1 is not None or state.guard_2 is not None:
        lines.append("Guards:")
        if state.guard_1 is not None:
            lines.append(f"  Guard 1: {state.guard_1}")
        if state.guard_2 is not None:
            lines.append(f"  Guard 2: {state.guard_2}")

    # Safety status
    if state.safe_zone is not None:
        lines.append(f"Safe Zone: {'Yes' if state.safe_zone else 'No'}")
    if state.spindle_stopped is not None:
        lines.append(f"Spindle: {'Stopped' if state.spindle_stopped else 'Running'}")
    if state.manual_override is not None:
        lines.append(f"Manual Override: {'Active' if state.manual_override else 'Inactive'}")

    # Power status
    lines.append(f"Power Ready: {state.power_ready or 'N/A'}")
    lines.append(f"Power Enabled: {state.power_enabled or 'N/A'}")

    # Error classification
    if state.error_type:
        lines.append(f"Error Type: {state.error_type}")

    return "\n  ".join(lines)


def monitor_power_state(device: SK2310g2, duration: float = 10.0, verbose: bool = False):
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
            diag_state = get_diagnostic_state(diagnostic)
            desc = diag_state.condition if diag_state else "Unknown"
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
    device.set_outputs(outputs_high, validate_safety=False)
    time.sleep(1.0)

    # Set bit 15 LOW
    print("Setting bit 15 LOW...")
    outputs_low = current_outputs & ~(1 << 15)
    device.set_outputs(outputs_low, validate_safety=False)

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
        diag_state = get_diagnostic_state(diagnostic)
        desc = diag_state.condition if diag_state else "Unknown"
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
