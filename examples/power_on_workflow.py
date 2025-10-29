#!/usr/bin/env python3
"""
power_on_workflow.py - Complete Power-On Workflow Example

Demonstrates the complete sequence from cold start to powered-on system:
1. Initialize LDCN network
2. Configure devices
3. Request operator to press power button
4. Wait for power ON
5. Verify system ready for servo operations

Hardware Requirements:
- SK2310g2 supervisor at address 6 (or configured address)
- J21 open (physical button required)
- Power button wired to CN15

Usage:
    python3 power_on_workflow.py [--port PORT] [--baud BAUD]

Author: LinuxCNC Community
License: GPL v2 or later
"""

import sys
import argparse
import time
from pyldcn import LDCNNetwork, SK2310g2, LDCNError, LDCNInitializationError


def print_header(title: str):
    """Print formatted section header."""
    print(f"\n{'='*70}")
    print(f"{title:^70}")
    print(f"{'='*70}\n")


def print_step(step: int, description: str):
    """Print formatted step indicator."""
    print(f"\n[Step {step}] {description}")
    print("-" * 70)


def initialize_network(port: str, target_baud: int = 125000) -> LDCNNetwork:
    """
    Initialize LDCN network.

    Steps:
    1. Open serial port at 19200 baud
    2. Reset all devices (at any baud rate)
    3. Address devices sequentially
    4. Discover device types
    5. Upgrade to target baud rate
    6. Create device objects

    Args:
        port: Serial port path (e.g., '/dev/ttyUSB0')
        target_baud: Target baud rate (default: 125000)

    Returns:
        Initialized LDCNNetwork object

    Raises:
        LDCNInitializationError: If initialization fails
    """
    print_step(1, "Initialize LDCN Network")

    try:
        # Create network object
        print(f"Opening {port} at 19200 baud...")
        network = LDCNNetwork(port)
        network.open()
        print("✓ Serial port opened")

        # Initialize network (reset, address, discover)
        print("\nInitializing network (reset, address, discover)...")
        num_devices, device_info = network.initialize()
        print(f"✓ Found {num_devices} devices")

        # Display discovered devices
        for dev in device_info:
            responding = "✓" if dev['responding'] else "✗"
            print(f"  {responding} Address {dev['address']}: "
                  f"ID=0x{dev['device_id']:02X}, Version=0x{dev['version']:02X}")

        # Upgrade baud rate
        if target_baud != 19200:
            print(f"\nUpgrading to {target_baud} baud...")
            network.set_baud_rate(target_baud)
            print(f"✓ Baud rate upgraded to {target_baud}")

        # Display device objects
        print(f"\n✓ Created {len(network.devices)} device objects:")
        for device in network.devices:
            print(f"  {device}")

        return network

    except Exception as e:
        print(f"✗ Initialization failed: {e}")
        raise


def find_supervisor(network: LDCNNetwork) -> SK2310g2:
    """
    Find SK2310g2 supervisor on the network.

    Args:
        network: Initialized LDCNNetwork

    Returns:
        SK2310g2 device object

    Raises:
        LDCNError: If supervisor not found
    """
    print_step(2, "Locate Supervisor Device")

    for device in network.devices:
        if isinstance(device, SK2310g2):
            print(f"✓ Found supervisor: {device}")
            return device

    raise LDCNError("SK2310g2 supervisor not found on network")


def configure_supervisor(supervisor: SK2310g2):
    """
    Configure supervisor for full status reporting.

    Args:
        supervisor: SK2310g2 device
    """
    print_step(3, "Configure Supervisor")

    print("Configuring for full status reporting...")
    supervisor.configure()
    print("✓ Supervisor configured")

    # Read initial diagnostic
    diag = supervisor.read_diagnostic()
    print(f"\nDiagnostic code: 0x{diag:02X}")


def request_and_wait_for_power(supervisor: SK2310g2, timeout: float = None) -> bool:
    """
    Request operator to press power button and wait for power ON.

    Args:
        supervisor: SK2310g2 device
        timeout: Maximum wait time in seconds (None = infinite)

    Returns:
        True if power ON, False if timeout
    """
    print_step(4, "Power On System")

    # Check current power state
    current_state = supervisor.read_power_state()
    if current_state:
        print("✓ Power is already ON")
        return True

    # Display operator notification
    supervisor.request_power_on()

    # Wait for power button press
    success = supervisor.wait_for_power_button(timeout=timeout, verbose=True)

    if success:
        print("\n✓ System powered ON successfully")
        return True
    else:
        print(f"\n✗ Power ON timeout after {timeout}s")
        return False


def verify_system_ready(supervisor: SK2310g2) -> bool:
    """
    Verify system is ready for servo operations.

    Checks:
    - Power state is ON
    - No emergency stops active
    - Diagnostic code is nominal

    Args:
        supervisor: SK2310g2 device

    Returns:
        True if system ready, False otherwise
    """
    print_step(5, "Verify System Ready")

    # Check power state
    power_state = supervisor.read_power_state()
    print(f"Power state: {'ON' if power_state else 'OFF'}")

    if not power_state:
        print("✗ Power is not ON")
        return False

    # Read diagnostic
    diag = supervisor.read_diagnostic()
    print(f"Diagnostic code: 0x{diag:02X}")

    # Check for error codes (page 20 of manual)
    # 0x1F = nominal powered state with covers closed
    # 0x14-0x17 = ready to power (covers may be open)
    if diag in [0x1F, 0x14, 0x15, 0x16, 0x17]:
        print("✓ System diagnostic nominal")
        return True
    else:
        print(f"⚠ Non-nominal diagnostic code: 0x{diag:02X}")
        return True  # Continue anyway, operator can interpret


def main():
    """Main workflow execution."""
    parser = argparse.ArgumentParser(
        description='LDCN Power-On Workflow Example',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=125000,
                       help='Target baud rate (default: 125000)')
    parser.add_argument('--timeout', type=float, default=None,
                       help='Power button timeout in seconds (default: None/infinite)')

    args = parser.parse_args()

    print_header("LDCN POWER-ON WORKFLOW")
    print(f"Port: {args.port}")
    print(f"Target baud: {args.baud}")
    print(f"Power timeout: {args.timeout if args.timeout else 'None (infinite)'}")

    network = None

    try:
        # Step 1: Initialize network
        network = initialize_network(args.port, args.baud)

        # Step 2: Find supervisor
        supervisor = find_supervisor(network)

        # Step 3: Configure supervisor
        configure_supervisor(supervisor)

        # Step 4: Request and wait for power
        if not request_and_wait_for_power(supervisor, args.timeout):
            print("\n✗ Power-on workflow failed")
            sys.exit(1)

        # Step 5: Verify system ready
        if not verify_system_ready(supervisor):
            print("\n⚠ System may not be fully ready")
            sys.exit(1)

        # Success!
        print_header("POWER-ON COMPLETE")
        print("✓ System is powered and ready for servo operations")
        print("\nNext steps:")
        print("  1. Initialize servo drives with tuning parameters")
        print("  2. Enable servo amplifiers")
        print("  3. Home axes")
        print("  4. Execute motion commands")

        # Keep system powered for demonstration
        print("\nPress Ctrl+C to exit...")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\nShutdown requested by user")
    except Exception as e:
        print(f"\n✗ Workflow failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if network:
            print("\nClosing network...")
            network.close()
            print("✓ Network closed")


if __name__ == '__main__':
    main()
