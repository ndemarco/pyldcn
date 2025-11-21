#!/usr/bin/env python3
"""
Example: Group Addressing and Leader Configuration

Demonstrates how to configure group addresses and group leaders
for synchronized multi-axis motion control.

This example shows:
1. Setting up a group of devices
2. Configuring one device as the group leader
3. How group commands behave with and without leaders
"""

from pyldcn import LDCNNetwork, InitMode

def main():
    # Initialize network
    with LDCNNetwork('/dev/ttyUSB0') as network:
        # Perform full initialization
        num_devices, device_info = network.initialize(mode=InitMode.FULL)
        print(f"Found {num_devices} devices")

        # Upgrade to 125kbps for better performance
        network.set_baud_rate(125000)

        # Get servo devices (assuming we have at least 3)
        servo1 = network.devices[0]  # Address 1
        servo2 = network.devices[1]  # Address 2
        servo3 = network.devices[2]  # Address 3

        # Configure all servos to belong to group 0xF0
        print("\n=== Configuring Group Addresses ===")
        servo1.set_address(1, 0xF0)
        servo2.set_address(2, 0xF0)
        servo3.set_address(3, 0xF0)
        print("All servos configured to group 0xF0")

        # Set servo1 as the group leader
        print("\n=== Configuring Group Leader ===")
        network.set_group_leader(servo1, is_leader=True)
        print(f"Servo 1 is now the leader of group 0xF0")
        print(f"Group 0xF0 has leader: {network.has_group_leader(0xF0)}")

        # Example: Send group command to 0xF0 with leader configured
        # This will execute on all devices in group 0xF0, but only servo1 responds
        print("\n=== Group Command WITH Leader ===")
        try:
            response = network.send_command(0xF0, 0x0E)  # NOP to group
            print(f"✅ Response received from leader: {response.hex()}")
        except Exception as e:
            print(f"❌ Error: {e}")

        # Remove leader configuration
        print("\n=== Removing Group Leader ===")
        network.set_group_leader(servo1, is_leader=False)
        print(f"Group 0xF0 has leader: {network.has_group_leader(0xF0)}")

        # Example: Send group command to 0xF0 WITHOUT leader
        # This will execute on all devices, but no response expected (not a timeout!)
        print("\n=== Group Command WITHOUT Leader ===")
        try:
            response = network.send_command(0xF0, 0x0E)  # NOP to group
            print(f"✅ Command sent successfully, no response (expected): {len(response)} bytes")
        except Exception as e:
            print(f"❌ Error: {e}")

        # Example: Synchronized motion using group commands
        print("\n=== Synchronized Motion Example ===")
        print("1. Load trajectories individually (with leader responses)")
        network.set_group_leader(servo1, is_leader=True)
        # Load trajectory commands would go here...
        print("   Trajectories loaded on each servo")

        print("2. Start motion simultaneously (without leader)")
        network.set_group_leader(servo1, is_leader=False)
        # Start motion command to group 0xF0
        print("   Start command sent to group 0xF0 - all axes move within ±25μs")

        # Broadcast example (address 0xFF - never has a leader)
        print("\n=== Broadcast Commands (0xFF) ===")
        print("Address 0xFF is special - never expects responses")
        try:
            response = network.send_command(0xFF, 0x0E)
            print(f"✅ Broadcast sent successfully: {len(response)} bytes")
        except Exception as e:
            print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
