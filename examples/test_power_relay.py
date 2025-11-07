#!/usr/bin/env python3
"""
Test SK-2310g2 power relay enablement.

This script tests enabling the power relay (Output 15) on the SK-2310g2,
which should make the power button flash.
"""

import sys
from pyldcn import LDCNNetwork

def main():
    print("="*60)
    print("SK-2310g2 Power Relay Test")
    print("="*60)

    network = None
    try:
        # Initialize network
        print("\n1. Connecting to /dev/ttyUSB0...")
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        # Find SK-2310g2
        print("\n2. Finding SK-2310g2 I/O controller...")
        io_controller = None
        for dev in network.devices:
            if hasattr(dev, 'device_type') and dev.device_type == 'SK-2310g2':
                io_controller = dev
                break

        if not io_controller:
            print("ERROR: SK-2310g2 not found on network")
            return 1

        print(f"   Found SK-2310g2 at address {io_controller.address}")

        # Configure SK-2310g2 (REQUIRED before setting outputs)
        print("\n3. Configuring SK-2310g2 (DEFINE_STATUS)...")
        io_controller.configure()
        print("   ✓ SK-2310g2 configured")

        # Read initial diagnostic
        print("\n4. Reading initial diagnostic code...")
        initial_diag = io_controller.read_diagnostic()
        print(f"   Initial diagnostic: 0x{initial_diag:02X}")

        # Enable power relay (Output 15)
        print("\n5. Enabling power relay (Output 15 = 0x8000)...")
        print("   Sending SET_OUTPUTS command: [0x00, 0x80]")
        io_controller.set_outputs(0x8000)
        print("   ✓ Command sent")

        # Check if button is flashing
        print("\n" + "="*60)
        print("CHECK: Is the power button now FLASHING? (Y/N)")
        print("="*60)
        print("\nIf YES: The fix worked! Press the power button and")
        print("        the diagnostic code should change.")
        print("\nIf NO:  The button is not flashing. There may be")
        print("        another issue to investigate.")

        # Wait for user input
        response = input("\nIs the button flashing? (Y/N): ").strip().upper()

        if response == 'Y':
            print("\n✓ SUCCESS! The power relay is working correctly.")
            print("\nPress the power button, then press Enter...")
            input()

            # Read diagnostic after button press
            final_diag = io_controller.read_diagnostic()
            print(f"\nDiagnostic after button press: 0x{final_diag:02X}")

            if final_diag != initial_diag and final_diag >= 0x08:
                print("✓ Diagnostic changed correctly! Power-on detected.")
                return 0
            else:
                print("⚠ Diagnostic did not change as expected.")
                return 1
        else:
            print("\n⚠ Button is not flashing. Issue not resolved.")
            print("\nDebugging info:")
            print(f"  - SK-2310g2 address: {io_controller.address}")
            print(f"  - Diagnostic code: 0x{initial_diag:02X}")
            return 1

    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if network:
            print("\n6. Closing network connection...")
            network.close()
            print("   ✓ Network closed")

if __name__ == "__main__":
    sys.exit(main())
