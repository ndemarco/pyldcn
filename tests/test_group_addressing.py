#!/usr/bin/env python3
"""
Comprehensive Group Addressing Tests

Tests group address assignment, group leader configuration, and group command
behavior with multiple groups and multiple devices per group.

Test scenarios:
1. Assign devices to multiple groups (some with leaders, some without)
2. Send NOP to individual devices (verify responses)
3. Send NOP to groups with leaders (verify leader responds)
4. Send NOP to groups without leaders (verify no timeout, empty response)
5. Remove devices from groups (reassign to broadcast 0xFF)
6. Remove group leaders (verify group commands no longer expect response)
7. Assign new group leaders (verify group commands now expect response)

Requirements:
- Real hardware LDCN network with at least 6 devices
- Port path specified via command line or defaults to /dev/ttyUSB0
"""

import sys
import argparse
from typing import List, Dict
from pyldcn import LDCNNetwork, InitMode
from pyldcn.device import LDCNDevice
from pyldcn.exceptions import LDCNTimeoutError, LDCNChecksumError


class TestResult:
    """Track test results"""
    def __init__(self, name: str):
        self.name = name
        self.passed = False
        self.message = ""

    def __repr__(self):
        status = "✅ PASS" if self.passed else "❌ FAIL"
        return f"{status}: {self.name}\n    {self.message}"


class GroupAddressingTest:
    """Comprehensive group addressing test suite"""

    def __init__(self, port: str, verbose: bool = False):
        self.port = port
        self.verbose = verbose
        self.network: LDCNNetwork = None
        self.devices: List[LDCNDevice] = []
        self.results: List[TestResult] = []

    def log(self, message: str):
        """Print message if verbose enabled"""
        if self.verbose:
            print(f"  {message}")

    def run_all_tests(self) -> bool:
        """Run all test scenarios. Returns True if all pass."""
        try:
            print(f"\n{'='*70}")
            print("Group Addressing Test Suite")
            print(f"{'='*70}\n")

            # Initialize network
            print("Initializing network...")
            self.network = LDCNNetwork(self.port)
            self.network.open()
            num_devices, device_info = self.network.initialize(mode=InitMode.FULL)
            print(f"Found {num_devices} devices\n")

            if num_devices < 6:
                print(f"❌ ERROR: Need at least 6 devices, found {num_devices}")
                return False

            # Upgrade to 125kbps
            self.log("Upgrading to 125kbps")
            self.network.set_baud_rate(125000)
            self.devices = self.network.devices

            # Run test scenarios
            self.test_scenario_1_assign_groups()
            self.test_scenario_2_individual_nops()
            self.test_scenario_3_group_nops_with_leaders()
            self.test_scenario_4_group_nops_without_leaders()
            self.test_scenario_5_remove_devices_from_groups()
            self.test_scenario_6_remove_group_leaders()
            self.test_scenario_7_reassign_group_leaders()

            # Print summary
            self.print_summary()

            return all(r.passed for r in self.results)

        finally:
            if self.network:
                self.network.close()

    def test_scenario_1_assign_groups(self):
        """Scenario 1: Assign devices to multiple groups"""
        print("\n" + "="*70)
        print("SCENARIO 1: Assign devices to groups")
        print("="*70)
        print("  Group 0xF0: Devices 1, 2, 3 (device 1 is leader)")
        print("  Group 0xF1: Devices 4, 5 (no leader)")
        print("  Device 6: Broadcast 0xFF (no group)")
        print()

        result = TestResult("Assign devices to groups")

        try:
            # Group 0xF0: devices 1, 2, 3 with device 1 as leader
            self.log("Assigning device 1 to group 0xF0 as leader")
            self.devices[0].set_address(1, 0xF0, is_leader=True)

            self.log("Assigning device 2 to group 0xF0 as member")
            self.devices[1].set_address(2, 0xF0, is_leader=False)

            self.log("Assigning device 3 to group 0xF0 as member")
            self.devices[2].set_address(3, 0xF0, is_leader=False)

            # Group 0xF1: devices 4, 5 with NO leader (all members)
            self.log("Assigning device 4 to group 0xF1 as member")
            self.devices[3].set_address(4, 0xF1, is_leader=False)

            self.log("Assigning device 5 to group 0xF1 as member")
            self.devices[4].set_address(5, 0xF1, is_leader=False)

            # Device 6: broadcast group 0xFF (always member, never leader)
            self.log("Assigning device 6 to broadcast group 0xFF as member")
            self.devices[5].set_address(6, 0xFF, is_leader=False)

            # Verify assignments
            assert self.devices[0].address == 1 and self.devices[0].group_address == 0xF0
            assert self.devices[1].address == 2 and self.devices[1].group_address == 0xF0
            assert self.devices[2].address == 3 and self.devices[2].group_address == 0xF0
            assert self.devices[3].address == 4 and self.devices[3].group_address == 0xF1
            assert self.devices[4].address == 5 and self.devices[4].group_address == 0xF1
            assert self.devices[5].address == 6 and self.devices[5].group_address == 0xFF

            # Verify leader
            assert self.network.has_group_leader(0xF0) == True
            assert self.network.has_group_leader(0xF1) == False
            assert self.network.get_group_leader(0xF0).address == 1

            result.passed = True
            result.message = "All devices assigned to correct groups with proper leader configuration"

        except Exception as e:
            result.passed = False
            result.message = f"Failed: {e}"

        print(result)
        self.results.append(result)

    def test_scenario_2_individual_nops(self):
        """Scenario 2: Send NOP to each individual device"""
        print("\n" + "="*70)
        print("SCENARIO 2: Send NOP to individual devices")
        print("="*70)
        print("  Verify each device responds at its individual address")
        print()

        for i, device in enumerate(self.devices[:6], 1):
            result = TestResult(f"NOP to device {i} at address {device.address}")

            try:
                self.log(f"Sending NOP to device {i} (address {device.address})")
                response = device.nop()

                # Verify response has valid checksum (len >= 2)
                if len(response) >= 2:
                    result.passed = True
                    result.message = f"Device responded: {response.hex()}"
                else:
                    result.passed = False
                    result.message = f"Invalid response: {response.hex()}"

            except (LDCNTimeoutError, LDCNChecksumError) as e:
                result.passed = False
                result.message = f"Communication error: {e}"

            print(result)
            self.results.append(result)

    def test_scenario_3_group_nops_with_leaders(self):
        """Scenario 3: Send NOP to groups WITH hardware-configured leaders"""
        print("\n" + "="*70)
        print("SCENARIO 3: Send NOP to group WITH leader")
        print("="*70)
        print("  Group 0xF0 has device 1 as hardware-configured leader - should respond")
        print()

        result = TestResult("NOP to group 0xF0 (hardware leader configured)")

        try:
            self.log("Sending NOP to group 0xF0")
            response = self.network.send_command(0xF0, 0x0E)  # NOP command

            # Should receive response from hardware-configured leader (device 1)
            if len(response) >= 2:
                result.passed = True
                result.message = f"Hardware leader responded: {response.hex()}"
            else:
                result.passed = False
                result.message = f"No response from hardware leader (unexpected)"

        except LDCNTimeoutError:
            result.passed = False
            result.message = "Timeout - hardware leader should have responded"
        except Exception as e:
            result.passed = False
            result.message = f"Error: {e}"

        print(result)
        self.results.append(result)

    def test_scenario_4_group_nops_without_leaders(self):
        """Scenario 4: Send NOP to groups WITHOUT leaders"""
        print("\n" + "="*70)
        print("SCENARIO 4: Send NOP to groups WITHOUT leaders")
        print("="*70)
        print("  Group 0xF1 has no leader - should NOT timeout")
        print("  Broadcast 0xFF never has leader - should NOT timeout")
        print()

        # Test group 0xF1 (no leader)
        result1 = TestResult("NOP to group 0xF1 (no leader)")
        try:
            self.log("Sending NOP to group 0xF1 (no leader)")
            response = self.network.send_command(0xF1, 0x0E)

            # Should NOT timeout, should return empty response
            if len(response) == 0:
                result1.passed = True
                result1.message = "No response (expected - no leader configured)"
            else:
                result1.passed = False
                result1.message = f"Unexpected response: {response.hex()}"

        except LDCNTimeoutError:
            result1.passed = False
            result1.message = "Timeout - should NOT timeout when no leader"
        except Exception as e:
            result1.passed = False
            result1.message = f"Error: {e}"

        print(result1)
        self.results.append(result1)

        # Test broadcast 0xFF (never has leader)
        result2 = TestResult("NOP to broadcast 0xFF (never has leader)")
        try:
            self.log("Sending NOP to broadcast 0xFF")
            response = self.network.send_command(0xFF, 0x0E)

            # Should NOT timeout, should return empty response
            if len(response) == 0:
                result2.passed = True
                result2.message = "No response (expected - broadcast never responds)"
            else:
                result2.passed = False
                result2.message = f"Unexpected response: {response.hex()}"

        except LDCNTimeoutError:
            result2.passed = False
            result2.message = "Timeout - broadcast should NOT timeout"
        except Exception as e:
            result2.passed = False
            result2.message = f"Error: {e}"

        print(result2)
        self.results.append(result2)

    def test_scenario_5_remove_devices_from_groups(self):
        """Scenario 5: Remove devices from groups (reassign to broadcast)"""
        print("\n" + "="*70)
        print("SCENARIO 5: Remove devices from groups")
        print("="*70)
        print("  Reassign devices 2 and 3 from group 0xF0 to broadcast 0xFF")
        print()

        result = TestResult("Remove devices from group")

        try:
            # Remove devices 2 and 3 from group 0xF0
            self.log("Reassigning device 2 to broadcast 0xFF as member")
            self.devices[1].set_address(2, 0xFF, is_leader=False)

            self.log("Reassigning device 3 to broadcast 0xFF as member")
            self.devices[2].set_address(3, 0xFF, is_leader=False)

            # Verify they still respond individually
            self.log("Verifying device 2 still responds individually")
            response2 = self.devices[1].nop()

            self.log("Verifying device 3 still responds individually")
            response3 = self.devices[2].nop()

            if len(response2) >= 2 and len(response3) >= 2:
                result.passed = True
                result.message = "Devices removed from group, still respond individually"
            else:
                result.passed = False
                result.message = "Devices failed to respond after group removal"

        except Exception as e:
            result.passed = False
            result.message = f"Error: {e}"

        print(result)
        self.results.append(result)

        # Verify group 0xF0 still works (device 1 is still leader)
        result2 = TestResult("Group 0xF0 still works with remaining leader")
        try:
            self.log("Verifying group 0xF0 still responds (device 1 still leader)")
            response = self.network.send_command(0xF0, 0x0E)

            if len(response) >= 2:
                result2.passed = True
                result2.message = "Group leader still responds"
            else:
                result2.passed = False
                result2.message = "Group leader failed to respond"

        except Exception as e:
            result2.passed = False
            result2.message = f"Error: {e}"

        print(result2)
        self.results.append(result2)

    def test_scenario_6_remove_group_leaders(self):
        """Scenario 6: Remove group leaders"""
        print("\n" + "="*70)
        print("SCENARIO 6: Remove group leaders")
        print("="*70)
        print("  Remove device 1 as leader of group 0xF0")
        print()

        result = TestResult("Remove group leader")

        try:
            self.log("Removing device 1 as leader of group 0xF0")
            self.network.set_group_leader(self.devices[0], is_leader=False)

            # Verify leader was removed
            assert self.network.has_group_leader(0xF0) == False

            result.passed = True
            result.message = "Group leader removed successfully"

        except Exception as e:
            result.passed = False
            result.message = f"Error: {e}"

        print(result)
        self.results.append(result)

        # Test that group 0xF0 now doesn't expect response
        result2 = TestResult("Group 0xF0 without leader - no timeout")
        try:
            self.log("Sending NOP to group 0xF0 (no leader now)")
            response = self.network.send_command(0xF0, 0x0E)

            # Should NOT timeout, should return empty response
            if len(response) == 0:
                result2.passed = True
                result2.message = "No response (expected - no leader)"
            else:
                result2.passed = False
                result2.message = f"Unexpected response: {response.hex()}"

        except LDCNTimeoutError:
            result2.passed = False
            result2.message = "Timeout - should NOT timeout when no leader"
        except Exception as e:
            result2.passed = False
            result2.message = f"Error: {e}"

        print(result2)
        self.results.append(result2)

    def test_scenario_7_reassign_group_leaders(self):
        """Scenario 7: Reassign new group leaders"""
        print("\n" + "="*70)
        print("SCENARIO 7: Reassign group leaders")
        print("="*70)
        print("  Make device 4 the leader of group 0xF1")
        print("  Make device 1 the leader of group 0xF0 again")
        print()

        # Assign device 4 as leader of group 0xF1
        result1 = TestResult("Assign device 4 as leader of group 0xF1")
        try:
            self.log("Setting device 4 as leader of group 0xF1")
            self.network.set_group_leader(self.devices[3], is_leader=True)

            # Verify leader was set
            assert self.network.has_group_leader(0xF1) == True
            assert self.network.get_group_leader(0xF1).address == 4

            result1.passed = True
            result1.message = "Device 4 is now leader of group 0xF1"

        except Exception as e:
            result1.passed = False
            result1.message = f"Error: {e}"

        print(result1)
        self.results.append(result1)

        # Test group 0xF1 now expects response
        result2 = TestResult("Group 0xF1 with new leader - expects response")
        try:
            self.log("Sending NOP to group 0xF1 (device 4 is leader)")
            response = self.network.send_command(0xF1, 0x0E)

            # Should receive response from leader (device 4)
            if len(response) >= 2:
                result2.passed = True
                result2.message = f"Leader responded (expected): {response.hex()}"
            else:
                result2.passed = False
                result2.message = "No response from leader (unexpected)"

        except LDCNTimeoutError:
            result2.passed = False
            result2.message = "Timeout - leader should have responded"
        except Exception as e:
            result2.passed = False
            result2.message = f"Error: {e}"

        print(result2)
        self.results.append(result2)

        # Reassign device 1 as leader of group 0xF0
        result3 = TestResult("Reassign device 1 as leader of group 0xF0")
        try:
            self.log("Reassigning device 1 as leader of group 0xF0")
            self.network.set_group_leader(self.devices[0], is_leader=True)

            # Verify leader was set
            assert self.network.has_group_leader(0xF0) == True
            assert self.network.get_group_leader(0xF0).address == 1

            # Test that group 0xF0 now expects response
            self.log("Sending NOP to group 0xF0 (device 1 is leader again)")
            response = self.network.send_command(0xF0, 0x0E)

            if len(response) >= 2:
                result3.passed = True
                result3.message = f"Leader responded (expected): {response.hex()}"
            else:
                result3.passed = False
                result3.message = "No response from leader (unexpected)"

        except Exception as e:
            result3.passed = False
            result3.message = f"Error: {e}"

        print(result3)
        self.results.append(result3)

    def print_summary(self):
        """Print test summary"""
        print("\n" + "="*70)
        print("TEST SUMMARY")
        print("="*70)

        passed = sum(1 for r in self.results if r.passed)
        failed = sum(1 for r in self.results if not r.passed)
        total = len(self.results)

        print(f"\nTotal Tests: {total}")
        print(f"Passed: {passed} ✅")
        print(f"Failed: {failed} ❌")
        print(f"Success Rate: {passed/total*100:.1f}%\n")

        if failed > 0:
            print("Failed Tests:")
            for result in self.results:
                if not result.passed:
                    print(f"  ❌ {result.name}")
                    print(f"     {result.message}")
            print()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Comprehensive group addressing tests for LDCN network"
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port path (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Verbose output"
    )
    args = parser.parse_args()

    # Run tests
    tester = GroupAddressingTest(args.port, args.verbose)
    success = tester.run_all_tests()

    # Exit with appropriate code
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
