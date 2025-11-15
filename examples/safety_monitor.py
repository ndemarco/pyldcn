#!/usr/bin/env python3
"""
safety_monitor.py - LDCN Safety System Monitor

Monitors all LDCN devices to identify which device(s) triggered safety shutdown.
Uses routine polling with heuristic analysis to determine the first trigger.

Author: NickyDoes
License: GPL v2 or later
Date: 2025-11-05
"""

import time
from typing import Dict, List, Optional, Tuple
from datetime import datetime
from dataclasses import dataclass, field


@dataclass
class DeviceState:
    """Tracks safety-related state for a single device"""
    address: int
    device_type: str

    # Current state
    servo_fault: bool = False
    safety_link_out: bool = True  # True = OK (relay closed)
    safety_link_in: bool = True   # True = OK (receiving 24V)
    enable_stop: bool = True      # True = enabled
    at_home: bool = False
    test_mode: bool = False
    diagnostic_code: Optional[int] = None

    # Historical tracking
    last_update: float = field(default_factory=time.time)
    fault_timestamp: Optional[float] = None
    fault_history: List[Tuple[float, str]] = field(default_factory=list)

    # Communication tracking
    consecutive_failures: int = 0
    last_successful_poll: float = field(default_factory=time.time)

    def update_fault_state(self, fault: bool, fault_type: str):
        """Update fault state and record timestamp if transitioning to fault"""
        if fault and not self.servo_fault:
            # Transitioning from OK to FAULT
            self.fault_timestamp = time.time()
            self.fault_history.append((self.fault_timestamp, fault_type))
        elif not fault and self.servo_fault:
            # Clearing fault
            self.fault_timestamp = None

        self.servo_fault = fault
        self.last_update = time.time()

    def mark_poll_success(self):
        """Mark successful poll"""
        self.consecutive_failures = 0
        self.last_successful_poll = time.time()

    def mark_poll_failure(self):
        """Mark failed poll"""
        self.consecutive_failures += 1


class SafetyMonitor:
    """
    Monitors LDCN network safety system state.

    Uses routine CMD_NOP polling to track all devices and identify
    which device triggered a safety shutdown.

    Example:
        from pyldcn import LDCNNetwork
        from safety_monitor import SafetyMonitor

        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        network.initialize()
        network.set_baud_rate(125000)

        monitor = SafetyMonitor(network)

        # One-shot analysis
        report = monitor.analyze_safety_state()
        print(report)

        # Continuous monitoring
        monitor.start_monitoring(interval=0.5, duration=10.0)
    """

    def __init__(self, network):
        """
        Initialize safety monitor.

        Args:
            network: LDCNNetwork instance (must be initialized)
        """
        self.network = network
        self.devices: Dict[int, DeviceState] = {}
        self.sk2310g2_address: Optional[int] = None

        # Initialize device tracking
        self._initialize_device_tracking()

    def _initialize_device_tracking(self):
        """Initialize state tracking for all devices"""
        for device in self.network.devices:
            state = DeviceState(
                address=device.address,
                device_type=device.device_type
            )
            self.devices[device.address] = state

            # Track SK-2310g2 supervisor
            if device.device_type == 'SK-2310g2':
                self.sk2310g2_address = device.address

    def poll_all_devices(self) -> Dict[int, bool]:
        """
        Poll all devices using CMD_NOP and update state.

        Returns:
            Dictionary mapping address to success status
        """
        poll_results = {}

        for device in self.network.devices:
            addr = device.address
            state = self.devices[addr]

            try:
                # Use CMD_NOP (0x00) which returns status in response
                # This is the lowest-overhead status query
                status = device.read_status()

                # Update device-specific state
                if device.device_type == 'SK-2310g2':
                    self._update_sk2310g2_state(device, status, state)
                elif device.device_type == 'LS-231SE':
                    self._update_ls231se_state(device, status, state)
                else:
                    # Generic device - just track communication
                    state.mark_poll_success()

                poll_results[addr] = True

            except Exception as e:
                state.mark_poll_failure()
                poll_results[addr] = False

        return poll_results

    def _update_sk2310g2_state(self, device, status, state: DeviceState):
        """Update SK-2310g2 specific state from status response"""
        # Read diagnostic code
        try:
            diag = device.read_diagnostic()
            state.diagnostic_code = diag
        except:
            pass

        # Extract safety-related inputs from digital_inputs
        # Format: Byte0 (LSB) = application I/O, Byte1 (MSB) = status/safety
        if 'digital_inputs' in status:
            inputs = status['digital_inputs']
            byte1 = (inputs >> 8) & 0xFF

            # Byte1 mapping (from sk2310g2_safety.md):
            # Bit 0: At Home
            # Bit 1: Test Mode
            # Bit 2: ServoFAULT (active high)
            state.at_home = bool(byte1 & 0x01)
            state.test_mode = bool(byte1 & 0x02)
            servo_fault = bool(byte1 & 0x04)

            # Update fault state
            state.update_fault_state(servo_fault, "ServoFAULT input active")

            # Safety Link status from diagnostic code
            # Diagnostic bit 1 = Safety Link OK when 1
            if state.diagnostic_code is not None:
                state.safety_link_in = bool(state.diagnostic_code & 0x02)

        state.mark_poll_success()

    def _update_ls231se_state(self, device, status, state: DeviceState):
        """Update LS-231SE servo drive state from status response"""
        # Extract fault status from aux_status_byte
        # Bit 1 = Servo Fault (active high)
        if 'aux_status_byte' in status:
            aux = status['aux_status_byte']
            servo_fault = bool((aux >> 1) & 0x01)

            # Determine fault type if present
            if servo_fault:
                fault_code = (aux >> 4) & 0x0F
                fault_type = f"Servo fault code 0x{fault_code:X}"
            else:
                fault_type = "OK"

            state.update_fault_state(servo_fault, fault_type)

        state.mark_poll_success()

    def identify_trigger_devices(self) -> List[Tuple[int, DeviceState, str]]:
        """
        Identify which device(s) triggered the safety shutdown.

        Uses heuristic analysis:
        1. Find all devices currently in fault
        2. Sort by fault timestamp (earliest = likely trigger)
        3. Check Safety Link chain integrity
        4. Identify communication failures

        Returns:
            List of (address, state, reason) tuples, sorted by likelihood
        """
        triggers = []

        # Check for devices with active faults
        faulted_devices = [
            (addr, state) for addr, state in self.devices.items()
            if state.servo_fault
        ]

        # Sort by fault timestamp (earliest first)
        faulted_devices.sort(key=lambda x: x[1].fault_timestamp or float('inf'))

        for addr, state in faulted_devices:
            if state.fault_timestamp:
                elapsed = time.time() - state.fault_timestamp
                reason = f"Servo fault detected {elapsed:.1f}s ago"
                if state.fault_history:
                    _, fault_type = state.fault_history[-1]
                    reason += f": {fault_type}"
            else:
                reason = "Servo fault (timing unknown)"

            triggers.append((addr, state, reason))

        # Check for Safety Link chain issues
        for addr, state in self.devices.items():
            if not state.safety_link_in:
                reason = "Safety Link IN = LOW (upstream device fault or wiring issue)"
                triggers.append((addr, state, reason))

        # Check for communication failures (device may have powered off/disconnected)
        for addr, state in self.devices.items():
            if state.consecutive_failures >= 3:
                elapsed = time.time() - state.last_successful_poll
                reason = f"Communication lost {elapsed:.1f}s ago ({state.consecutive_failures} failures)"
                triggers.append((addr, state, reason))

        return triggers

    def analyze_safety_state(self) -> str:
        """
        Perform one-shot analysis of current safety state.

        Returns:
            Formatted report string
        """
        # Poll all devices once
        poll_results = self.poll_all_devices()

        # Build report
        lines = []
        lines.append("=" * 70)
        lines.append("LDCN SAFETY SYSTEM ANALYSIS")
        lines.append(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
        lines.append("=" * 70)

        # Overall system state
        lines.append("\n[SYSTEM STATE]")
        if self.sk2310g2_address is not None:
            sk_state = self.devices[self.sk2310g2_address]
            lines.append(f"SK-2310g2 (Addr {self.sk2310g2_address}):")
            if sk_state.diagnostic_code is not None:
                lines.append(f"  Diagnostic Code: 0x{sk_state.diagnostic_code:02X}")
                lines.append(f"  {self._decode_diagnostic(sk_state.diagnostic_code)}")
            lines.append(f"  ServoFAULT Input: {'ACTIVE' if sk_state.servo_fault else 'Clear'}")
            lines.append(f"  Safety Link IN:   {'OK' if sk_state.safety_link_in else 'FAULT'}")
            lines.append(f"  At Home:          {'Yes' if sk_state.at_home else 'No'}")
            lines.append(f"  Test Mode:        {'Active' if sk_state.test_mode else 'Inactive'}")

        # Device status table
        lines.append("\n[DEVICE STATUS]")
        lines.append(f"{'Addr':<6} {'Type':<12} {'Fault':<8} {'SafeLink':<10} {'Poll':<8} {'Notes':<30}")
        lines.append("-" * 70)

        for addr in sorted(self.devices.keys()):
            state = self.devices[addr]
            fault_str = "FAULT" if state.servo_fault else "OK"
            link_str = "OK" if state.safety_link_in else "FAULT"
            poll_str = "OK" if poll_results.get(addr, False) else "FAIL"

            notes = []
            if state.consecutive_failures > 0:
                notes.append(f"{state.consecutive_failures} failures")
            if state.fault_history:
                notes.append(f"{len(state.fault_history)} events")
            notes_str = ", ".join(notes)

            lines.append(f"{addr:<6} {state.device_type:<12} {fault_str:<8} {link_str:<10} {poll_str:<8} {notes_str:<30}")

        # Trigger analysis
        triggers = self.identify_trigger_devices()
        if triggers:
            lines.append("\n[TRIGGER ANALYSIS]")
            lines.append("Devices that likely triggered safety shutdown (earliest first):\n")

            for i, (addr, state, reason) in enumerate(triggers, 1):
                lines.append(f"{i}. Address {addr} ({state.device_type}):")
                lines.append(f"   → {reason}")

                if state.fault_history:
                    lines.append(f"   Fault history ({len(state.fault_history)} events):")
                    for ts, fault_type in state.fault_history[-3:]:  # Show last 3
                        time_str = datetime.fromtimestamp(ts).strftime('%H:%M:%S.%f')[:-3]
                        lines.append(f"     • {time_str}: {fault_type}")
        else:
            lines.append("\n[TRIGGER ANALYSIS]")
            lines.append("✓ No safety triggers identified - system appears normal")

        # Safety Link chain integrity
        lines.append("\n[SAFETY LINK CHAIN]")
        chain_ok = all(state.safety_link_in for state in self.devices.values())
        if chain_ok:
            lines.append("✓ Safety Link daisy-chain is intact")
        else:
            lines.append("⚠ Safety Link chain has breaks:")
            for addr, state in self.devices.items():
                if not state.safety_link_in:
                    lines.append(f"  • Address {addr} ({state.device_type}): Safety Link IN = LOW")

        lines.append("\n" + "=" * 70)

        return "\n".join(lines)

    def start_monitoring(self, interval: float = 0.5, duration: Optional[float] = None):
        """
        Start continuous monitoring with periodic polling.

        Args:
            interval: Polling interval in seconds (default 0.5s)
            duration: Total monitoring duration in seconds (None = infinite)

        Prints real-time updates when state changes are detected.
        """
        print(f"Starting safety monitor (polling every {interval}s)...")
        print("Press Ctrl+C to stop\n")

        start_time = time.time()
        last_trigger_count = 0

        try:
            while True:
                # Check duration limit
                if duration is not None and (time.time() - start_time) > duration:
                    break

                # Poll all devices
                poll_results = self.poll_all_devices()

                # Check for new triggers
                triggers = self.identify_trigger_devices()

                # Report changes
                if len(triggers) != last_trigger_count:
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    if triggers:
                        print(f"\n[{timestamp}] ⚠ Safety trigger detected:")
                        for addr, state, reason in triggers[:3]:  # Show top 3
                            print(f"  • Addr {addr} ({state.device_type}): {reason}")
                    else:
                        print(f"\n[{timestamp}] ✓ All triggers cleared")

                    last_trigger_count = len(triggers)

                # Report communication failures
                for addr, success in poll_results.items():
                    state = self.devices[addr]
                    if state.consecutive_failures == 3:  # First time hitting threshold
                        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                        print(f"\n[{timestamp}] ⚠ Communication lost with Addr {addr} ({state.device_type})")

                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user")

        # Final report
        print("\n" + self.analyze_safety_state())

    def _decode_diagnostic(self, code: int) -> str:
        """Decode SK-2310g2 diagnostic code to human-readable string"""
        from pyldcn.devices.sk2310g2 import get_diagnostic_state

        state = get_diagnostic_state(code)
        if state:
            return state.condition
        return f"Unknown diagnostic code 0x{code:02X}"


def main():
    """Command-line interface for safety monitor"""
    import argparse
    from pyldcn import LDCNNetwork

    parser = argparse.ArgumentParser(
        description='LDCN Safety System Monitor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # One-shot analysis
  python -m pyldcn.safety_monitor /dev/ttyUSB0

  # Continuous monitoring for 30 seconds
  python -m pyldcn.safety_monitor /dev/ttyUSB0 --monitor --duration 30

  # Continuous monitoring with 1s polling interval
  python -m pyldcn.safety_monitor /dev/ttyUSB0 --monitor --interval 1.0
        """
    )

    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=125000, help='Baud rate (default: 125000)')
    parser.add_argument('--monitor', action='store_true', help='Enable continuous monitoring')
    parser.add_argument('--interval', type=float, default=0.5, help='Polling interval in seconds (default: 0.5)')
    parser.add_argument('--duration', type=float, help='Monitoring duration in seconds (default: infinite)')

    args = parser.parse_args()

    # Initialize network
    print(f"Connecting to {args.port} at {args.baud} baud...")
    network = LDCNNetwork(args.port)

    try:
        network.open()
        network.initialize()
        if args.baud != 125000:
            network.set_baud_rate(args.baud)

        print(f"Found {len(network.devices)} devices\n")

        # Create monitor
        monitor = SafetyMonitor(network)

        if args.monitor:
            # Continuous monitoring
            monitor.start_monitoring(interval=args.interval, duration=args.duration)
        else:
            # One-shot analysis
            print(monitor.analyze_safety_state())

    finally:
        network.close()


if __name__ == '__main__':
    main()
