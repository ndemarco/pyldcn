#!/usr/bin/env python3
"""
LDCN Capture Analyzer

Parse and analyze captured LDCN frames from the snooper.
Reads CSV output from the snooper and provides basic frame analysis.

Usage:
    python3 analyze_capture.py capture.csv
    python3 analyze_capture.py --live /dev/ttyACM0
"""

import sys
import argparse
from collections import defaultdict
from datetime import timedelta


class FrameStats:
    """Statistics for captured frames."""

    def __init__(self):
        self.tx_count = 0
        self.rx_count = 0
        self.total_bytes_tx = 0
        self.total_bytes_rx = 0
        self.devices_seen = set()
        self.commands_seen = defaultdict(int)
        self.status_seen = defaultdict(int)
        self.first_timestamp = None
        self.last_timestamp = None

    def add_frame(self, timestamp, direction, payload):
        """Add a frame to statistics."""
        if self.first_timestamp is None:
            self.first_timestamp = timestamp

        self.last_timestamp = timestamp

        # Parse basic frame structure (assuming LDCN format)
        if len(payload) >= 3:
            address = payload[1]
            cmd_status = payload[2]

            self.devices_seen.add(address)

            if direction == "tx":
                self.tx_count += 1
                self.total_bytes_tx += len(payload)
                self.commands_seen[cmd_status] += 1
            else:
                self.rx_count += 1
                self.total_bytes_rx += len(payload)
                self.status_seen[cmd_status] += 1

    def print_summary(self):
        """Print statistics summary."""
        print("\n" + "=" * 60)
        print("LDCN Capture Analysis Summary")
        print("=" * 60)

        if self.first_timestamp is not None and self.last_timestamp is not None:
            duration_ms = self.last_timestamp - self.first_timestamp
            duration = timedelta(milliseconds=duration_ms)
            print(f"Duration: {duration} ({duration_ms} ms)")

        print(f"\nFrames:")
        print(f"  TX (Controller→Device): {self.tx_count}")
        print(f"  RX (Device→Controller): {self.rx_count}")
        print(f"  Total: {self.tx_count + self.rx_count}")

        print(f"\nBytes:")
        print(f"  TX: {self.total_bytes_tx}")
        print(f"  RX: {self.total_bytes_rx}")
        print(f"  Total: {self.total_bytes_tx + self.total_bytes_rx}")

        if self.tx_count + self.rx_count > 0:
            avg_frame_size = (self.total_bytes_tx + self.total_bytes_rx) / (
                self.tx_count + self.rx_count
            )
            print(f"  Average frame size: {avg_frame_size:.1f} bytes")

        print(f"\nDevices seen: {len(self.devices_seen)}")
        if self.devices_seen:
            print(f"  Addresses: {sorted(self.devices_seen)}")

        print(f"\nCommands seen: {len(self.commands_seen)}")
        if self.commands_seen:
            for cmd, count in sorted(self.commands_seen.items(), key=lambda x: x[1], reverse=True)[:10]:
                print(f"  0x{cmd:02X}: {count} times")

        print(f"\nStatus codes seen: {len(self.status_seen)}")
        if self.status_seen:
            for status, count in sorted(self.status_seen.items(), key=lambda x: x[1], reverse=True)[:10]:
                print(f"  0x{status:02X}: {count} times")

        if self.first_timestamp is not None and self.last_timestamp is not None and duration_ms > 0:
            frame_rate = (self.tx_count + self.rx_count) / (duration_ms / 1000.0)
            print(f"\nFrame rate: {frame_rate:.2f} frames/second")
            byte_rate = (self.total_bytes_tx + self.total_bytes_rx) / (duration_ms / 1000.0)
            print(f"Byte rate: {byte_rate:.2f} bytes/second")

        print("=" * 60 + "\n")


def parse_frame_line(line):
    """
    Parse a single line from the capture CSV.

    Returns:
        (timestamp_ms, direction, payload_bytes) or None if invalid
    """
    line = line.strip()

    # Skip empty lines and statistics/header lines
    if not line or line.startswith("---") or line.startswith("=") or "Statistics" in line:
        return None

    try:
        parts = line.split(",")
        if len(parts) != 3:
            return None

        timestamp_ms = int(parts[0])
        direction = parts[1]
        payload_hex = parts[2]

        # Convert hex string to bytes
        payload = bytes.fromhex(payload_hex)

        return (timestamp_ms, direction, payload)

    except (ValueError, IndexError):
        return None


def print_frame(timestamp, direction, payload, verbose=False):
    """Print a single frame in human-readable format."""
    arrow = "→" if direction == "tx" else "←"
    dir_label = "TX" if direction == "tx" else "RX"

    # Basic frame info
    print(f"[{timestamp:>10} ms] {dir_label} {arrow} {len(payload):2} bytes: {payload.hex()}")

    if verbose and len(payload) >= 4:
        # Parse LDCN frame structure
        start = payload[0]
        addr = payload[1]
        cmd_status = payload[2]
        length = payload[3]

        print(f"    Start: 0x{start:02X}  Addr: {addr:3}  Cmd/Status: 0x{cmd_status:02X}  Len: {length}")

        if len(payload) > 4:
            data = payload[4:-1]  # Exclude checksum
            checksum = payload[-1]
            print(f"    Data: {data.hex()}")
            print(f"    Checksum: 0x{checksum:02X}")


def analyze_file(filename, verbose=False, max_frames=None):
    """Analyze a capture file."""
    stats = FrameStats()
    frame_count = 0

    print(f"Analyzing: {filename}")

    try:
        with open(filename, "r") as f:
            for line in f:
                result = parse_frame_line(line)
                if result:
                    timestamp, direction, payload = result
                    stats.add_frame(timestamp, direction, payload)

                    if verbose:
                        print_frame(timestamp, direction, payload, verbose=True)

                    frame_count += 1
                    if max_frames and frame_count >= max_frames:
                        break

    except FileNotFoundError:
        print(f"Error: File not found: {filename}")
        return
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    stats.print_summary()


def analyze_live(device, verbose=False):
    """Analyze live capture from serial device."""
    import serial

    print(f"Connecting to: {device}")
    print("Press Ctrl+C to stop and show statistics\n")

    stats = FrameStats()

    try:
        with serial.Serial(device, 115200, timeout=1) as ser:
            while True:
                try:
                    line = ser.readline().decode("ascii", errors="ignore")
                    result = parse_frame_line(line)

                    if result:
                        timestamp, direction, payload = result
                        stats.add_frame(timestamp, direction, payload)

                        if verbose:
                            print_frame(timestamp, direction, payload, verbose=True)
                        else:
                            # Simple progress indicator
                            if stats.tx_count + stats.rx_count % 10 == 0:
                                print(".", end="", flush=True)

                except KeyboardInterrupt:
                    print("\n\nStopping capture...")
                    break

    except serial.SerialException as e:
        print(f"Error: {e}")
        return
    except Exception as e:
        print(f"Unexpected error: {e}")
        return

    stats.print_summary()


def main():
    parser = argparse.ArgumentParser(
        description="Analyze LDCN capture files from snooper",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "input",
        help="Capture file (CSV) or serial device for live capture"
    )

    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Print detailed frame information"
    )

    parser.add_argument(
        "-l", "--live",
        action="store_true",
        help="Analyze live capture from serial device"
    )

    parser.add_argument(
        "-n", "--max-frames",
        type=int,
        help="Maximum number of frames to process"
    )

    args = parser.parse_args()

    if args.live:
        analyze_live(args.input, verbose=args.verbose)
    else:
        analyze_file(args.input, verbose=args.verbose, max_frames=args.max_frames)


if __name__ == "__main__":
    main()
