"""
Command spacing experiment for Logosol LDCN devices.

Send back-to-back commands with decreasing inter-command delay to find
the shortest safe spacing for a given device/baud.

Usage example:
    python experiments/command_spacing_test.py \
        --port /dev/ttyUSB0 --address 1 --baud 125000 \
        --start-ms 10 --end-ms 0 --step-ms 1 --trials 20

Notes:
- Default command is NOP (0x0E). Change with --command if desired.
- Stops on the first failure and reports the delay that failed.
- Requires live hardware on the given address/baud.
"""

import argparse
import time
from typing import List

from pyldcn.protocol import LDCNProtocol, CMD_NOP, LDCNTimeoutError, LDCNChecksumError


def run_sweep(
    proto: LDCNProtocol,
    address: int,
    command: int,
    data: List[int],
    start_ms: int,
    end_ms: int,
    step_ms: int,
    trials: int,
) -> None:
    proto.open(args.baud)
    print(
        f"Starting spacing sweep: addr={address} baud={proto.baud_rate} "
        f"command=0x{command:02X} data_len={len(data)} "
        f"delays {start_ms}→{end_ms} step={step_ms} trials={trials}"
    )

    for delay_ms in range(start_ms, end_ms - 1, -step_ms):
        failures = 0
        for i in range(trials):
            try:
                resp = proto.send_command(address, command, data)
                if len(resp) < 2:
                    failures += 1
                    print(f"  Delay {delay_ms}ms trial {i+1}: short response ({len(resp)} bytes)")
                    break
            except (LDCNTimeoutError, LDCNChecksumError) as e:
                failures += 1
                print(f"  Delay {delay_ms}ms trial {i+1}: error {type(e).__name__} -> {e}")
                break
            except Exception as e:
                failures += 1
                print(f"  Delay {delay_ms}ms trial {i+1}: unexpected error -> {e}")
                break

            if delay_ms > 0:
                time.sleep(delay_ms / 1000.0)

        if failures == 0:
            print(f"✓ Delay {delay_ms}ms: {trials}/{trials} succeeded")
            continue

        print(f"✗ Delay {delay_ms}ms: stopped on failure after {i+1} trial(s)")
        break
    else:
        print("Completed sweep without failures")

    proto.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LDCN command spacing sweep")
    parser.add_argument("--port", required=True, help="Serial port path (e.g., /dev/ttyUSB0)")
    parser.add_argument("--address", type=int, default=1, help="Device address (default: 1)")
    parser.add_argument("--baud", type=int, default=125000, help="Baud rate (default: 125000)")
    parser.add_argument(
        "--command", type=lambda x: int(x, 0), default=CMD_NOP, help="Command byte (default: NOP 0x0E)"
    )
    parser.add_argument(
        "--data",
        type=lambda x: [int(b, 0) for b in x.split(",") if b],
        default=[],
        help="Comma-separated data bytes (default: none)",
    )
    parser.add_argument("--start-ms", type=int, default=10, help="Starting delay in ms (default: 10)")
    parser.add_argument("--end-ms", type=int, default=0, help="Ending delay in ms (default: 0)")
    parser.add_argument("--step-ms", type=int, default=1, help="Step size in ms (default: 1)")
    parser.add_argument("--trials", type=int, default=20, help="Trials per delay (default: 20)")

    args = parser.parse_args()

    if args.step_ms <= 0:
        parser.error("--step-ms must be > 0")
    if args.start_ms < args.end_ms:
        parser.error("--start-ms must be >= --end-ms")
    if args.address < 1 or args.address > 255:
        parser.error("--address must be 1-255")

    proto = LDCNProtocol(args.port, timeout=0.015)
    run_sweep(
        proto=proto,
        address=args.address,
        command=args.command,
        data=args.data,
        start_ms=args.start_ms,
        end_ms=args.end_ms,
        step_ms=args.step_ms,
        trials=args.trials,
    )
