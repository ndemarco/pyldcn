# LS-231SE Servo Drive Status Reporting - Quick Reference

**Author:** NickyDoes
**Last Updated:** 2025-11-20
**Source:** LS-231SE Multifunctional Servo Drive Datasheet (Doc # 712231004 / Rev. A, 05/05/2011)

---

## Overview

This document provides a quick reference for LS-231SE status reporting. For complete diagnostic information, detailed status bit definitions, and troubleshooting procedures, see [LS-231SE/LS-231SE_status](LS-231SE/LS-231SE_status).

The LS-231SE uses a flexible status reporting system where the host selects which data items to include in status responses, minimizing communication overhead.

---

## Status Response Mechanism

Status responses are returned for:
- `NOP` command (0x0E) - Returns status with previously defined items
- `Read Status` command (0x03) - Returns status with specified items (one-time)
- Most other commands - Return status according to defined items

### Configuration Methods

| Method | Command | Persistence | Use Case |
|--------|---------|-------------|----------|
| **Define Status** | 0x02 | Persistent (all future responses) | Set reporting mode once at initialization |
| **Read Status** | 0x03 | One-time only | Query specific data without changing config |

**Default State:** After power-up or hard reset: `0x0000` (only status byte + checksum)

---

## Status Response Structure

Every status response consists of three parts:

```
[Status Byte] [Status Items...] [Checksum]
     ↓              ↓                ↓
  See below    Configurable      8-bit sum
```

### 1. Status Byte (Always Returned)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | move_done | 0=moving, 1=stopped or servo disabled |
| 1 | cksum_error | Checksum error detected (sticky) |
| 2 | current_limit | Current limiting occurred (sticky) |
| 3 | power_on | Amplifier power enabled |
| 4 | pos_error | Position error exceeded limit (sticky) |
| 5 | home_source | Home switch state (or diagnostic) |
| 6 | limit2 | Forward limit switch (or diagnostic) |
| 7 | home_in_progress | Homing active |

**Sticky bits** (1, 2, 4) must be cleared with Clear Bits command (0x0B).

For complete status byte interpretation including diagnostic conditions, see [LS-231SE/LS-231SE_status](LS-231SE/LS-231SE_status#status-byte).

### 2. Status Items (Configurable)

Configure which items to return using a 16-bit bitmap:

| Bit | Data Item | Bytes | Description |
|-----|-----------|-------|-------------|
| 0 | Position | 4 | Encoder position (int32, LSB first) |
| 1 | A/D Value | 1 | Analog input (0-255) |
| 2 | Velocity | 2 | Velocity (int16, counts/tick) |
| 3 | Auxiliary Status | 1 | Extended status flags |
| 4 | Home Position | 4 | Captured home (int32) |
| 5 | Device ID/Version | 2 | ID (0x00) + Version (20-29) |
| 6 | Position Error | 2 | Following error (int16) |
| 7 | Path Count | 1 | Path buffer depth (0-255) |
| 8 | Digital Inputs | 2 | I/O input states |
| 9 | Analog Inputs | 2 | Analog values |
| 12 | Watchdog Status | 2 | Watchdog timer |
| 13 | Motor Position | 6 | Motor pos + error |

### 3. Auxiliary Status Byte (Bit 3)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | index | Encoder index input |
| 1 | pos_wrap | Position counter wrapped (sticky) |
| 2 | servo_on | Position servo enabled |
| 3 | accel_done | Acceleration phase complete |
| 4 | slew_done | Constant velocity phase complete |
| 5 | servo_overrun | Calculation exceeded tick time (sticky) |
| 6 | path_mode | Path buffer executing |

For complete auxiliary status interpretation, see [LS-231SE/LS-231SE_status](LS-231SE/LS-231SE_status#auxiliary-status-byte).

---

## Quick Start Examples

### Define Persistent Status Configuration

```python
# Configure to return position, velocity, and auxiliary status
status_bits = 0x0001 | 0x0004 | 0x0008  # Bits 0, 2, 3
send_command(addr, 0x02, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

# All future NOP and command responses will include these items
```

### One-Time Status Read

```python
# Read position and velocity without changing persistent config
status_bits = 0x0005  # Bits 0 and 2
response = send_command(addr, 0x03, [status_bits & 0xFF, 0x00])

# Parse response
status_byte = response[0]
position = struct.unpack('<i', bytes(response[1:5]))[0]
velocity = struct.unpack('<h', bytes(response[5:7]))[0]
checksum = response[7]
```

### Using pyldcn Library

```python
from pyldcn import LDCNNetwork

with LDCNNetwork("/dev/ttyUSB0") as network:
    network.initialize()
    servo = network.find_device_by_type("LS-231SE")

    # Configure status
    servo.configure_status(0x0001 | 0x0004 | 0x0008)

    # Read status
    status = servo.read_status()
    print(f"Position: {status['position']}, Velocity: {status['velocity']}")
```

---

## Commands

### Define Status Command (0x02)

**Command:** `0x02` (CMD_DEFINE_STATUS)
**Data bytes:** 1 or 2 bytes (status items bitmap)
**Returns:** Status packet with defined items

Sets persistent status configuration for all future responses.

**Example:**
```python
# Position + velocity + auxiliary status
status_mask = 0x000D  # Bits 0, 2, 3
send_command(addr, 0x02, [status_mask & 0xFF, (status_mask >> 8) & 0xFF])
```

For complete command details, see [LS-231SE/LS-231SE_commands](LS-231SE/LS-231SE_commands#define-status).

---

### Read Status Command (0x03)

**Command:** `0x03` (CMD_READ_STATUS)
**Data bytes:** 1 or 2 bytes (status items bitmap)
**Returns:** Status packet (one-time only)

Non-persistent version of Define Status.

For complete command details, see [LS-231SE/LS-231SE_commands](LS-231SE/LS-231SE_commands#read-status).

---

## Related Documentation

- **[LS-231SE/LS-231SE_status](LS-231SE/LS-231SE_status)** - Complete status byte definitions, diagnostic conditions, LED indicators, fault troubleshooting
- **[LS-231SE/LS-231SE_commands](LS-231SE/LS-231SE_commands)** - All servo drive commands including status commands
- **[LS-231SE/LS-231SE_IO](LS-231SE/LS-231SE_IO)** - Digital I/O mapping and HomeSEL configuration
- **[ldcn_protocol](ldcn_protocol)** - Generic LDCN network protocol
- **pyldcn implementation:** `pyldcn/devices/servo_status.py` - Status parsing implementation

---

## Hardware Specifications

- **Device ID:** 0 (Motor Controller)
- **Firmware Version:** 20-29 (decimal, version dependent)
- **Encoder:** Quadrature with index
- **Servo Rate:** Configurable, up to 19.531 kHz
- **Position Resolution:** 32-bit signed counter
- **Communication:** RS-485, LDCN protocol (9600-230400 baud)

---

## References

- Logosol LS-231SE Multifunctional Servo Drive Datasheet (Doc # 712231004 / Rev. A, 05/05/2011)
- pyldcn implementation: `pyldcn/devices/servo.py` (LS231SE class)
