# SK-2310g2 Status Reporting - LS-773 Protocol

**Device:** SK-2310g2 Supervisory I/O Controller (LS-773 based)
**Author:** NickyDoes
**Source:** CNC-SK-2310g2 Manual, Doc # 710231005 / Rev. D, 03/05/2020
**Date:** 2025-11-13

---

## Overview

The LS-773 controller provides configurable status reporting. Use the Define Status command to specify which data items to include in status responses.

---

## Define Status Command

**Command:** `0x02` (CMD_DEFINE_STATUS)
**Data bytes:** 1 byte (status items bitmap)
**Default:** `0x00` (no items)

### Status Items Bitmap

Configure which data to include in Read Status responses:

| Bit | Data Item | Size | Description |
|-----|-----------|------|-------------|
| **0** | Input Bytes | 2 bytes | Digital input Byte 0 and Byte 1 |
| **1** | Analog Input 0 | 1 byte | Analog input channel 0 (0-255) |
| **2** | Analog Input 1 | 1 byte | Analog input channel 1 (0-255) |
| **3** | Analog Input 2 | 1 byte | Analog input channel 2 (0-255) |
| **4** | Counter/Timer | 4 bytes | Counter/timer value (LSB first) |
| **5** | Device ID/Version | 2 bytes | Device ID=2, Version=50 (LS-773) |
| **6** | Synch Input Bits | 2 bytes | Input bits captured with Synch Input command |
| **7** | Synch Counter/Timer | 4 bytes | Counter/timer captured with Synch Input (LSB first) |

---

## Status Response Format

Status response size depends on which bits are set in Define Status.

### Example Configurations

**Inputs only:**
```python
send_command(CMD_DEFINE_STATUS, [0x01])  # Bit 0 only
# Response: 2 bytes (input byte 0, input byte 1)
```

**Inputs + All Analog:**
```python
send_command(CMD_DEFINE_STATUS, [0x0F])  # Bits 0-3
# Response: 5 bytes (input byte 0, input byte 1, ain0, ain1, ain2)
```

**Inputs + Counter/Timer:**
```python
send_command(CMD_DEFINE_STATUS, [0x11])  # Bits 0 and 4
# Response: 6 bytes (input byte 0, input byte 1, counter[0-3])
```

**Everything:**
```python
send_command(CMD_DEFINE_STATUS, [0xFF])  # All bits
# Response: 16 bytes total
```

---

## Counter/Timer

When bit 4 or 7 is set, counter/timer values are included as 4 bytes (32-bit), least significant byte first.

**Counter/Timer Modes:**
- Timer mode: Counts time intervals
- Counter mode: Counts external events on configured input
- Synch mode (bit 7): Captures counter/timer value with Synch Input command

See manual for counter/timer configuration commands.

---

## Device ID and Version

When bit 5 is set, response includes:
- Byte 0: Device ID = `0x02` (LS-773)
- Byte 1: Version = `0x32` (50 decimal)

---

## Synch Input Command

The Synch Input command captures current input states and counter/timer value atomically. Retrieved using bits 6 and 7 in Define Status.

Use for time-critical applications requiring synchronized input sampling.

---

## Implementation Notes

### Efficient Status Configuration

For typical I/O polling, use bit 0 only (inputs):
```python
# Initialize once
send_command(CMD_DEFINE_STATUS, [0x01])

# Poll status repeatedly
while True:
    response = send_command(CMD_READ_STATUS, [0xFF, 0xFF])
    inputs = (response[2] << 8) | response[1]
```

### Reading Analog Inputs

Include analog bits when needed:
```python
# Configure: inputs + all analog
send_command(CMD_DEFINE_STATUS, [0x0F])

# Read status
response = send_command(CMD_READ_STATUS, [0xFF, 0xFF])
byte0 = response[1]           # Digital input byte 0
byte1 = response[2]           # Digital input byte 1
ain0 = response[3]            # Analog input 0 (0-255)
ain1 = response[4]            # Analog input 1 (0-255)
ain2 = response[5]            # Analog input 2 (0-255)
```

### Performance Considerations

- Minimize status packet size for faster polling
- Only include data items you actually use
- Typical configuration: `0x01` (inputs only) for fast digital I/O monitoring
- Add analog bits (`0x0F`) when analog feedback needed

---

## Related Documentation

- [sk-2310g2_io_mapping.md](sk-2310g2_io_mapping.md) - Complete I/O pin assignments
- [SK-2310g2_supervisor.md](SK-2310g2_supervisor.md) - Safety system and diagnostic codes
- CNC-SK-2310g2 Manual - Hardware reference

---

## Hardware Verification Status

🔴 **UNVERIFIED** - Command formats documented from manual, not yet tested on hardware.
