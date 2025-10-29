# LDCN Protocol Documentation

This document describes the Logosol Distributed Control Network (LDCN) serial communication protocol.

## Overview

LDCN is a master-slave serial protocol using RS-485 physical layer. The PC (master) sends commands to devices (slaves), and devices respond with status data.

### Key Characteristics

- **Physical Layer**: RS-485 differential signaling
- **Topology**: Daisy-chained multidrop bus
- **Baud Rates**: 9600 to 1.25 Mbps
- **Addressing**: Dynamic (1-127) with group addressing (128-255)
- **Timing**: 51.2 µs servo tick (19.53 kHz control loop)

## Packet Structure

### Command Packet (Host → Device)

```
┌────────┬─────────┬──────────┬──────────────────────┬──────────┐
│ Header │ Address │  Cmd/Len │ Data (0-16 bytes)    │ Checksum │
│ (0xAA) │ (1 byte)│ (1 byte) │                      │ (1 byte) │
└────────┴─────────┴──────────┴──────────────────────┴──────────┘
```

- **Header**: Always 0xAA
- **Address**: Device address (1-127) or group address (128-255)
- **Cmd/Len**: `[Data Length (4 bits)][Command (4 bits)]`
- **Data**: 0-16 data bytes (command-specific)
- **Checksum**: 8-bit sum of Address + Cmd/Len + Data bytes

### Response Packet (Device → Host)

```
┌────────┬──────────────────────────┬──────────┐
│ Status │ Additional Status Data   │ Checksum │
│(1 byte)│ (0-20 bytes, variable)   │ (1 byte) │
└────────┴──────────────────────────┴──────────┘
```

- **Status**: Status byte (see Status Byte section)
- **Additional Data**: Configurable via Define Status command
- **Checksum**: 8-bit sum of all bytes before checksum

## Baud Rate Divisors (BRD)

| Baud Rate | BRD Value | Typical Use |
|-----------|-----------|-------------|
| 9600      | 0x81      | Debugging |
| 19200     | 0x3F      | Default after reset |
| 57600     | 0x14      | - |
| 115200    | 0x0A      | - |
| **125000**| **0x27**  | **Recommended** |
| 312500    | 0x0F      | High-speed |
| 625000    | 0x07      | High-speed |
| 1250000   | 0x03      | Maximum |

## Generic LDCN Commands

The following commands are part of the base LDCN protocol and supported by **all device types** (servo drives, I/O controllers, etc.).

Device-specific commands (e.g., Load Trajectory, Load Gains for servos) are documented separately:
- Servo Drive Commands: See `SERVO_COMMANDS.md`
- I/O Controller Commands: See hardware manual

### 0x1 - Set Address

Sets individual and group addresses for a device.

**Data**:
- Byte 0: Individual address (1-127)
- Byte 1: Group address (128-255)

**Example**:
```
AA 00 21 01 FF 21  # Set device to address 1, group 0xFF
```

**Notes**:
- Sent to address 0x00 (unaddressed)
- First Set Address after reset enables next device in chain
- Used for auto-addressing during initialization

### 0x2 - Define Status

Configures what additional data to return in status packets.

**Data**:
- Bytes 0-1: Status bits (16-bit little-endian)

**Status Bits**:
| Bit | Data Returned |
|-----|---------------|
| 0   | Position (4 bytes) |
| 1   | A/D value (1 byte) |
| 2   | Velocity (2 bytes) |
| 3   | Auxiliary status byte |
| 4   | Home position (4 bytes) |
| 5   | Device ID and version (2 bytes) |
| 6   | Position error (2 bytes) |
| 7   | Path buffer count (1 byte) |
| 8   | Digital inputs (2 bytes) |
| 9   | Analog inputs (2 bytes) |
| 12  | Watchdog status (2 bytes) |
| 13  | Motor position and error (6 bytes) |

**Example**:
```
AA 06 22 FF FF 26  # Device 6, request all status data (0xFFFF)
```

### 0x3 - Read Status

Like Define Status, but only affects the immediate response (non-permanent).

### 0xA - Set Baud Rate

Changes baud rate of all devices. **Group command only** (no status response).

**Data**:
- Byte 0: BRD value (see Baud Rate Divisors table)

**Example**:
```
AA FF 1A 27 46  # Change all devices to 125kbps (BRD=0x27)
```

**Notes**:
- Must be sent to group address 0xFF
- Do not set a group leader when sending this command
- Master must close and reopen serial port at new baud rate

### 0xE - No Operation (NOP)

Returns current status data without performing any action.

**Data**: None

**Example**:
```
AA 01 0E 0F  # NOP to device 1
```

**Notes**:
- Used for polling status
- Fastest way to check device responsiveness

### 0xF - Hard Reset

Resets controller to power-up state.

**Data**: None

**Example**:
```
AA FF 0F 0E  # Reset all devices (group command)
```

**Notes**:
- No status returned
- Device returns to address 0x00, baud 19200
- Typically sent to group address 0xFF to reset all devices
- Wait 2 seconds after reset before communicating

## Status Byte Interpretation

The meaning of status byte bits and auxiliary status data is **device-specific**:

- **Servo Drives (LS-231SE)**: See `SERVO_COMMANDS.md` for status bit definitions
- **I/O Controller (SK-2310g2)**: See `docs/logosol/LS-2310g2-Supervisor-IO-Controller.pdf`

**Common Status Bits** (most devices):
| Bit | Name | Description |
|-----|------|-------------|
| 1   | cksum_error | Checksum error in received packet |

**Note**: The additional status data returned depends on the Define Status configuration and is device-specific.

## Initialization Sequence

Typical network initialization:

1. **Hard Reset** (at any baud rate)
   ```
   AA FF 0F 0E
   ```
   Wait 2 seconds

2. **Set Addresses** (at 19200 baud)
   ```
   AA 00 21 01 FF 21  # Device 1
   AA 00 21 02 FF 22  # Device 2
   AA 00 21 03 FF 23  # Device 3
   AA 00 21 04 FF 24  # Device 4
   AA 00 21 05 FF 25  # Device 5
   AA 00 21 06 FF 26  # Device 6 (I/O controller)
   ```
   Wait 300ms between each

3. **Verify Communication**
   ```
   AA 01 0E 0F  # NOP to each device
   AA 02 0E 10
   ...
   ```

4. **Change Baud Rate**
   ```
   AA FF 1A 27 46  # Change to 125kbps
   ```
   Close serial port, wait 500ms, reopen at 125kbps

5. **Configure I/O Controller**
   ```
   AA 06 22 FF FF 26  # Define full status for device 6
   ```

## Timing Requirements

- **Command Spacing**: Minimum 10ms between commands
- **Reset Wait**: 2000ms after Hard Reset
- **Address Wait**: 300ms after Set Address
- **Baud Change Wait**: 500ms before/after reopening serial port
- **Status Read**: Poll at 10-20 Hz for power monitoring

## Error Handling

### Checksum Errors

If checksum doesn't match, device sets bit 1 in status byte. Resend command.

### No Response

- Verify device address is correct
- Check RS-485 wiring (A, B, GND)
- Verify baud rate matches
- Check termination resistors

### Communication Lost

If device stops responding:
1. Try Hard Reset at current baud rate
2. Try Hard Reset at 19200 baud
3. Power cycle hardware

## RS-485 Wiring

```
┌─────────┐          ┌─────────┐          ┌─────────┐
│ Master  │          │ Device 1│          │ Device N│
│  (PC)   │          │         │          │         │
├─────────┤          ├─────────┤          ├─────────┤
│ A   ────┼──────────┼─ A  A ──┼──────────┼─ A      │
│ B   ────┼──────────┼─ B  B ──┼──────────┼─ B      │
│ GND ────┼──────────┼─GND GND─┼──────────┼─GND     │
└─────────┘          └─────────┘          └─────────┘
     │                                          │
    [R]                                        [R]
    120Ω                                       120Ω
```

- Use shielded twisted pair cable
- Terminate both ends with 120Ω resistors
- Maximum cable length: ~1000 ft at 125kbps
- Connect all GND for reference

## Example Communication Session

```
# Power up - devices at 19200 baud
TX: AA FF 0F 0E              # Hard reset
RX: (none)
<wait 2 seconds>

# Address device 1
TX: AA 00 21 01 FF 21        # Set address 1, group 0xFF
RX: 31 31                    # Response from LS-231SE

# Check status
TX: AA 01 0E 0F              # NOP to device 1
RX: 59 59                    # Status byte 0x59

# Change to 125kbps
TX: AA FF 1A 27 46           # Set baud to 125kbps
<close serial port>
<wait 500ms>
<reopen at 125kbps>

# Verify at new baud rate
TX: AA 01 0E 0F              # NOP to device 1
RX: 31 31                    # Device responding

# Enable amplifier
TX: AA 01 17 05 1D           # Stop abruptly + amp enable
RX: (no response expected for stop command)
```

## Protocol Gotchas

1. **Status byte position**: For 2310g2 with full status, status byte is at index 1, not 0
2. **Group commands**: Set Baud Rate must be sent to group, never individual address
3. **Baud rate change**: Must physically close/reopen serial port at new baud rate
4. **Hard reset**: No response expected, wait full 2 seconds
5. **Address 0x00**: Only used during initial addressing, devices don't stay at address 0
6. **Checksum**: Calculated from address byte onwards, excludes header

## References

- Logosol LS-231SE Datasheet (Doc # 712231004)
- Logosol CNC-SK-2310g2 Manual (Doc # 710231005)
- RS-485 Application Note AN-960 (Texas Instruments)
