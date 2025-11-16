# LDCN Protocol

This document describes the Logosol Distributed Control Network (LDCN) serial communication protocol.

## Overview

LDCN is a master-slave serial protocol using RS-485 physical layer. The controller (master) sends commands to devices (slaves), and devices respond with status data.

### Key Characteristics

- **Physical Layer**: RS-485, Daisy-chained multidrop topology
- **Baud Rates**: 9600 to 1.25 Mbps (19200 default at power on/reset)
- **Addressing**: Dynamic (1-127) augmented with groups (128-255)

## Baud Rate Divisors (BRD)

| Baud Rate | BRD Value | Note
|-----------|-----------|-------------|
| 9600      | 0x81      | 
| 19200     | 0x3F      | Default after reset |
| 57600     | 0x14      | 
| 115200    | 0x0A      | 
| **125000**| **0x27**  | **Recommended** |
| 312500    | 0x0F      | 
| 625000    | 0x07      | 
| 1250000   | 0x03      | 

## Packet Structure

### Command Packet (Host → Device)

```text
┌────────┬─────────┬──────────┬──────────────────────┬──────────┐
│ Header │ Address │  Cmd/Len │ Data (0-16 bytes)    │ Checksum │
│ (0xAA) │ (1 byte)│ (1 byte) │                      │ (1 byte) │
└────────┴─────────┴──────────┴──────────────────────┴──────────┘
```

- **Header**: `0xAA`
- **Address**: Device address (1-127) or group address (128-255)
- **Cmd/Len**: `[Data Length (4 bits)][Command (4 bits)]`
- **Data**: 0-16 data bytes (command-specific)
- **Checksum**: 8-bit sum of Address + Cmd/Len + Data bytes

### Status Packet (Device → Host)

The status is a compliled set of device states, sent in response to a **Nop** or **Read Status** command.
```text
┌────────┬──────────────────────────┬──────────┐
│ Status │ Status Data              │ Checksum │
│(1 byte)│ (0-20 bytes, variable)   │ (1 byte) │
└────────┴──────────────────────────┴──────────┘
```

- **Status**: Status byte (see Status Byte section)
- **Status Data**: Device-type dependent with configurable detail.
- **Checksum**: 8-bit sum of all bytes before checksum

## Group Addressing

Each LDCN device can optionally be assigned to one group addresses (128-255) in addition to the mandatory individual address (1-127). By assigning the same group address to multiple devices, commands can be executed simultaneously without response collisions.

**Broadcast Group Address:**

| Address | Description |
|---------|-------------|
| 0xFF    | All devices (broadcast) - no group leader |

Group addressing scenarios:

- **Set Baud Rate** - Change network speed for all devices atomically
- **Start Motion** - Begin coordinated multi-axis moves at the same time
- **Stop Motion** - Emergency stop across all axes

### Group Leader

One device in a group can be designated as the **group leader**. Only the group's leader responds to group commands, elminate bus contention issues.

### Configuration

Set group addresses using the **Set Address** command (0x1):

```text
AA 00 21 [individual] [group] [checksum]
```

**Example - Motion group:**

```text
AA 00 21 01 F0 11  # Device 1: individual=1, group=0xF0
AA 00 21 02 F0 12  # Device 2: individual=2, group=0xF0
AA 00 21 03 F0 13  # Device 3: individual=3, group=0xF0
```

Now all three devices belong to group 0xF0.

### Synchronized Motion Example

**Workflow for coordinated 3-axis move:**

```python
# 1. Load trajectories on each axis individually (bit 7 = 0, don't start yet)
send_command(addr=1, cmd=0x8, data=trajectory_x)  # X axis
send_command(addr=2, cmd=0x8, data=trajectory_y)  # Y axis
send_command(addr=3, cmd=0x8, data=trajectory_z)  # Z axis

# 2. Start all axes simultaneously with group command
send_command(addr=0xF0, cmd=0x5, data=[0x00])  # Start motion on group 0xF0
# All three axes begin moving within ±25 microseconds
```

### Notes

- **Baud rate changes**: Use group address 0xFF with no group leader
- **Timing**: Devices execute group commands within ±25 microseconds of each other
- **Single group**: A device can belong to zero or one group
- ** Reassignment**: A device's group address can be modified

## Hardware Synchronization Mode

Some LDCN devices support **hardware synchronization** to eliminate timing errors from oscillator drift during coordinated multi-axis motion. Without hardware sync, two timing error sources exist:

1. **Start Time Variation**: ±25 microseconds (group command execution spread)
2. **Oscillator Drift**: ~10 ppm typical (each drive has independent oscillator)

Oscillator drift causes timing errors to accumulate during motion. After 10 seconds, drift can produce ±0.0001 second timing error between axes. Errors reset at the start of each new move.

### Configuration

Hardware sync mode synchronizes servo ticks across multiple drives via dedicated hardware sync lines. This eliminates oscillator drift, leaving only the ±25 μs start time variation regardless of move duration.

See [servo_commands.md](servo_commands.md) for the **Enable/Disable Hardware Synchronization Mode** command.

## Generic LDCN Commands

The following commands are part of the base LDCN protocol and supported by **all device types** (servo drives, I/O controllers, etc.).

Device-specific commands are documented separately:

- Servo Drive Commands: See `servo_commands.md`
- I/O Controller Commands: See 'io_commands.md`

### Set Address (0x1)

Sets individual and group addresses for a device.

**Data**:

- Byte 0: Individual address (1-127)
- Byte 1: Group address (128-255)

**Example**:

```text
AA 00 21 01 FF 21  # Set device to address 1, group 0xFF
```

**Notes**:

- Sent to address 0x00 (unaddressed)
- First Set Address after reset enables next device in chain
- Used for auto-addressing during initialization

### Define Status Mask (0x2)

Selects items to return in status packets.

**Data**:

- Bytes 0-n: Status bits (16-bit little-endian)
- Byte map is device type dependent
- Devices default to 0x00 - no items returned


**Example**:

```text
AA 06 22 FF FF 26  # Device 6, request all status items (0xFFFF)
```

### Read Status (0x3)

Returns status items specified by the status mask (one-time, non-permanent).

### Set Baud Rate (0xA)

Changes baud rate of all devices. **Group command only** (no status response).

**Data**:

- Byte 0: BRD value (see [Baud Rate Divisors](#baud-rate-divisors-brd) table)

**Example**:

```text
AA FF 1A 27 46  # Change all devices to 125kbps (BRD=0x27)
```

**Notes**:

- Must be sent to group address 0xFF
- Do not set a group leader when sending this command
- Master must close and reopen serial port at new baud rate

### Nop - No Operation (0xE)

Returns current status items according to status mask, without performing any action.

**Data**: None

**Example**:

```text
AA 01 0E 0F  # NOP to device 1
```

**Notes**:

- Used for polling status
- checks for device responsiveness

### Hard Reset (0xF)

Resets controller to power-up state.

**Data**: None

**Example**:

```text
AA FF 0F 0E  # Reset all devices
```

**Notes**:

- No status returned
- Device returns to address 0x00, baud 19200
- **Special behavior at address 0xFF**: Resets device regardless of its configured group address
- Can reset entire network or contiguous sub-chain (if at default baud)
- Wait 2 seconds after reset before establishing communications

## Status Byte Interpretation

The meaning of status byte bits and auxiliary status data is device-type specific.

**Common Status Bits** (most devices):

| Bit | Name | Description |
|-----|------|-------------|
| 1   | cksum_error | Checksum error in received packet |


## Initialization Sequence

Typical network initialization:

1. **Hard Reset** (at any valid baud rate)

   ```text
   AA FF 0F 0E
   ```

   Wait 2 seconds.

2. **Set Addresses** (at 19200 baud)

   ```text
   AA 00 21 01 FF 21  # Device 1
   AA 00 21 02 FF 22  # Device 2
   AA 00 21 03 FF 23  # Device 3
   AA 00 21 04 FF 24  # Device 4
   AA 00 21 05 FF 25  # Device 5
   AA 00 21 06 FF 26  # Device 6 (I/O controller)
   ```

   Wait 300ms between each. Continue until no response is received from address 0x00.

3. **Verify Communication**

   ```text
   AA 01 0E 0F  # NOP to each device
   AA 02 0E 10
   ...

4. **Change Baud Rate**

   ```text
   AA FF 1A 27 46  # Change to 125kbps
   ```

   Close serial port, wait 500ms, reopen at 125kbps

5. **Commmunicate**

## Timing Requirements

- **Command Spacing**: Minimum 10ms between commands
- **Reset Wait**: 2s after Hard Reset
- **Address Wait**: 300ms after Set Address
- **Baud Change Wait**: 500ms before/after reopening serial port
- **Status Read**: Poll

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

## Example Communication Session

```text
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

1. **Response format**: For 2310g2 with full status, response[0]=status byte, response[1]=diagnostic code
2. **Group commands**: Set Baud Rate must be sent to group, never individual address
3. **Baud rate change**: Must physically close/reopen serial port at new baud rate
4. **Hard reset**: No response expected, wait full 2 seconds
5. **Address 0x00**: Only used during initial addressing, devices don't stay at address 0
6. **Checksum**: Calculated from address byte onwards, excludes header

## References

- Logosol LS-231SE Datasheet (Doc # 712231004)
- Logosol CNC-SK-2310g2 Manual (Doc # 710231005)
- RS-485 Application Note AN-960 (Texas Instruments)
