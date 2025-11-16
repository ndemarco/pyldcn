# LDCN Protocol Documentation

This document describes the Logosol Distributed Control Network (LDCN) serial communication protocol.

## Overview

LDCN is a master-slave serial protocol using RS-485 physical layer. The PC (master) sends commands to devices (slaves), and devices respond with status data.

### Key Characteristics

- **Physical Layer**: RS-485, Daisy-chained multidrop topology
- **Baud Rates**: 9600 to 1.25 Mbps (19200 default at power on/reset)
- **Addressing**: Dynamic (1-127) with group addressing (128-255)

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

## Packet Structure

### Command Packet (Host → Device)

```text
┌────────┬─────────┬──────────┬──────────────────────┬──────────┐
│ Header │ Address │  Cmd/Len │ Data (0-16 bytes)    │ Checksum │
│ (0xAA) │ (1 byte)│ (1 byte) │                      │ (1 byte) │
└────────┴─────────┴──────────┴──────────────────────┴──────────┘
```text

- **Header**: `0xAA`
- **Address**: Device address (1-127) or group address (128-255)
- **Cmd/Len**: `[Data Length (4 bits)][Command (4 bits)]`
- **Data**: 0-16 data bytes (command-specific)
- **Checksum**: 8-bit sum of Address + Cmd/Len + Data bytes

### Response Packet (Device → Host)

```text
┌────────┬──────────────────────────┬──────────┐
│ Status │ Additional Status Data   │ Checksum │
│(1 byte)│ (0-20 bytes, variable)   │ (1 byte) │
└────────┴──────────────────────────┴──────────┘
```text

- **Status**: Status byte (see Status Byte section)
- **Additional Data**: Configurable via *Define Status* command
- **Checksum**: 8-bit sum of all bytes before checksum

## Group Addressing

In addition to individual addresses (1-127), each LDCN device has a **group address** (128-255). Multiple devices can share the same group address, allowing simultaneous command execution without response collisions.

**Broadcast Group Address:**

| Address | Description |
|---------|-------------|
| 0xFF    | All devices (broadcast) - no group leader |

Group addressing scenarios:

- **Set Baud Rate** - Change network speed for all devices atomically
- **Load Trajectory** - Prepare multiple axes, then trigger with group start
- **Start Motion** - Begin coordinated multi-axis moves at the same time
- **Stop Motion** - Emergency stop across all axes

### Group Leader

One device in a group can be designated as the **group leader**. Only the group's leader responds to group commands. All other group members remain silent to elminate bus contention.

**How It Works**

1. Host sends command to group address (e.g., 0xFF)
2. All devices with that group address execute the command
3. If a group leader is configured, it responds. If no leader is configured, all devices remain silent.

### Configuration

Set group addresses using the **Set Address** command (0x1):

```text
AA 00 21 [individual] [group] [checksum]
```text

**Example - Creating a motion group:**

```text
AA 00 21 01 F0 11  # Device 1: individual=1, group=0xF0
AA 00 21 02 F0 12  # Device 2: individual=2, group=0xF0
AA 00 21 03 F0 13  # Device 3: individual=3, group=0xF0
```text

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
```text

### Important Notes

- **No response collision**: Group commands never generate responses (except from group leader)
- **Baud rate changes**: Always use group address 0xFF and do NOT set a group leader
- **Timing**: Devices execute group commands within ±25 microseconds of each other
- **Each device has one group**: A device can only belong to one group at a time

## Hardware Synchronization Mode

LS-231SE servo drives support **hardware synchronization** to eliminate timing errors from oscillator drift during coordinated multi-axis motion.

### Timing Error Sources

Without hardware sync, two timing error sources exist:

1. **Start Time Variation**: ±25 microseconds (group command execution spread)
2. **Oscillator Drift**: ~10 ppm typical (each drive has independent oscillator)

Oscillator drift causes timing errors to accumulate during motion. After 10 seconds, drift can produce ±0.0001 second timing error between axes. Errors reset at the start of each new move.

### Configuration

Hardware sync mode synchronizes servo ticks across multiple drives via dedicated hardware sync lines. This eliminates oscillator drift, leaving only the ±25 μs start time variation regardless of move duration.

See [servo_commands.md](servo_commands.md) for the **Enable/Disable Hardware Synchronization Mode** command.

## Generic LDCN Commands

The following commands are part of the base LDCN protocol and supported by **all device types** (servo drives, I/O controllers, etc.).

Device-specific commands (e.g., Load Trajectory, Load Gains for servos) are documented separately:

- Servo Drive Commands: See `servo_commands.md`
- I/O Controller Commands: See 'io_commands.md`

### 0x1 - Set Address

Sets individual and group addresses for a device.

**Data**:

- Byte 0: Individual address (1-127)
- Byte 1: Group address (128-255)

**Example**:

```text
AA 00 21 01 FF 21  # Set device to address 1, group 0xFF
```text

**Notes**:

- Sent to address 0x00 (unaddressed)
- First Set Address after reset enables next device in chain
- Used for auto-addressing during initialization

### 0x2 - Define Status

Selects data to return in status packets.

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

```text
AA 06 22 FF FF 26  # Device 6, request all status data (0xFFFF)
```text

### 0x3 - Read Status

Like Define Status, but only affects the immediate response (non-permanent).

### 0xA - Set Baud Rate

Changes baud rate of all devices. **Group command only** (no status response).

**Data**:

- Byte 0: BRD value (see Baud Rate Divisors table)

**Example**:

```text
AA FF 1A 27 46  # Change all devices to 125kbps (BRD=0x27)
```text

**Notes**:

- Must be sent to group address 0xFF
- Do not set a group leader when sending this command
- Master must close and reopen serial port at new baud rate

### 0xE - No Operation (NOP)

Returns current status data without performing any action.

**Data**: None

**Example**:

```text
AA 01 0E 0F  # NOP to device 1
```text

**Notes**:

- Used for polling status
- checks for device responsiveness

### 0xF - Hard Reset

Resets controller to power-up state.

**Data**: None

**Example**:

```text
AA FF 0F 0E  # Reset all devices
```text

**Notes**:

- No status returned
- Device returns to address 0x00, baud 19200
- **Special behavior at address 0xFF**: Resets device regardless of its configured group address
- Can reset entire network or contiguous sub-chain (if at default baud)
- Wait 2 seconds after reset before establishing communications

## Status Byte Interpretation

The meaning of status byte bits and auxiliary status data is **device-specific**:

- **Servo Drives (LS-231SE)**: See `SERVO_COMMANDS.md` for status bit definitions
- **I/O Controller (SK-2310g2)**: See `docs/logosol/LS-2310g2-Supervisor-IO-Controller.pdf`

**Common Status Bits** (most devices):

| Bit | Name | Description |
|-----|------|-------------|
| 1   | cksum_error | Checksum error in received packet |

**Note**: The additional status data returned depends on the *Define Status* configuration and is device-specific.

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

   Wait 300ms between each

3. **Verify Communication**

   ```text
   AA 01 0E 0F  # NOP to each device
   AA 02 0E 10
   ...
   ```

4. **Change Baud Rate**

   ```text
   AA FF 1A 27 46  # Change to 125kbps
   ```

   Close serial port, wait 500ms, reopen at 125kbps

5. **Commmunicate**

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

```text
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
```text

- Use shielded twisted pair cable
- Terminate each end. LDCN devices have selectable termination. See datasheet.
- Maximum cable length: ~1000 ft at 125kbps
- Connect all GND for reference

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
```text

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
