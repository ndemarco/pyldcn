# Logosol LS-231SE Servo Drive Status Reporting

**Author:** NickyDoes
**Source:** LS-231SE Multifunctional Servo Drive Datasheet (Doc # 712231004 / Rev. A, 05/05/2011)
**Date:** 2025-11-13

---

## General Introduction

This document describes status reporting for the LS-231SE servo drive, a multifunctional brushless DC servo motor controller. Status reporting is the primary mechanism for reading servo state, position, velocity, and diagnostic information from the drive.

The LS-231SE uses a flexible status reporting system where the host selects which data items to include in status responses. This allows applications to minimize communication overhead by requesting only the data they need.

Status responses are returned for:

- `NOP` command (0x0) - Returns status with previously defined items
- `Read Status` command (0x3) - Returns status with specified items (one-time)
- Most other commands - Return status according to defined items

---

## LS-231SE Hardware Features

- **Device ID:** 0 (Motor Controller)
- **Version:** 20-29 (decimal, firmware dependent)
- **Encoder Interface:** Quadrature encoder with index
- **Control Mode:** Position servo with trapezoidal profiling and path point trajectories
- **Servo Rate:** Configurable (19.531 kHz max)
- **Position Resolution:** 32-bit signed position counter
- **Digital I/O:** Configurable inputs/outputs (refer to Diagnostic and I/O section in datasheet)
- **Analog Input:** 8-bit A/D converter (0-255)
- **Communication:** RS-485, LDCN protocol (9600-230400 baud)

---

## Status Reporting

The LS-231SE servo drive conveys position, velocity, servo state, and diagnostic information via status reporting. The drive reports status in response to `Read Status` and `NOP` commands, returning a configurable set of status data items.

### Configuration Methods

**Persistent Configuration** - Use `Define Status` (0x2) to configure which status items will be returned with all subsequent `NOP` and command responses.

**One-Time Read** - Use `Read Status` (0x3) with desired status items to receive a status response one time without changing the persistent configuration.

**Default State** - After power-up or hard reset, the default status configuration is `0x0000` (no additional data items, only status byte and checksum).

---

## Status Response Structure

Every status response packet consists of three parts:

### 1. Status Byte

The first byte of every response packet contains drive state and error flags:

| Bit | Name | Description |
|-----|------|-------------|
| **0** | move_done | Clear when in the middle of a trapezoidal profile move or in velocity mode when accelerating from one velocity to the next. Set otherwise, including while position servo is disabled |
| **1** | cksum_error | Set if there was a checksum error in the just received command packet |
| **2** | current_limit | Set if current limiting has occurred. **Sticky bit** - must be cleared by user with Clear Bits command (0xE) |
| **3** | power_on/diag | Amplifier power enabled or diagnostic bit (refer to LS-231SE Diagnostic and I/O section) |
| **4** | pos_error | Set if position error has exceeded the position error limit. Also set whenever position servo is disabled (power_on=0). **Sticky bit** - must be cleared with Clear Bits command |
| **5** | home_source/diag | Home switch input state or diagnostic bit (refer to LS-231SE Diagnostic and I/O section) |
| **6** | limit2/diag | Forward limit switch or diagnostic bit (refer to LS-231SE Diagnostic and I/O section) |
| **7** | home_in_progress | Set while searching for a home position. Reset to zero once home position has been captured |

**Sticky Bits**: Bits 2 (current_limit) and 4 (pos_error) remain set once triggered and must be explicitly cleared using the Clear Bits command (0xE).

### 2. Status Items

The data to include in status responses is encoded into a 16-bit bitmap. Set a bit to include the item, clear a bit to omit the item. Status items are always returned in the order listed below.

| Bit | Data Item | Size | Description |
|-----|-----------|------|-------------|
| **0** | Position | 4 bytes | Current position in encoder counts (32-bit signed, LSB first) |
| **1** | A/D Value | 1 byte | Analog-to-digital converter value (0-255) |
| **2** | Velocity | 2 bytes | Current velocity in counts per servo tick (16-bit signed, LSB first, no fractional component) |
| **3** | Auxiliary Status | 1 byte | Auxiliary status byte (servo on, overrun flags, path mode) |
| **4** | Home Position | 4 bytes | Captured home position in encoder counts (32-bit signed, LSB first) |
| **5** | Device ID/Version | 2 bytes | Device ID (0x00 for LS-231SE), Version (20-29 decimal) |
| **6** | Position Error | 2 bytes | Current position following error in encoder counts (16-bit signed, LSB first) |
| **7** | Path Count | 1 byte | Number of points in the path buffer (motion queue depth, 0-255) |
| **8** | Digital Inputs | 2 bytes | Digital input states (refer to Diagnostic and I/O section) |
| **9** | Analog Inputs | 2 bytes | Analog input values (refer to Diagnostic and I/O section) |
| **10-11** | Reserved | - | Reserved. Clear to 0 |
| **12** | Watchdog Status | 2 bytes | Watchdog timer status |
| **13** | Motor Position | 6 bytes | Motor position and position error (6 bytes) |
| **14-15** | Reserved | - | Reserved. Clear to 0 |

**Example**: To request position (bit 0) and velocity (bit 2), set status_bits = 0x0005 (bits 0 and 2).

### 3. Auxiliary Status Byte

When bit 3 is set in the status configuration, the auxiliary status byte is returned:

| Bit | Name | Description |
|-----|------|-------------|
| **0** | index/diag | Complement of the value of the index input or diagnostic bit (refer to LS-231SE Diagnostic and I/O section) |
| **1** | pos_wrap | Set if the 32-bit position counter wraps around. **Sticky bit** - must be cleared with Clear Bits command |
| **2** | servo_on | Set if the position servo is enabled, clear otherwise |
| **3** | accel_done | Set when the initial acceleration phase of a trapezoidal profile move is completed. Cleared when the next move is started |
| **4** | slew_done | Set when the slew portion of a trapezoidal profile move is complete. Cleared when the next move is started |
| **5** | servo_overrun | At the highest baud rate and servo rate, certain combinations of calculations may cause the servo, profiling, and command processing to take longer than one servo tick (e.g., 51.2 µs). **Sticky bit** - must be cleared with Clear Bits command. This is typically not serious, only periodically introducing a small fraction of a millisecond delay to the servo tick time |
| **6** | path_mode | Set when the drive is currently executing a path. Cleared when buffer is emptied or Stop Motor or Load Trajectory command is sent |
| **7** | Reserved | - |

**Sticky Bits**: Bits 1 (pos_wrap) and 5 (servo_overrun) remain set once triggered and must be explicitly cleared.

### 4. Checksum Byte

The checksum byte is the 8-bit sum of the status byte plus all data bytes.

**Verification**:

```python
# Calculate expected checksum
expected = sum(response[:-1]) & 0xFF
received = response[-1]

if expected != received:
    print(f"Checksum error: expected 0x{expected:02X}, got 0x{received:02X}")
```

---

## Command Reference

### Command Summary Table

| Command | Code | Data Bytes | Description |
|---------|------|------------|-------------|
| [Define Status](#define-status-command) | 0x2 | 1 or 2 | Defines which data should be sent in every status packet |
| [Read Status](#read-status-command) | 0x3 | 1 or 2 | Causes particular status data to be returned just once |

For LDCN network commands (Set Address, NOP, Hard Reset, etc.), see [ldcn_protocol.md](ldcn_protocol.md).
For servo motion commands, see [servo_commands.md](servo_commands.md).

---

### Define Status Command

**Command:** `0x02` (CMD_DEFINE_STATUS)
**Data bytes:** 1 or 2 bytes (status items bitmap)
**Command byte:** `0x12` (1 data byte) or `0x22` (2 data bytes)
**Default:** `0x0000` (no items)
**Returns:** Yes - Status packet with defined status items

Defines what additional data will be sent in the status packet along with the status byte. Setting bits in the command's data bytes will cause the corresponding additional data bytes to be sent after the status byte in all future status responses.

The status data will always be sent in the order listed in the Status Items table. For example, if bits 0 and 3 are set, the status packet will consist of the status byte followed by four bytes of position data, followed by the auxiliary status byte, followed by the checksum.

**Data Byte Encoding**:

- If status_bits ≤ 0xFF (bits 0-7 only): Send 1 byte (command byte 0x12)
- If status_bits > 0xFF (bits 8-15 used): Send 2 bytes (command byte 0x22)

**Example**:

```python
# Define persistent status: position, velocity, aux status, position error
status_bits = 0x0001 | 0x0004 | 0x0008 | 0x0040  # 0x004D
send_command(addr, 0x02, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

# All subsequent NOP commands and responses will include these items
```

**Notes**:

- Configuration persists until power cycle or hard reset
- Hard Reset or power cycle returns to default (`0x0000`)
- To minimize communication overhead, request only needed data items

---

### Read Status Command

**Command:** `0x03` (CMD_READ_STATUS)
**Data bytes:** 1 or 2 bytes (status items bitmap)
**Command byte:** `0x13` (1 data byte) or `0x23` (2 data bytes)
**Returns:** Yes - Status packet with requested status items (one time only)

This is a non-permanent version of the Define Status command. The status packet returned in response to this command will incorporate the data bytes specified, but subsequent status packets will include only the data bytes previously specified with the Define Status command.

**Example**:

```python
# One-time read: position and velocity only
status_bits = 0x0001 | 0x0004  # 0x0005
response = send_command(addr, 0x03, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

# Parse response
status_byte = response[0]
position = struct.unpack('<i', bytes(response[1:5]))[0]  # 4 bytes, signed
velocity = struct.unpack('<h', bytes(response[5:7]))[0]  # 2 bytes, signed
checksum = response[7]
```

**Use Cases**:

- Reading status items without changing persistent configuration
- Debugging or diagnostics
- One-time queries for specific data

---

## Implementation Notes

### Efficient Status Configuration

For fast position polling, minimize status response size by requesting only position:

```python
# Define persistent status: position only
status_bits = 0x0001  # bit 0
send_command(addr, 0x02, [status_bits])

# Efficient polling loop
while True:
    response = send_command(addr, 0x00)  # NOP
    # Response is 6 bytes: [status, pos0, pos1, pos2, pos3, checksum]
    status_byte = response[0]
    position = struct.unpack('<i', bytes(response[1:5]))[0]

    # Check for faults
    if status_byte & 0x02:  # cksum_error
        print("Checksum error detected")
    if status_byte & 0x04:  # current_limit (sticky)
        print("Current limit exceeded - clearing fault")
        send_command(addr, 0x0E, [0x15])  # Clear Bits command
```

### Reading Multiple Status Items

Configure multiple items for comprehensive status monitoring:

```python
# Define comprehensive status
status_bits = (
    0x0001 |  # position
    0x0004 |  # velocity
    0x0008 |  # aux status
    0x0040    # position error
)  # = 0x004D

send_command(addr, 0x02, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

# Read status
response = send_command(addr, 0x00)  # NOP
# Response is 12 bytes: [status, pos(4), vel(2), aux(1), pos_err(2), checksum]

status_byte = response[0]
position = struct.unpack('<i', bytes(response[1:5]))[0]
velocity = struct.unpack('<h', bytes(response[5:7]))[0]
aux_status = response[7]
pos_error = struct.unpack('<h', bytes(response[8:10]))[0]
checksum = response[10]

# Decode auxiliary status
servo_on = bool(aux_status & 0x04)
servo_overrun = bool(aux_status & 0x20)
path_mode = bool(aux_status & 0x40)

# Check for sticky faults
if status_byte & 0x04:  # current_limit
    print("Current limit fault (sticky)")
if status_byte & 0x10:  # pos_error
    print(f"Position error fault: {pos_error} counts (sticky)")
if aux_status & 0x02:  # pos_wrap
    print("Position counter wrapped (sticky)")
```

### Clearing Sticky Bits

Certain status bits remain set until explicitly cleared:

```python
# Clear all sticky bits using Clear Bits command (0xE, sub-command 0x15)
# Clears: current_limit, pos_error, pos_wrap, servo_overrun
send_command(addr, 0x0E, [0x15])

# Verify faults cleared
response = send_command(addr, 0x00)  # NOP
status_byte = response[0]
if status_byte & 0x14 == 0:  # bits 2 and 4
    print("Sticky faults cleared successfully")
```

### Variable-Length Response Parsing

The pyldcn library includes a `_parse_status()` method to handle variable-length responses:

```python
def _parse_status(response: bytes, status_bits: int) -> dict:
    """Parse variable-length servo status response."""
    if len(response) < 2:
        return {}

    status_byte = response[0]
    data = response[1:-1]  # Everything except status and checksum

    result = {'status': status_byte, 'flags': decode_status_flags(status_byte)}
    idx = 0

    # Parse each item in order based on status_bits configuration
    if status_bits & 0x0001 and len(data) >= idx + 4:  # Position
        result['position'] = struct.unpack('<i', bytes(data[idx:idx+4]))[0]
        idx += 4

    if status_bits & 0x0002 and len(data) >= idx + 1:  # A/D value
        result['ad_value'] = data[idx]
        idx += 1

    if status_bits & 0x0004 and len(data) >= idx + 2:  # Velocity
        result['velocity'] = struct.unpack('<h', bytes(data[idx:idx+2]))[0]
        idx += 2

    if status_bits & 0x0008 and len(data) >= idx + 1:  # Aux status
        result['aux_status'] = data[idx]
        result['servo_on'] = bool(data[idx] & 0x04)
        result['servo_overrun'] = bool(data[idx] & 0x20)
        result['path_mode'] = bool(data[idx] & 0x40)
        idx += 1

    if status_bits & 0x0040 and len(data) >= idx + 2:  # Position error
        result['pos_error'] = struct.unpack('<h', bytes(data[idx:idx+2]))[0]
        idx += 2

    return result
```

---

## Examples

### Basic Servo Initialization

```python
def initialize_servo(addr):
    """Initialize LS-231SE servo drive with status reporting."""

    # Step 1: Hard reset (see ldcn_protocol.md)
    send_command(addr, 0x0F, [0x5A])

    # Step 2: Define status reporting (position, velocity, aux, pos_error)
    status_bits = 0x0001 | 0x0004 | 0x0008 | 0x0040  # 0x004D
    send_command(addr, 0x02, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

    # Step 3: Set gains (KP, KD, KI, IL, OL, CL, EL, SR, DB)
    # Minimal requirements: KP != 0, EL != 0, SR != 0
    send_command(addr, 0x06, [kp, kd, ki, il, ol, cl, el, sr, db])

    # Step 4: Initialize trajectory registers (Load Trajectory)
    # Minimal requirements: acceleration != 0, target position = 0
    send_command(addr, 0x04, [...])  # See servo_commands.md

    # Step 5: Close servo loop (Stop Motor: Pic_ae=1, Stop abruptly=1)
    send_command(addr, 0x01, [0x05])  # AMP_ENABLE | STOP_ABRUPT

    # Step 6: Read and verify status
    response = send_command(addr, 0x00)  # NOP
    return parse_servo_status(response, status_bits)
```

### Motion Monitoring

```python
def monitor_motion(addr):
    """Monitor servo motion until move completes."""

    # Define status: position, aux status
    status_bits = 0x0001 | 0x0008
    send_command(addr, 0x02, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

    move_complete = False
    while not move_complete:
        response = send_command(addr, 0x00)  # NOP
        status_byte = response[0]
        position = struct.unpack('<i', bytes(response[1:5]))[0]
        aux_status = response[5]

        # Check move_done flag (bit 0)
        move_done = bool(status_byte & 0x01)

        # Check accel_done and slew_done (aux status bits 3 and 4)
        accel_done = bool(aux_status & 0x08)
        slew_done = bool(aux_status & 0x10)

        # Check servo_on (aux status bit 2)
        servo_on = bool(aux_status & 0x04)

        print(f"Position: {position}, Move Done: {move_done}, "
              f"Accel Done: {accel_done}, Slew Done: {slew_done}, "
              f"Servo On: {servo_on}")

        if move_done and slew_done:
            move_complete = True

        time.sleep(0.01)  # 10ms polling

    print("Move complete!")
```

### Fault Detection and Recovery

```python
def check_and_clear_faults(addr):
    """Check for faults and clear sticky bits."""

    # Read comprehensive status
    status_bits = 0x0001 | 0x0008 | 0x0040  # position, aux, pos_error
    response = send_command(addr, 0x03, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

    status_byte = response[0]
    position = struct.unpack('<i', bytes(response[1:5]))[0]
    aux_status = response[5]
    pos_error = struct.unpack('<h', bytes(response[6:8]))[0]

    faults = []

    # Check status byte faults
    if status_byte & 0x02:
        faults.append("Checksum error")
    if status_byte & 0x04:
        faults.append(f"Current limit exceeded (sticky)")
    if status_byte & 0x10:
        faults.append(f"Position error: {pos_error} counts (sticky)")

    # Check auxiliary status faults
    if aux_status & 0x02:
        faults.append(f"Position counter wrapped (sticky)")
    if aux_status & 0x20:
        faults.append(f"Servo overrun (sticky)")

    if faults:
        print("Faults detected:")
        for fault in faults:
            print(f"  - {fault}")

        # Clear sticky bits
        print("Clearing sticky faults...")
        send_command(addr, 0x0E, [0x15])  # Clear Bits command

        # Verify cleared
        response = send_command(addr, 0x00)  # NOP
        if response[0] & 0x14 == 0:  # bits 2 and 4
            print("Sticky faults cleared successfully")
    else:
        print("No faults detected")

    return faults
```

---

## References

- Logosol LS-231SE Multifunctional Servo Drive Datasheet (Doc # 712231004 / Rev. A, 05/05/2011)
  - LS-231SE Device ID: 0 (Motor Controller)
  - Version: 20-29 (decimal, firmware dependent)
- [ldcn_protocol.md](ldcn_protocol.md) - Generic LDCN network protocol documentation
- [servo_commands.md](servo_commands.md) - Complete LS-231SE command reference
- pyldcn implementation: `pyldcn/devices/servo.py` (LS231SE class)
