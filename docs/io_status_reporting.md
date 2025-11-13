# Logosol Network I/O Node commands

**Author:** NickyDoes
**Source:** LS-773 Network I/O Node Datasheet (Doc# 712773001, Rev. B) and CNC-SK-2310g2 Manual (Doc # 710231005 / Rev. D, 03/05/2020)
**Date:** 2025-11-13

---

## General Introduction

This document is based on two LDCN-compliant I/O devices, the LS-773 and the SK-2310g2. The '773 is a general purpose I/O device, whereas the '2310 is a supervisory controller with I/O characteristics, plus general I/O features. The LDCN protocol and commands are substantially similar between the two.

These are polled, not interrupt-driven or real-time. The counter/timer is designed for synchronized input capture - Hardware-latched position/time snapshots using the `Sync Input` command.


## Specific Models

### LS-773 Hardware Features
- 10 general purpose digital inputs (with configurable pull-up/pull-down)
- 6 open collector outputs (1A max each)
- 1 solid-state relay output (0.5A max, OUTPUT 0/POWER)
- 3 analog inputs (8-bit, 0-5V/0-10V/0-20V/0-30V selectable)
- 32-bit counter/timer with prescaler (5.0 MHz clock)
- 20 KHz PWM mode for OUTPUT 1 and OUTPUT 2
- Device ID: 2, Version: 50

#### LS-773 Input Bit Layout

Byte 0:
| Bit | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|-----|---|---|---|---|---|---|---|---|
| Input | IN7 | IN6 | IN5 | IN4 | IN3 | IN2 | IN1 | IN0 |

Byte 1:
| Bit | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|-----|---|---|---|---|---|---|---|---|
| Input | - | - | - | - | - | - | IN9 | IN8 |
| Flag | - | - | - | - | - | OUT_SH | - | - |

#### OUT_SH Flag (Byte 1, bit 1):
- OUT_SH = 1: One or more outputs are shorted to POWER(+)
- OUT_SH = 0: Normal operation

### SK-2310g2 Hardware Features
The SK-2310g2 is a supervisory controller with specialized hardware and I/O capabilities.

### I/O Hardware Differences

| Feature | LS-773 | SK-2310g2 |
|---------|--------|-----------|
| **Digital Inputs** | 10 inputs | 7 physical + 9 internal status |
| **Digital Outputs** | 7 outputs | 8 physical + 8 internal control |
| **Analog Inputs** | 3 inputs (0-5/10/20/30V) | 3 inputs (ADC-1: 0-10V, ADC-2/3: 0-5V) |
| **Analog Outputs** | None | 1 output (0-10V DAC for spindle control) |
| **Counter/Timer** | 32-bit (5.0 MHz) | Unknown (verify with device) |
| **PWM Outputs** | OUTPUT 1, OUTPUT 2 (20 KHz) | OUTPUT 4 (20 KHz) |

### Analog I/O Summary

**SK-2310g2 Analog Inputs**:
- **ADC-1** (CN6 pin 10): 0-10V (typically spindle F/V feedback)
- **ADC-2** (CN17 pin 3): 0-5V (general purpose, potentiometer-ready)
- **ADC-3** (CN17 pin 2): 0-5V (general purpose, potentiometer-ready)

**SK-2310g2 Analog Output**:
- **DAC** (CN6 pin 11): 0-10V (spindle speed control)
- Control method: Device-specific firmware command (consult SK-2310g2 documentation)


## Common Features

### Status Reporting

Logosol LDCN compliant I/O nodes convey their input state and other state details via status reporting. These nodes report status in response to `READ STATUS` and `NOP` commands and return a configurable set of status details.

To return a defined status one time, request status with `Read Status`, and append the byte encoded set of desired status items.

To define a persistent subset of status information, send the `Define Status` command and append the byte-encoded desired items. All future `Nop` commands will return the configured status items.

By default, both methods to read status will return the full set of status information (`0x00`). The status configuration is cleared upon `Hard Reset`.

It is not possible to read the state of outputs. The host must store the output state if desired.

---

### Status Response

Every status response packet consists of three parts, with packet size being the sum of `Define Status` item sizes:

#### 1. Status Byte

The first byte of every response packet contains error flags:

| Bit | Description | Note |
|-----|-------------|-----|
| **0** | Undefined | ignore |
| **1** | Checksum error flag | Set if a checksum error was detected in the most recent command packet |
| **2-7** | Undefined | ignore |

#### 2. Status Items

The data to include in status responses is encoded into a byte as follows. Set a bit to include the item, clear a bit to omit the item.

| Bit | Data Item | Size | Description |
|-----|-----------|------|-------------|
| **0** | Input Bytes | 2 bytes | Digital input Byte 0, Byte 1 |
| **1** | Analog Input 0 | 1 byte | Analog input channel 0 (0-255) |
| **2** | Analog Input 1 | 1 byte | Analog input channel 1 (0-255) |
| **3** | Analog Input 2 | 1 byte | Analog input channel 2 (0-255) |
| **4** | Counter/Timer | 4 bytes | Counter/timer value (LSB first) |
| **5** | Device ID/Version | 2 bytes | Device ID byte, Version byte|
| **6** | Sync Input Bits | 2 bytes | Input bits captured with Sync Input command |
| **7** | Sync Counter/Timer | 4 bytes | Counter/timer captured with Sync Input (LSB first) |

#### 3. Checksum Byte

The checksum byte is the 8-bit sum of the status byte plus all data bytes.

#### Item: Device ID and Version

When bit 5 is set, response includes:
- Byte 0: Device ID = `0x02` (LS-773)
- Byte 1: Version = `0x32` (50 decimal)

### Counter/Timer

One 32-bit selectable counter/timer is available with simple reset-to-zero overflow behavior. Application must detect wrap by polling and comparing consecutive readings.


- Timer mode: Counts intervals based on a 5 MHz clock (200 ns resolution)
- Counter mode: Counts external events on configured input
- Sync mode: Captures counter/timer value with Sync Input command


#### Counter Mode
- Counts high-to-low transitions
  - For LS-773, on DIGITAL IN 9/COUNT
  - For SK-2310g2, input TBD
- Configurable prescaler divides input frequency (1:1, 2:1, 4:1, 8:1)


#### Reset

To reset to zero:

1. Disable the counter/timer (Set Timer Mode, bit 0 = 0)
2. Re-enable the counter/timer (Set Timer Mode, bit 0 = 1)

## Command Reference

### Command Summary Table

| Command | Code | Data Bytes | Description |
|---------|------|------------|-------------|
| [Define Status](#define-status-command) | 0x2 | 1 | Defines which data should be sent in every status packet |
| [Read Status](#read-status-command) | 0x3 | 1 | Causes particular status data to be returned just once |
| [Set PWM](#set-pwm) | 0x4 | 2 | Set PWM duty cycle for OUTPUT 1 and OUTPUT 2 |
| [Sync Output](#sync-output) | 0x5 | 0 | Apply previously staged output values |
| [Set Outputs](#set-outputs) | 0x6 | 2 | Immediately set all digital output states |
| [Set Sync Output](#set-sync-output) | 0x7 | 4 | Stage output states and PWM values for later sync |
| [Set Timer Mode](#set-timer-mode) | 0x8 | 1 | Configure 32-bit counter/timer operation mode |
| [Sync Input](#sync-input-command) | 0xC | 0 | Atomically capture input states and counter value |

For LDCN network commands (Set Address, NOP, etc.), see [ldcn_protocol.md](ldcn_protocol.md).

---

### Define Status Command

**Command:** `0x02` (CMD_DEFINE_STATUS)
**Data bytes:** 1 byte (from **status items bitmap**)
**Default:** `0x00` (no items)
**Returns:** Yes - Status packet with defined status items

Causes subsequent `Nop` commands to return the defined status items.

`Hard Reset` or power cycle will return to `0x00`

---

### Read Status Command

**Command:** `0x03` (CMD_READ_STATUS)
**Data bytes:** 1 byte (from **status items bitmap**)
**Returns:** Yes - Status packet with requested status items (one time only)

This is a non-permanent version of the Define Status command. The status packet returned in response to this command will incorporate the data bytes specified, but subsequent status packets will include only the data bytes previously specified with the Define Status command.

---

### Set PWM

**Command:** `0x04` (CMD_SET_PWM)
**Data bytes:** 2 bytes
**Returns:** Yes - Standard status packet

For PWM capable outputs, sets the PWM duty cycle.

**Data**:
- Byte 0: PWM 1 output value (255-0)
- Byte 1: PWM 2 output value (255-0)

**PWM Value Mapping**:
- 255 = 0% PWM (output OFF)
- 128 = 50% PWM
- 0 = 100% PWM (output fully ON)

**Example**:
```python
# Set OUTPUT 1 to 50% PWM, OUTPUT 2 to 75% PWM
pwm1 = 128  # 50%
pwm2 = 64   # 75%
send_command(addr, 0x04, [pwm1, pwm2])
```

**Notes**:
- OUTPUT 1 and OUTPUT 2 must first be enabled for PWM mode by setting their output bits to `1` using `Set Outputs` command
- PWM frequency is fixed at 20 KHz
- To use PWM outputs as digital outputs, use `Set Outputs` command or set PWM to `0` or `255`.

---

### Sync Output

**Command:** `0x05` (CMD_SYNC_OUTPUT)<br>
**Data bytes:** 0 bytes <br>
**Returns:** Yes - Standard status packet

Synchronously applies output values previously stored with `Set Sync Output` command.

**Use Case**: Allows simultaneous state change on multiple outputs across multiple nodes.

First `Set Sync Output` to stage the values, then `Sync Output` to apply.

---

### Set Outputs

**Command:** `0x06` (CMD_SET_OUTPUTS)<br>
**Data bytes:** 2 bytes <br>
**Returns:** Yes - Standard status packet

Immediately sets the states of all digital output bits.

**Data**:
- Byte 0: Output bits (bit 0-6 = OUTPUT 0-6, bit 7 unused)
- Byte 1: Reserved (set to 0x00)

** LS-773 Output Bit Mapping**:
| Bit | Output | Description |
|-----|--------|-------------|
| 0   | OUTPUT 0/POWER | Solid-state relay, 0.5A |
| 1   | OUTPUT 1/PWM | Open collector, 1A, PWM capable |
| 2   | OUTPUT 2/PWM | Open collector, 1A, PWM capable |
| 3   | OUTPUT 3 | Open collector, 1A |
| 4   | OUTPUT 4 | Open collector, 1A |
| 5   | OUTPUT 5 | Open collector, 1A |
| 6   | OUTPUT 6 | Open collector, 1A |
| 7   | (unused) | Ignored |

**Example**:
```python
# Turn on OUTPUT 0, OUTPUT 2, and OUTPUT 5
outputs = 0b00100101  # bits 0, 2, 5 set
send_command(addr, 0x06, [outputs, 0x00])
```

**Notes**:
- To set PWM and digital values simultaneoustly, see `sync output` command.
- If a PWM-capable output is set to 1, it operates in PWM mode, with duty cycle set by `Set PWM`
- If a PWM-capable output is set to 0, it is immediately turned off, regardless of 'Set PWM' value
- All outputs are open collector, with diode protection for inductive loads
- Each output is short circuit protected. Shorting an output to POWER(+) turns off all outputs until next Set Outputs command.

---

### Set Sync Output

**Command:** `0x07` (CMD_SET_SYNC_OUTPUT) <br>
**Data bytes:** 4 bytes <br>
**Returns:** Yes - Standard status packet

Stores output states and PWM values for later synchronous application using `Sync Output`

**Data**:
- Byte 0: Output bits (bit 0-6 = OUTPUT 0-6, bit 7 unused)
- Byte 1: Reserved (set to 0x00)
- Byte 2: PWM 1 output value (255-0)
- Byte 3: PWM 2 output value (255-0)

**Example**:
```python
# Stage outputs: OUTPUT 0 and OUTPUT 1 on, OUTPUT 1 at 75% PWM
outputs = 0b00000011
pwm1 = 64   # 75%
pwm2 = 255  # off
send_command(addr, 0x07, [outputs, 0x00, pwm1, pwm2])

# Later, simultaneously apply the staged values
send_command(addr, 0x05, [])  # Sync Output
```

---

### Set Timer Mode

**Command:** `0x08` (CMD_SET_TIMER_MODE)<br>
**Data bytes:** 1 byte<br>
**Returns:** Yes - Standard status packet

Configures the 32-bit counter/timer operation mode.

**Data**:
- Byte 0: Timer mode configuration byte

**Configuration Byte Bits**:
| Bit | Function |
|-----|----------|
| 0   | 0 = Disabled, 1 = Enabled |
| 1   | 0 = Timer mode (5.0 MHz), 1 = Counter mode (DIGITAL IN 9/COUNT) |
| 4-5 | Prescaler: 00=1:1, 01=2:1, 10=4:1, 11=8:1 |
| 2,3,6,7 | Not used |

**Example**:
```python
# Enable counter mode with 4:1 prescaler
config = 0b00100011  # bit 0=1 (enable), bit 1=1 (counter), bits 4-5=10 (4:1)
send_command(addr, 0x08, [config])

# Enable timer mode (5.0 MHz clock), no prescaler
config = 0b00000001  # bit 0=1 (enable), bit 1=0 (timer), bits 4-5=00 (1:1)
send_command(addr, 0x08, [config])

# Reset counter (disable then re-enable)
send_command(addr, 0x08, [0x00])  # Disable
send_command(addr, 0x08, [0x03])  # Re-enable in counter mode
```

**Notes**:
- Counter value is read via Define Status or Read Status commands (bit 4)
- Counter wraps at 2^32 - 1 with no status indication or interrupt
- See [Counter/Timer](#countertimer) for overflow behavior and use cases

---

### Sync Input Command

**Command:** `0x0C` (CMD_SYNC_INPUT)<br>
**Data bytes:** 0 bytes<br>
**Returns:** Yes - Standard status packet

Captures current input states and counter/timer value atomically.

**Use Case**:
- Atomic, simultaneous snapshot of all digital inputs and timer/counter value
- Read captured values using `Read Status` with values reported on status bits 6 and 7

**Workflow**:
```python
# 1. Send Sync Input command
send_command(addr, 0x0C, [])

# 2. Read captured values using Read Status bit 6 (inputs) and bit 7 (counter)
status_bits = 0b11000000  # bits 6 and 7
response = send_command(addr, 0x03, [status_bits])

# Response contains:
# - Status byte
# - Captured input byte 0
# - Captured input byte 1
# - Captured counter value (4 bytes, LSB first)
# - Checksum
```

**Note:** There is no trigger or interrupt to capture all values on a counter/timer or  input transition. Capture can only be initiated via a LDCN command and is subject to transmission and command processing delays.

---

## Implementation Notes

### Efficient Status Configuration

For fast I/O polling, limit responses, e.g. for inputs only, status mask bit 0:

```python
# Initialize once
send_command(CMD_DEFINE_STATUS, [0x01])

# Poll status repeatedly
while True:
    response = send_command(CMD_NOP)
    # Response is 4 bytes: [status, input_byte0, input_byte1, checksum]
    status_byte = response[0]
    input_byte0 = response[1]
    input_byte1 = response[2]
    checksum = response[3]

    # Combine into 16-bit input word
    inputs = (input_byte1 << 8) | input_byte0

    # Verify checksum
    if status_byte & 0x02:
        print("Warning: Checksum error detected")
```

### Reading Analog Inputs

Include analog bits when needed:
```python
# Configure: inputs + all analog
send_command(CMD_DEFINE_STATUS, [0x0F])

# Read status
response = send_command(CMD_NOP)
# Response is 7 bytes: [status, in0, in1, ain0, ain1, ain2, checksum]
status_byte = response[0]     # Status byte
input_byte0 = response[1]     # Digital input byte 0
input_byte1 = response[2]     # Digital input byte 1
ain0 = response[3]            # Analog input 0 (0-255)
ain1 = response[4]            # Analog input 1 (0-255)
ain2 = response[5]            # Analog input 2 (0-255)
checksum = response[6]        # Checksum
```

## Examples

**Basic initialization**:

```python
def initialize_io_node(addr):
    # Step 1: Define status reporting (inputs, analog, counter)
    status_bits = 0x01 | 0x0E | 0x10  # inputs, all analog, counter
    send_command(addr, 0x02, [status_bits])

    # Step 2: Configure counter/timer if needed
    # Enable counter mode with no prescaler
    timer_config = 0b00000011  # bit 0=1 (enable), bit 1=1 (counter)
    send_command(addr, 0x08, [timer_config])

    # Step 3: Initialize outputs to known state
    send_command(addr, 0x06, [0x00, 0x00])  # All outputs off

    # Step 4: Read and verify status
    response = send_command(addr, 0x0E, [])  # NOP to read status
    return parse_io_status(response)
```

---

**PWM Output**:

- Inverted scale: 255=OFF, 0=FULLY ON
- Frequency: 20 KHz (fixed)
- Resolution: 8-bit (0-255)
- Only specific outputs support PWM


```python
# Step 1: Enable PWM outputs (set output bits to 1)
outputs = 0b00000110  # bits 1 and 2 for OUTPUT 1 and OUTPUT 2
send_command(addr, 0x06, [outputs, 0x00])

# Step 2: Set PWM duty cycles
pwm1 = 128  # 50% duty cycle
pwm2 = 64   # 75% duty cycle
send_command(addr, 0x04, [pwm1, pwm2])

# To disable PWM: set output bits to 0
send_command(addr, 0x06, [0x00, 0x00])
```


## References

- Logosol LS-773 Network I/O Node Datasheet (Doc# 712773001, Rev. B)
  - LS-773 Device ID: 2, Version: 50
- Logosol CNC-SK-2310g2 Supervisor I/O Controller Manual (Doc# 710231005, Rev. D)
  - Specialized supervisory controller for CNC machines
  - Application-specific I/O mappings
  - Includes analog output for spindle control
- [ldcn_protocol.md](ldcn_protocol.md) - Generic LDCN network protocol documentation

