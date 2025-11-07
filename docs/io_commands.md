# LS-773 Network I/O Node Commands

This document describes commands specific to the **LS-773 Network I/O Node** (and likely other Logosol I/O controllers).

---

## Device Overview

The LS-773 is a LDCN network compliant I/O controller.

**Hardware Features**:
- 10 general purpose digital inputs (with configurable pull-up/pull-down)
- 6 open collector outputs (1A max each)
- 1 solid-state relay output (0.5A max, OUTPUT 0/POWER)
- 3 analog inputs (8-bit, 0-5V/0-10V/0-20V/0-30V selectable)
- 32-bit counter/timer with prescaler (5.0 MHz clock)
- 20 KHz PWM mode for OUTPUT 1 and OUTPUT 2
- Device ID: 2, Version: 50

---

## Counter/Timer Overview

The LS-773 includes a **32-bit counter/timer** with the following characteristics:

### Operating Modes

**Timer Mode** (internal clock):
- Counts 5.0 MHz internal clock
- Resolution: 200 ns per count
- Maximum time: 858 seconds (14.3 minutes) at 1:1 prescaler

**Counter Mode** (external input):
- Counts high-to-low transitions on DIGITAL IN 9/COUNT
- Prescaler divides input frequency (1:1, 2:1, 4:1, 8:1)

### Overflow Behavior

The counter/timer is a **free-running counter** with simple wraparound behavior:

- **Maximum value**: 2^32 - 1 (4,294,967,295)
- **On overflow**: Wraps to 0 and continues counting
- **No interrupt**: Counter wraps silently with no status flag or notification
- **No hardware detection**: Application must detect wraparound by comparing consecutive readings

### Reset Method

There is **no direct counter reset command**. To reset the counter to zero:

1. Disable the counter/timer (Set Timer Mode, bit 0 = 0)
2. Re-enable the counter/timer (Set Timer Mode, bit 0 = 1)

### Practical Use Cases

The LS-773 is a **polled I/O controller**, not an interrupt-driven real-time controller. The counter/timer is designed for:

1. **External Event Counting** - Encoder pulses, production parts, flow meter pulses
2. **Elapsed Time Measurement** - Time intervals between polls (not precise event timing)
3. **Synchronized Capture** - Hardware-latched position/time snapshots using Synch Input (0xC)

**For high-accuracy timed events** (e.g., "trigger output when counter reaches X"), use motion controllers (like LS-370) or PLCs with hardware timer/comparator capabilities.

---

## Command Summary

For generic LDCN network commands (Set Address, Define Status, NOP, etc.), see [LDCN_PROTOCOL.md](LDCN_PROTOCOL.md).

| Command | Code | Data Bytes | Description |
|---------|------|------------|-------------|
| [Set PWM](#set-pwm-0x4) | 0x4 | 2 | Set PWM duty cycle for OUTPUT 1 and OUTPUT 2 |
| [Synch Output](#synch-output-0x5) | 0x5 | 0 | Apply previously staged output values |
| [Set Outputs](#set-outputs-0x6) | 0x6 | 2 | Immediately set all digital output states |
| [Set Synch Output](#set-synch-output-0x7) | 0x7 | 4 | Stage output states and PWM values for later sync |
| [Set Timer Mode](#set-timer-mode-0x8) | 0x8 | 1 | Configure 32-bit counter/timer operation mode |
| [Synch Input](#synch-input-0xc) | 0xC | 0 | Atomically capture input states and counter value |

---

## Command Detail

### **Set PWM** (0x4)

Sets the PWM duty cycle for PWM-capable outputs.

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
- PWM frequency is fixed at 20 KHz (TODO: This may be selectable - verify using LDCN Utility)
- For simple on/off control, use `Set Outputs` command or set PWM to `0` or `1`.

---

### **Synch Output** (0x5)

Synchronously applies output values previously stored with `Set Synch Output` command.

**Data**: None

**Example**: Claude TODO: Verify this works across nodes by reviewing the LS-773 Logosol PDF. Also explain the data format (0x5 is the command; What is 0x6 (checksum) and why Device 1? How do I sync across multiple device nodes?)
```
AA 01 05 06  # Synch output on device 1
```

**Use Case**: Allows simultaneous state change on multiple outputs across multiple nodes.

First `Set Sync Output` to stage the values, then `Sync Output` to apply. (Claude TODO: Use the more modern 'Sync' instead of Synch.)

---

### **Set Outputs** (0x6)

Immediately sets the states of all digital output bits.

**Data**:
- Byte 0: Output bits (bit 0-6 = OUTPUT 0-6, bit 7 unused)
- Byte 1: Reserved (set to 0x00)

**Output Bit Mapping**:
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
- Outputs change immediately upon command receipt
- If OUTPUT 1 or OUTPUT 2 bit is set to 1, they operate in PWM mode (duty cycle set by Set PWM)
- If OUTPUT 1 or OUTPUT 2 bit is set to 0, they are turned off regardless of PWM setting
- All open collector outputs have protective diodes for inductive loads
- Short circuit protection: if any output is shorted to POWER(+), all outputs turn off until next Set Outputs command

---

### **Set Synch Output** (0x7)

Stores output states and PWM values for later synchronous application.

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
send_command(addr, 0x05, [])  # Synch Output
```
---

### **Set Timer Mode** (0x8)

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
- See [Counter/Timer Overview](#countertimer-overview) for overflow behavior and use cases

---

### **Synch Input** (0xC)

Captures current input states and counter/timer value atomically.

**Data**: None

**Example**:
```
AA 01 0C 0D  # Synch input on device 1
```

**Use Case**:
- Atomic snapshot of all digital inputs and counter value
- Useful for synchronized multi-axis position capture
- Captured values are read via Define Status or Read Status commands (bits 6 and 7)

**Workflow**:
```python
# 1. Send Synch Input command
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

---

## Status Byte

The status byte returned by the LS-773 has minimal bit definitions:

| Bit | Name | Description |
|-----|------|-------------|
| 1   | cksum_error | Checksum error in received command packet |
| 0,2-7 | (unused) | Undefined, can be ignored |

**Fault Condition**:
- Bit 1 = 1: Checksum error - resend command

---

## Define Status / Read Status Data Bits

When using Define Status (0x2) or Read Status (0x3) commands, the following data bits are available:

| Bit | Data Returned | Size | Description |
|-----|---------------|------|-------------|
| 0   | Input bits | 2 bytes | DIGITAL IN 0-9, OUT_SH flag |
| 1   | ANALOG IN 0 | 1 byte | 8-bit ADC value (0-255) |
| 2   | ANALOG IN 1 | 1 byte | 8-bit ADC value (0-255) |
| 3   | ANALOG IN 2 | 1 byte | 8-bit ADC value (0-255) |
| 4   | Counter/Timer | 4 bytes | 32-bit value, LSB first |
| 5   | Device ID | 2 bytes | Device ID (2) and version (50) |
| 6   | Synch Input bits | 2 bytes | Captured by Synch Input (0xC) |
| 7   | Synch Counter | 4 bytes | Captured by Synch Input (0xC) |

**Input Bit Layout** (Status bit 0):

Byte 0:
| Bit | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|-----|---|---|---|---|---|---|---|---|
| Input | IN7 | IN6 | IN5 | IN4 | IN3 | IN2 | IN1 | IN0 |

Byte 1:
| Bit | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|-----|---|---|---|---|---|---|---|---|
| Input | - | - | - | - | - | - | IN9 | IN8 |
| Flag | - | - | - | - | - | OUT_SH | - | - |

**OUT_SH Flag** (Byte 1, bit 1):
- OUT_SH = 1: One or more outputs are shorted to POWER(+)
- OUT_SH = 0: Normal operation

---

## Digital Input Configuration

Digital inputs have configurable pull resistors (10K) via DIP switches or jumper:

**Active LOW** (default):
- Inputs have pull-up resistors to POWER(+)
- Input reads 1 when pulled to GND
- Input reads 0 when floating or at POWER(+)

**Active HIGH**:
- Inputs have pull-down resistors to GND
- Input reads 1 when connected to POWER(+) or SENSOR POWER
- Input reads 0 when floating or at GND

---

## Analog Input Configuration

Analog inputs are 8-bit (0-255) and support multiple voltage ranges via DIP switches:

**Voltage Range Selection** (DIP switches 3-8):
- 0-5V
- 0-10V
- 0-20V
- 0-30V

**Resolution**:
- 5V range: ~19.6 mV per count
- 10V range: ~39.2 mV per count
- 20V range: ~78.4 mV per count
- 30V range: ~117.6 mV per count

**Example**:
```python
# Read all three analog inputs
status_bits = 0b00001110  # bits 1, 2, 3
response = send_command(addr, 0x03, [status_bits])
# Response: [status_byte, an0, an1, an2, checksum]

# Convert to voltage (assuming 0-10V range)
voltage = (response[1] / 255.0) * 10.0
```

---

## Output Protection

**Short Circuit Protection**:
- All outputs are protected against short circuits
- If any open collector output (OUTPUT 1-6) is shorted to POWER(+), all outputs turn off
- Normal operation resumes after next Set Outputs command
- OUT_SH flag in input status indicates short condition

**Overcurrent Protection**:
- OUTPUT 0/POWER: 0.5A max, solid-state relay with short-to-GND protection
- OUTPUT 1-6: 1A max per output, open collector with short-to-POWER(+) protection

---

## LS-773 Initialization Sequence

Basic initialization sequence for I/O controller:

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

## PWM Output Configuration

To use OUTPUT 1 and OUTPUT 2 as PWM outputs:

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

**PWM Specifications**:
- Frequency: 20 KHz (fixed)
- Resolution: 8-bit (256 levels)
- Only OUTPUT 1 and OUTPUT 2 support PWM
- Inverted scale: 255=OFF, 0=FULLY ON

---

## SK-2310g2 I/O Hardware Comparison

The **SK-2310g2** is a specialized supervisory I/O controller that uses the same LDCN command set (0x4, 0x5, 0x6, 0x7, 0x8, 0xC) but has different I/O hardware capabilities.

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

### Application Notes

**For I/O-focused applications**: Use LS-773 for general-purpose I/O with counter/timer

**For supervisory/safety applications**: See [SUPERVISORY_CONTROLLER.md](SUPERVISORY_CONTROLLER.md) for:
- Safety system architecture and jumper configuration
- Diagnostic codes and troubleshooting
- Spindle control with safety interlocks
- LDCN command reference for safety monitoring
- Application-specific wiring and integration

---

## References

- Logosol LS-773 Network I/O Node Datasheet (Doc# 712773001, Rev. B)
  - LS-773 Device ID: 2, Version: 50
- Logosol CNC-SK-2310g2 Supervisor I/O Controller Manual (Doc# 710231005, Rev. D)
  - Specialized supervisory controller for CNC machines
  - Application-specific I/O mappings
  - Includes analog output for spindle control
