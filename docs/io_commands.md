# LS-773 Network I/O Node Commands

This document describes commands specific to the **LS-773 Network I/O Node** (and likely other Logosol I/O controllers).

For generic LDCN network commands (Set Address, Define Status, NOP, etc.), see `LDCN_PROTOCOL.md`.

---

## Device Overview

The LS-773 is a multifunctional I/O controller designed for a wide range of applications.

**Hardware Features**:
- 10 general purpose digital inputs (with configurable pull-up/pull-down)
- 6 open collector outputs (1A max each)
- 1 solid-state relay output (0.5A max, OUTPUT 0/POWER)
- 3 analog inputs (8-bit, 0-5V/0-10V/0-20V/0-30V selectable)
- 32-bit counter/timer with prescaler (5.0 MHz clock)
- 20 KHz PWM mode for OUTPUT 1 and OUTPUT 2
- Device ID: 2, Version: 50

**Key Specifications**:
- Power supply: 12 to 32Vdc
- Communication speed: 19.2 Kbps to 1.25 Mbps
- Command rate: up to 1000/sec
- Up to 31 LS-773 nodes on a single network

---

## I/O-Specific Commands

### 0x4 - Set PWM

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
- OUTPUT 1 and OUTPUT 2 must first be enabled for PWM mode by setting their output bits to 1 using Set Outputs (0x6)
- PWM frequency is fixed at 20 KHz
- For simple on/off control, set PWM to 0 (fully on) or use Set Outputs command

---

### 0x5 - Synch Output

Synchronously applies output values previously stored with Set Synch Output command.

**Data**: None

**Example**:
```
AA 01 05 06  # Synch output on device 1
```

**Use Case**: Allows multiple outputs to change state simultaneously across multiple nodes. First use Set Synch Output (0x7) to stage the values, then use Synch Output (0x5) to apply them atomically.

---

### 0x6 - Set Outputs

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

### 0x7 - Set Synch Output

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

# Later, apply the staged values atomically
send_command(addr, 0x05, [])  # Synch Output
```

**Use Case**: Coordinate simultaneous output changes across multiple I/O nodes on the network.

---

### 0x8 - Set Timer Mode

Configures the 32-bit counter/timer operation mode.

**Data**:
- Byte 0: Timer mode configuration byte

**Configuration Byte Bits**:
| Bit | Function |
|-----|----------|
| 0   | 0 = Counter/timer disabled, 1 = Counter/timer enabled |
| 1   | 0 = Timer mode (internal clock), 1 = Counter mode (external input) |
| 4-5 | Prescaler: 00=1:1, 01=2:1, 10=4:1, 11=8:1 |
| 2,3,6,7 | Not used |

**Timer Mode** (bit 1 = 0):
- Counts internal 5.0 MHz clock
- Resolution: 200 ns per count
- Max time (no prescaler): 858 seconds (14.3 minutes)

**Counter Mode** (bit 1 = 1):
- Counts high-to-low transitions on DIGITAL IN 9/COUNT input
- Prescaler divides input frequency

**Example**:
```python
# Enable counter mode with 4:1 prescaler
config = 0b00100011  # bit 0=1 (enable), bit 1=1 (counter), bits 4-5=10 (4:1)
send_command(addr, 0x08, [config])

# Enable timer mode (5.0 MHz clock), no prescaler
config = 0b00000001  # bit 0=1 (enable), bit 1=0 (timer), bits 4-5=00 (1:1)
send_command(addr, 0x08, [config])
```

**Notes**:
- Counter value is read via Define Status or Read Status commands (bit 4)
- Counter is 32-bit, wraps at 2^32 - 1
- Prescaler applies to both timer and counter modes

---

### 0xC - Synch Input

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

---

## SK-2310g2 Supervisor I/O Controller

The **SK-2310g2** (model CNC-SK-2310g2) is a **specialized supervisory I/O controller** designed for CNC machine control systems. Unlike the generic LS-773, it has dedicated safety interlock functions and spindle control capabilities.

### Device Overview

**Hardware Features**:
- 7 universal digital inputs
- 8 short-protected digital outputs
- 3 analog inputs (ADC-1: 0-10V, ADC-2/3: 0-5V)
- 1 analog output (0-10V for spindle speed control)
- Dual mechanical relay power supply control
- Spindle control with safety Enable/Stop relay
- Emergency stop monitoring (dual line)
- Work zone cover control (dual contact with Lock/Unlock)
- Safe zone sensor interface (dual line NC/NO)
- Safety Bus interface (LS-231 compatible)
- 5 diagnostic LEDs

**Key Specifications**:
- Power supply: 18 to 32Vdc
- Communication speed: 19.2 Kbps to 1.25 Mbps
- Safety-rated relay contacts (40Vdc, 0.5A)
- Device ID: (varies by firmware)
- Version: (varies by firmware)

**Note**: The SK-2310g2 uses the **same LDCN command set** as the LS-773 (commands 0x4, 0x5, 0x6, 0x7, 0x8, 0xC), but has application-specific I/O mappings and additional analog output capability.

---

### SK-2310g2 Specific Features

#### Analog Output (Spindle Speed Control)

The SK-2310g2 includes a 0-10V **analog output** (DAC) on connector CN6 pin 11 for controlling spindle speed.

**Connector**: CN6 - SPINDLE, Pin 11 (DAC)
**Range**: 0 to 10V
**Resolution**: Device-specific (typically 8-bit or 10-bit)
**Application**: Spindle VFD speed control

**Control Method**:
The analog output is typically controlled via a device-specific command or by writing to an internal register. Consult the SK-2310g2 firmware documentation for the exact command structure.

**Typical Usage**:
```python
# Example: Set spindle speed to 50% (5.0V)
# Exact command structure depends on SK-2310g2 firmware
# This may use a vendor-specific extension command or DEFINE_STATUS register write
```

**Pin Assignment** (CN6 - SPINDLE):
| Pin | Signal | Description |
|-----|--------|-------------|
| 1   | GND | Ground |
| 2   | Spindle Stopped | Input: HIGH=Spindle stopped |
| 3   | Input 3 | General purpose (typically Spindle FAULT) |
| 4   | Input 4 | General purpose (typically Spindle AT SPEED) |
| 5   | Spindle ON | Output: Spindle ENABLE relay |
| 6   | Output 3 | General purpose (typically Spindle REVERSE) |
| 7   | Output 4 | General purpose or PWM |
| 8   | +24V | Short protected 24V source |
| 9   | Analog GND | Analog ground |
| 10  | ADC | Analog input 0-10V (Spindle F/V feedback) |
| 11  | **DAC** | **Analog output 0-10V (Spindle SPEED control)** |

---

#### Analog Inputs

Unlike the LS-773 (which has uniform analog input configuration), the SK-2310g2 has **different voltage ranges** for its analog inputs:

**Analog Input Configuration**:
| Input | Connector | Range | Description |
|-------|-----------|-------|-------------|
| ADC-1 (ADC) | CN6 pin 10 | 0-10V | Spindle F/V (actual speed feedback) |
| ADC-2 | CN17 pin 3 | 0-5V | General purpose analog input |
| ADC-3 | CN17 pin 2 | 0-5V | General purpose analog input |

**CN17 - ANALOG INPUTS**:
| Pin | Signal | Description |
|-----|--------|-------------|
| 1   | POT (-) | GND or 100Ω to GND (J7 selectable) |
| 2   | ADC 3 | Analog input 0-5V |
| 3   | ADC 2 | Analog input 0-5V |
| 4   | POT (+) | +5V or 100Ω to +5V (J6 selectable) |

**Note**: ADC-2 and ADC-3 can be used with potentiometers using the POT(+) and POT(-) pins for reference voltage.

---

#### Digital I/O Mapping (Application-Specific)

The SK-2310g2 has **application-specific I/O mappings** for supervisory control. The following is the standard configuration from the documentation:

**Digital Inputs (Byte0)**:
| Bit | Input | Function | Connector | Application |
|-----|-------|----------|-----------|-------------|
| 0   | Input 0 | Non-dedicated | CN14 | Program run |
| 1   | Input 1 | Non-dedicated | CN14 | Program stop |
| 2   | Input 2 | **Spindle OFF** | CN6 | Spindle stopped detection |
| 3   | Input 3 | Non-dedicated | CN6 | Spindle fault |
| 4   | Input 4 | Non-dedicated | CN6 | Spindle at speed |
| 5   | Input 5 | Non-dedicated | CN7 | Air pressure sensor |
| 6   | Input 6 | Non-dedicated | CN7 | Measurement switch |
| 7   | Input 7 | Non-dedicated | CN7 | Tool changer closed |

**Digital Inputs (Byte1)** - Status/Safety:
| Bit | Input | Function | Source | Description |
|-----|-------|----------|--------|-------------|
| 0   | Input 8 | **At Home** | Internal | Safe zone active |
| 1   | Input 9 | **Test Mode** | Internal | Test mode active |
| 2   | Input 10 | **Servo Fault** | CN3 | Servo drive fault monitor |
| 3-7 | Input 11-15 | **Status LEDs** | Internal | LED1-LED5 status |

**Digital Outputs (Byte0)**:
| Bit | Output | Function | Connector | Application |
|-----|--------|----------|-----------|-------------|
| 0   | Output 0 | Non-dedicated | CN14 | Program running lamp |
| 1   | Output 1 | Non-dedicated | CN14 | Program stopped lamp |
| 2   | Output 2 | **Spindle ON** | CN6 | Spindle ENABLE output |
| 3   | Output 3 | Non-dedicated | CN6 | Spindle direction |
| 4   | Output 4 | Non-dedicated/PWM | CN6 | Spindle DC-braking or PWM |
| 5   | Output 5 | Non-dedicated | CN7 | Tool clamp |
| 6   | Output 6 | Non-dedicated | CN7 | Spindle motor cooling |
| 7   | Output 7 | Non-dedicated | CN7 | Tool cooling |

**Digital Outputs (Byte1)** - Supervisory:
| Bit | Output | Function | Connector | Description |
|-----|--------|----------|-----------|-------------|
| 0   | Output 8 | Non-dedicated | CN7 | Tool changer unlock |
| 1   | Output 9 | **Cover Lock** | CN9, CN10 | Cover lock control |
| 2   | Output 10 | Home Enable | Internal | Home enable (automation modes) |
| 3   | Output 11 | **Test Mode Inhibit** | Internal | Disable test mode entry |
| 4   | Output 12 | **Safety Link Bridge** | CN3 | Safety bus bridge control |
| 5   | Output 13 | Inverted output | CN7 | General purpose (inverted) |
| 6   | Output 14 | Reserved | - | Cover lock/unlock (alternate) |
| 7   | Output 15 | **System Lock** | Internal | System lock / Power ON/OFF |

**Important Notes**:
- **Spindle ON (Output 2)** and **Safety Link Bridge (Output 12)** cannot be used simultaneously
- **Input 2 (Spindle OFF)** = 1 when: Output 2 (Spindle ON) = 0 AND physical spindle stopped signal = HIGH
- See SK-2310g2 wiring diagrams for proper spindle control configuration (Option 1 vs Option 2)

---

#### SK-2310g2 Diagnostic Codes

The SK-2310g2 provides diagnostic information via **Byte1 bits 3-7** and **LED indicators**.

**Common Diagnostic Codes** (Byte1):
| Code | Byte1 | Description | LED Pattern |
|------|-------|-------------|-------------|
| 0x04 | 0x04 | Control voltage LOW (<18V) | LED: ●●○●● |
| 0x08 | 0x08 | System LOCKED | LED: ●○●●● |
| 0x0C | 0x0C | Cover Open Stop (Cover open, spindle not stopped) | LED: ●○○●● |
| 0x10 | 0x10 | Emergency Stop | LED: ○●●●● |
| 0x14 | 0x14 | Cover-1 Open, Cover-2 Open (ready to power) | LED: ○●○●● |
| 0x1C | 0x1C | At Home, Spindle stopped, Covers Open | LED: ○○○●● |
| 0x1F | 0x1F | Normal operation - Covers Closed, Power ON | LED: ○○○○○ |
| 0x00 | 0x00 | Power OFF delay in progress | LED: Blinking |

**LED Indicators**:
- ● = OFF
- ○ = ON
- ☼ = BLINK

**Reading Diagnostic Code**:
Use the Define Status or Read Status command with appropriate status bits to retrieve Byte1, then decode the upper 5 bits (bits 3-7) for diagnostic information.

---

#### Spindle Control

The SK-2310g2 provides comprehensive spindle control with safety interlocks:

**Spindle Control Signals**:
1. **Spindle ON Output** (Output 2 / Byte0 Bit2, CN6 pin 5):
   - Controls spindle enable relay
   - Active when: Output2=1 AND Safety Link Bridge=0 AND Power ON AND Covers closed
   - Additional conditions based on J16 jumper configuration

2. **Spindle Speed Control** (Analog Output, CN6 pin 11):
   - 0-10V analog output to spindle VFD
   - Controls spindle RPM
   - Typically: 0V = 0 RPM, 10V = Max RPM

3. **Spindle Stopped Input** (Input 2 / Byte0 Bit2, CN6 pin 2):
   - HIGH = Spindle is stopped
   - Used for safety interlocks (cover unlock, test mode)

4. **Spindle Feedback** (Analog Input, CN6 pin 10):
   - 0-10V analog input from spindle VFD
   - Actual spindle speed (tachometer/F-to-V output)
   - Used for "at speed" detection and monitoring

**Spindle Control Example Workflow**:
```python
# 1. Enable spindle (set Output 2)
outputs = 0b00000100  # Bit 2 = Spindle ON
send_command(addr, 0x06, [outputs, 0x00])

# 2. Set spindle speed (device-specific command for analog output)
# This requires SK-2310g2 firmware-specific command
# Consult device documentation for analog output command

# 3. Monitor spindle status
response = send_command(addr, 0x03, [0x01])  # Read inputs
status_byte0 = response[1]
spindle_stopped = (status_byte0 >> 2) & 0x01  # Check Input 2

# 4. Read spindle speed feedback
response = send_command(addr, 0x03, [0x02])  # Read analog input 0
adc_value = response[1]  # ADC-1 value (spindle F/V)
```

**Safety Notes**:
- Spindle cannot be enabled when covers are open (unless in Test Mode with specific jumper settings)
- Spindle ON requires Safety Link Bridge to be inactive
- Emergency stop immediately disables spindle
- Cover lock status affects spindle enable

---

## Common I/O Controller Commands

The following commands apply to **both LS-773 and SK-2310g2** devices (and other Logosol I/O controllers):

### Command Summary Table

| Command | Code | Data Bytes | LS-773 | SK-2310g2 | Description |
|---------|------|------------|--------|-----------|-------------|
| Set PWM | 0x4 | 2 | ✓ | ✓ | Set PWM duty cycle |
| Synch Output | 0x5 | 0 | ✓ | ✓ | Apply staged outputs |
| Set Outputs | 0x6 | 2 | ✓ | ✓ | Set all output states |
| Set Synch Output | 0x7 | 4 | ✓ | ✓ | Stage outputs for sync |
| Set Timer Mode | 0x8 | 1 | ✓ | ? | Configure counter/timer |
| Synch Input | 0xC | 0 | ✓ | ? | Capture inputs atomically |

**Note**: SK-2310g2 may not support all LS-773 features (counter/timer). Verify with device documentation.

---

## References

- Logosol LS-773 Network I/O Node Datasheet (Doc# 712773001, Rev. B)
  - LS-773 Device ID: 2, Version: 50
- Logosol CNC-SK-2310g2 Supervisor I/O Controller Manual (Doc# 710231005, Rev. D)
  - Specialized supervisory controller for CNC machines
  - Application-specific I/O mappings
  - Includes analog output for spindle control
