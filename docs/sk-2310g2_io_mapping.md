# SK-2310g2 I/O Controller - Complete I/O Assignment Reference

**Device:** SK-2310g2 Supervisory I/O Controller (LS-773 based)
**Author:** NickyDoes
**Source:** CNC-SK-2310g2 Manual, Doc # 710231005 / Rev. D, 03/05/2020
**Date:** 2025-11-12

---

## Terminology Note

Logosol documentation uses certain terms that may be confusing to US English speakers:

- **"Cover"** = Safety guard/guarding
- **"At Home"** = Machine in safe state
- **"Test Mode"** = Manual override mode (keyswitch-enabled, allows limited unguarded operations with operator presence detection)
- **"ACK" (Acknowledge)** = Enable button, enables motion when guarding is not safe
- **"UM" or "Um"** = Motor voltage

See [SK-2310g2_supervisor.md](SK-2310g2_supervisor.md) for complete safety system documentation.

---

## Document Purpose

Complete reference for all I/O assignments on the SK-2310g2. Use this to implement high-level methods in [pyldcn/devices/io.py](../pyldcn/devices/io.py) SK2310g2 class.

---

## Quick Reference: I/O Categories

| Category | Digital Inputs | Digital Outputs | Analog In | Analog Out | Related Connector |
|----------|----------------|-----------------|-----------|------------|-------------------|
| **Spindle Control** | 3 (fault, at-speed, stopped) | 3 (ON, direction, DC-brake) | 1 (load 0-10V) | 1 (speed 0-10V) | CN6 |
| **Safety/Guarding** | 2 guard zones (A&B each) + E-stop (A&B) | 2 lock releases + safety bridge | 0 | 0 | CN9, CN10, CN11, CN13, CN15, CN4 |
| **System Status** | 2 (at home, test mode) + 5 LEDs | 3 (home enable, test inhibit, system lock) | 0 | 0 | Internal (*expand later*) |
| **Application I/O** | 8 (general purpose) | 8 (general purpose) | 2 (0-5V GP) | 0 | CN7, CN14, CN17 |

---

## Digital Inputs (16 bits = Byte1:Byte0)

### Reading Digital Inputs

Digital inputs are returned in the status response as a 16-bit word split into two bytes:

```
Inputs = Byte1:Byte0 (MSB:LSB)
         ↓    ↓
      [15:8][7:0]
```

**Reading Inputs:**

```python
# Configure status once (bit 0 = include digital inputs)
send_command(CMD_DEFINE_STATUS, [0x01])

# Read digital inputs
response = send_command(CMD_READ_STATUS, [0xFF, 0xFF])
byte0 = response[1]  # Application inputs (bits 0-7)
byte1 = response[2]  # System status (bits 8-15)
inputs = (byte1 << 8) | byte0  # 16-bit word

# Check specific input
if inputs & (1 << 5):  # Input 5 (Air Pressure)
    print("Air pressure OK")
```

For detailed status configuration options, see [sk-2310g2_status_reporting.md](sk-2310g2_status_reporting.md).

### Byte0 - Application Inputs (Bits 0-7)

| Bit | Name | Connector | Function | Notes |
|-----|------|-----------|----------|-------|
| **0** | Input 0 | CN14 | Program Run | Non-dedicated (can be general purpose) |
| **1** | Input 1 | CN14 | Program Stop | Non-dedicated (can be general purpose) |
| **2** | Input 2 | CN6 | Spindle OFF | Computed - See Note 1 |
| **3** | Input 3 | CN6 | Spindle Fault | From spindle drive (can be general purpose) |
| **4** | Input 4 | CN6 | Spindle At Speed | Speed reached confirmation (can be general purpose) |
| **5** | Input 5 | CN7 | Air Pressure OK | Pneumatic pressure switch (can be general purpose) |
| **6** | Input 6 | CN7 | Tool Length Switch | Measurement probe (can be general purpose) |
| **7** | Input 7 | CN7 | Tool Changer Closed | Cover closed sensor (can be general purpose) |

### Digital Input Electrical Specifications

Inputs use industrial 24V-compatible voltage ranges:

| Input(s) | Logic Low (VL) | Logic High (VH) | Max Current | Notes |
|----------|----------------|-----------------|-------------|-------|
| **Input 2** | < 2.4V | > 17V | 33mA | Spindle OFF (computed input) |
| **Inputs 0, 1** | 0.5V < VL < 6.5V | 15V < VH < 36V | 1mA | Program run/stop |
| **Inputs 3-7** | 0.5V < VL < 6.5V | 15V < VH < 36V | 8mA | General purpose/spindle |

All inputs are compatible with 24VDC industrial control systems.

**Note 1: Spindle OFF (Input 2) Computation**

This is a **computed input** (not directly wired). It is set to 1 when:

- `Spindle ON` output (Outputs/Byte0/Bit 2) = 0 **AND**
- `Spindle Stopped` input (CN6 pin 2) = HIGH

This provides software confirmation that the spindle is both commanded OFF and physically stopped.

### Byte1 - System Status Inputs (Bits 8-15)

| Bit | Name | Connector | Function | Notes |
|-----|------|-----------|----------|-------|
| **8** | Input 8 | Internal | At Home (Safe State) | See Note 2 |
| **9** | Input 9 | CN13 | Test Mode (Manual Override) | Keyswitch-enabled manual mode |
| **10** | Input 10 | CN3 Safety Bus | Servo Fault | From safety bus, prevents power-on |
| **11** | Input 11 | Internal | Status LED 1 | Diagnostic code bit 3 |
| **12** | Input 12 | Internal | Status LED 2 | Diagnostic code bit 4 |
| **13** | Input 13 | Internal | Status LED 3 | Diagnostic code bit 5 |
| **14** | Input 14 | Internal | Status LED 4 | Diagnostic code bit 6 |
| **15** | Input 15 | Internal | Status LED 5 | Diagnostic code bit 7 |

**Note 2: At Home (Safe State)**

Indicates machine is in safe state. Controlled by home sensor (CN8) or Zero Speed mode (J10 configuration). Set to 0 when Test Mode with Acknowledge active. See [SK-2310g2_supervisor.md](SK-2310g2_supervisor.md) for safe state detection modes.

**Note:** Bits 11-15 reflect diagnostic LED states encoded in Byte1[7:3]. See diagnostic codes in supervisor doc.

---

## Digital Outputs (16 bits = Byte1:Byte0)

### Writing Digital Outputs

Digital outputs are sent as a 16-bit word split into two bytes:

```
Outputs = Byte1:Byte0 (MSB:LSB)
          ↓    ↓
        [15:8] [7:0]
```

**Writing Outputs:**

```python
# Set outputs as 16-bit word
outputs = 0x0000
outputs |= (1 << 2)   # Set Output 2 (Spindle ON)
outputs |= (1 << 9)   # Set Output 9 (Guard Lock)

# Send to controller
byte0 = outputs & 0xFF
byte1 = (outputs >> 8) & 0xFF
send_command(CMD_SET_OUTPUTS, [byte0, byte1])
```

### Digital Output Electrical Specifications

Outputs use dry contact relays:

| Specification | Value | Notes |
|---------------|-------|-------|
| **V_max** | 40VDC | Dry contact relay outputs |
| **I_max** | 0.5A | Per output |
$H_2O$

### Byte0 - Application Outputs (Bits 0-7)

| Bit | Name | Connector | Function | Notes |
|-----|------|-----------|----------|-------|
| **0** | Output 0 | CN14 | Program Running Lamp | |
| **1** | Output 1 | CN14 | Program Stopped Lamp | |
| **2** | Output 2 | CN6 | Spindle ON | See Note 3 (safety constraint), Note 4 (jumpers) |
| **3** | Output 3 | CN6 | Spindle Direction | 0=CW, 1=CCW |
| **4** | Output 4 | CN6 | Spindle DC-brake | DC brake/speed control |
| **5** | Output 5 | CN7 | Tool Clamp | Solenoid control |
| **6** | Output 6 | CN7 | Spindle Motor Cooling | Coolant control |
| **7** | Output 7 | CN7 | Tool Cooling | Solenoid control  |

**Note 3: CRITICAL SAFETY CONSTRAINT**

**Spindle ON (Bit 2)** and **Safety Link Bridge (Bit 12)** must not be set simultaneously. This is a hardware safety requirement enforced by the controller.

**Note 4: Spindle Control Configuration**

See "Sample application – Spindle Option 1" and "Sample application – Spindle Option 2" in the manual for jumper configurations (J16, J20, J10-3) controlling spindle behavior with guards open.

### Byte1 - System Control Outputs (Bits 8-15)

| Bit | Name | Connector | Function | Notes |
|-----|------|-----------|----------|-------|
| **8** | Output 8 | CN7 | Tool Changer Unlock |
| **9**| Output 9 | CN9, CN10 | Guard Lock | 1=locked, 0=unlocked |
| **10** | Output 10 | Internal | Home Enable | Automation mode dependent |
| **11** | Output 11 | Internal | Manual Mode Inhibit | 1=prevent manual override |
| **12** | Output 12 | CN3 | Safety Link Bridge | See Note 3 (safety constraint) |
| **13** | Output 13 | CN7 | Inverted Output | Logic HIGH when bit=0 |
| **14** | Output 14 | Internal | Reserved (set to 0) | Guards Lock/Unlock if J10-2 and J19 shorted (Note 5) |
| **15** | Output 15 | Internal | Software power | Controls power ON/OFF if J21 shorted (Note 6) |

**Note 5: Guards Lock/Unlock (Output 14)**

J10-2 and J19 must both be shorted to enable automatic guard unlocking in Zero Speed automation mode.

**Note 6: Power ON/OFF (Output 15)**

J21 must be shorted to enable software power control. With J21 open (default), power can only be controlled via physical button.

---

## Analog Inputs (3 channels)

### Reading Analog Inputs

Analog inputs are returned in status response when configured. See [sk-2310g2_status_reporting.md](sk-2310g2_status_reporting.md) for configuration details.

| Channel | Connector | Resolution | Range | Function | Notes |
|---------|-------------------|------|-------|----------|-------------|
| **0** | CN6.10 | 8-bit | 0-10V | **Spindle Load** | Spindle current/load feedback from LS2315 drive |
| **1** | CN17.3 | 8-bit | 0-5V | **ADC2** | General purpose analog input |
| **2** | CN17.2 | 8-bit | 0-5V | **ADC3** | General purpose analog input |

**Reading Analog Inputs:**

```python
# Configure status: inputs + all analog (bits 0-3)
send_command(CMD_DEFINE_STATUS, [0x0F])

# Read status
response = send_command(CMD_READ_STATUS, [0xFF, 0xFF])
ain0 = response[3]  # Analog 0 (0-255 for 0-10V)
ain1 = response[4]  # Analog 1 (0-255 for 0-5V)
ain2 = response[5]  # Analog 2 (0-255 for 0-5V)
```

## Analog Output (1 channel)

| Channel | Connector | Resolution | Range | Function |
|---------|-------------------|-----|-------|----------|
| **0** | CN6.11 | 8-bit | 0-10V | Spindle speed |

### Writing Analog Output

**Command:** `CMD_SET_PWM_IO` (0x04) - data format requires hardware verification

```python
# Set spindle speed as percentage
speed_percent = 75  # 75% of max RPM
dac_value = int((speed_percent / 100.0) * 255)

# Send to PWM/DAC (data format TBD - likely [channel, value])
send_command(CMD_SET_PWM_IO, [channel, dac_value])
```

## I/O Connector Reference

### CN3 - Safety Bus Connector

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1 | Safety Link OUT | Output | (Hardware) | Generated based on guards/e-stop/safe state |
| 2 | Safety Link IN | Input | (Hardware) | Monitors daisy chain integrity (diagnostic 0x0A if LOW) |
| 3 | Enable/Stop | Output | (Hardware) | HIGH when power ON |
| 4 | ServoFAULT | Input | Byte1/Bit2 | Prevents power-on if HIGH |

### CN4 - LDCN Slave Connector (with E-stop)

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 11-12 | E-stop A | Input | (Hardware) | Emergency stop channel A (series chain) |
| 13-14 | E-stop B | Input | (Hardware) | Emergency stop channel B (series chain) |
| Other | LDCN Bus | - | - | LDCN network communication pins |

### CN6 - Spindle Interface Connector

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 2 | Spindle Stopped | Input | Used in Spindle OFF computation | Required HIGH for guard unlock |
| 3 | Spindle Fault | Input | Byte0/Bit3 | General fault monitoring |
| 4 | Spindle At Speed | Input | Byte0/Bit4 | Speed reached confirmation |
| 5 | Spindle ON | Output | Byte0/Bit2 | Enable signal to LS2315 |
| 6 | Spindle Direction | Output | Byte0/Bit3 | 0=CW, 1=CCW |
| 7 | Spindle DC-brake | Output | Byte0/Bit4 | DC brake control |
| 10 | Spindle LOAD | Analog In | Analog Input 0 (0-10V) | Load feedback from LS2315 |
| 11 | Spindle SPEED | Analog Out | Analog Output 0 (0-10V) | Speed command to LS2315 |

**Note:** CN6 connects pin-for-pin to LS2315 CN7 spindle drive connector.

### CN7 - General I/O Connector

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1 | Input 5 | Input | Byte0/Bit5 | Air Pressure OK |
| 3 | Input 6 | Input | Byte0/Bit6 | Tool length measurement switch |
| 5 | Input 7 | Input | Byte0/Bit7 | Tool changer closed |
| 7 | Output 5 | Output | Byte0/Bit5 | Tool clamp solenoid |
| 9 | Output 6 | Output | Byte0/Bit6 | Spindle motor cooling |
| 11 | Output 7 | Output | Byte0/Bit7 | Tool cooling |
| 13 | Output 8 | Output | Byte1/Bit0 | Tool changer unlock |

### CN8 - Safe Zone Sensor (Home Switch)

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1-2 | Home A1-A2 | Input | (Hardware) | Contact A - Closed in safe state |
| 3-4 | Home B1-B2 | Input | (Hardware) | Contact B - Open in safe state |

Result reflected in Input 8 (At Home / Safe State).

### CN9 - Guard Zone 1

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1-2 | Guard 1 Closed (A) | Input | (Hardware) | Redundant monitored safety contact A |
| 3-4 | Guard 1 Lock Release | Output | Byte1/Bit1 | Solenoid unlock control |
| 5-6 | Guard 1 Closed (B) | Input | (Hardware) | Redundant monitored safety contact B |

### CN10 - Guard Zone 2

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1-2 | Guard 2 Closed (A) | Input | (Hardware) | Redundant monitored safety contact A |
| 3-4 | Guard 2 Lock Release | Output | Byte1/Bit1 | Solenoid unlock control (shared with Output 9) |
| 5-6 | Guard 2 Closed (B) | Input | (Hardware) | Redundant monitored safety contact B |

**Guard System:**

- Two independent guard zones, each with redundant monitored inputs (A & B channels)
- Each zone has a lock release output for solenoid control
- Guard states reflected in diagnostic codes 0x14-0x1F
- "Guard" = safety guard/guarding (Logosol term: "Cover")
- Lock solenoid energized state can be changed with jumpers J14 & J15

### CN11 - E-stop

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1-2 | E-stop A | Input | (Hardware) | in series with other e-stop contacts |
| 3-4 | E-stop B | Input | (Hardware) | in series with other e-stop contacts |

**E-stop System:**

- Redundant monitored E-stop with A & B channels
- Serially connected through 4 connectors: CN13.1-4, CN15.1-4, CN11.1-4, CN4.11-14
- All E-stop contacts must be closed for system to enable

### CN13 - Manual Control and E-stop

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1-2 | E-stop A | Input | (Hardware) | in series with other e-stop contacts |
| 3-4 | E-stop B | Input | (Hardware) | in series with other e-stop contacts |
| 5-6 | Unlock Switch | Input | (Hardware) | guard unlock button |
| 7-8 | Operator Present Button | Input | (Hardware) | Hold-to-run enable button |
| TBD | Manual Override Mode | Input | (Byte1/Bit1) | keyswitch |

"Test Mode" = Manual override mode. "ACK" (Acknowledge) = Enable button for motion with guards open.

### CN14 - General I/O Connector

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| TBD | Input 0 | Input | Byte0/Bit0 | Program run |
| TBD | Input 1 | Input | Byte0/Bit1 | Program stop |
| TBD | Output 0 | Output | Byte0/Bit0 | Program running lamp |
| TBD | Output 1 | Output | Byte0/Bit1 | Program stopped lamp |

### CN15 - E-stop and Power Monitor

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1-2 | E-stop A | Input | (Hardware) | Emergency stop channel A (series chain) |
| 3-4 | E-stop B | Input | (Hardware) | Emergency stop channel B (series chain) |
| 5-6 | Power Button | Input | (Hardware) | Power ON/OFF button |
| 7 | Power Lamp | Output | (Hardware) | Flashing when ready, ON when powered |
| 8 | GND | - | - | Ground |
| 9 | Manual Override Lamp | Output | (Hardware) | ON when in manual override mode |
| 10 | GND | - | - | Ground |
| 11 | Manual Override A | Input | (Hardware) | Normally open contact |
| 12 | +24VDC | - | - | +24V supply |
| 13 | Manual Override B | Input | (Hardware) | Normally closed contact |

**Manual Override Mode:**

- Requires A & B contact transitions within 100ms
- Manual Override Inhibit (Output 11, Byte1/Bit3) must be 0

### CN16 - Power Control Connector

**Power Supply Enable Functionality:**

- Redundant power enable inputs (A & B channels) - must be shorted to ground to enable power
- Spindle power enable output
- Power monitor loop (pins 5-6) verifies relay contacts are properly closed

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1 | Ground | - | - | Ground reference for Power Enable A |
| 2 | Power Enable A | Input | (Hardware) | Short to pin 1 (GND) to enable power |
| 4 | Power Enable B | Input | (Hardware) | Short to pin 9 (GND) to enable power |
| 5 | Power Monitor Input | Input | (Hardware) | Return from series-connected relay contacts |
| 6 | Power Monitor Output | Output | (Hardware) | -27V monitor supply |
| 8 | Power Enable (UM) | Output | (Hardware) | Motor voltage contactor control |
| 9 | Ground | - | - | Ground reference for Power Enable B |
| 10 | Spindle Power Enable | Output | (Hardware) | Spindle power control (connects to CN6.5) |

**Power Monitor Loop:**

- Pin 6 outputs -27V
- Pin 5 receives return through series-connected power-on relay contacts
- Verifies relay contacts are properly closed before enabling power

**Notes:**

- "UM" = Motor voltage (Logosol term)
- Power enable inputs A & B are redundant - both must be shorted to ground for power-on

---

### CN17 - Analog Input Connector

| Pin | Signal | Type | Channel | Notes |
|-----|--------|------|---------|-------|
| 2 | ADC3 | Analog In | Analog Input 2 (0-5V) | General purpose |
| 3 | ADC2 | Analog In | Analog Input 1 (0-5V) | General purpose |

---

## Implementation Guidance

When implementing high-level methods in the SK2310g2 class ([pyldcn/devices/io.py](../pyldcn/devices/io.py)), use these I/O assignments as the reference.

**Note:** The [pyldcn/devices/sk2310g2.py](../pyldcn/devices/sk2310g2.py) module contains parsing utilities and label mappings, while the SK2310g2 device class is implemented in io.py.

### Example: Spindle Control Implementation

```python
# Constants (already defined in io.py)
INPUT_SPINDLE_FAULT = 3        # Byte0/Bit3
INPUT_SPINDLE_AT_SPEED = 4     # Byte0/Bit4
OUTPUT_SPINDLE_ON = 2          # Byte0/Bit2
OUTPUT_SPINDLE_DIRECTION = 3   # Byte0/Bit3
ANALOG_OUT_SPINDLE_SPEED = 0   # CN6.11
ANALOG_IN_SPINDLE_LOAD = 0     # CN6.10

def enable_spindle(self, direction: str = 'forward') -> None:
    """Enable spindle with direction control."""
    # Read current outputs
    current_outputs = self.read_digital_outputs()

    # Set spindle ON bit and direction bit
    new_outputs = self._set_bit(current_outputs, OUTPUT_SPINDLE_ON, True)
    new_outputs = self._set_bit(new_outputs, OUTPUT_SPINDLE_DIRECTION, direction == 'reverse')

    # Validate safety constraint (Output 2 and Output 12 cannot both be 1)
    self._validate_spindle_safety_constraint(new_outputs)

    # Send outputs
    self.set_outputs(new_outputs)

def read_spindle_status(self) -> Dict[str, Any]:
    """Read all spindle-related inputs and analog feedback."""
    inputs = self.read_digital_inputs()

    return {
        'fault': self._get_bit(inputs, INPUT_SPINDLE_FAULT),
        'at_speed': self._get_bit(inputs, INPUT_SPINDLE_AT_SPEED),
        'load_voltage': self.read_analog_input(ANALOG_IN_SPINDLE_LOAD),
    }
```

### Example: Guard Lock Control Implementation

```python
# Constants (already defined in io.py)
OUTPUT_GUARD_LOCK = 9  # Byte1/Bit1

def lock_guards(self) -> None:
    """Lock guard doors."""
    self.set_output_bit(OUTPUT_GUARD_LOCK, True)

def unlock_guards(self) -> None:
    """Unlock guard doors (if conditions allow)."""
    self.set_output_bit(OUTPUT_GUARD_LOCK, False)
```

## Related Documentation

- [sk-2310g2_status_reporting.md](sk-2310g2_status_reporting.md) - LS-773 status protocol and Define Status command
- [SK-2310g2_supervisor.md](SK-2310g2_supervisor.md) - Safety system, diagnostic codes, jumper configuration
- [pyldcn/devices/io.py](../pyldcn/devices/io.py) - SK2310g2 class implementation
- [pyldcn/devices/sk2310g2.py](../pyldcn/devices/sk2310g2.py) - LS-773 status parsing utilities
- CNC-SK-2310g2 Manual, Doc # 710231005 / Rev. D - Official hardware manual

---

## Notes and Observations

### Needs Clarification (Hardware Testing Required)

5. **Counter/Timer input** - Verify if Input 9 (CN13.1-2) is the counter input for Set Timer Mode (0x8), or identify which input is used. Note: Input 9 is currently assigned to Manual Override keyswitch, which may conflict with counter mode

### Design Decisions for Implementation

1. Use constants defined in [pyldcn/devices/io.py](../pyldcn/devices/io.py) for bit positions
2. Validate safety constraints in software before sending commands
