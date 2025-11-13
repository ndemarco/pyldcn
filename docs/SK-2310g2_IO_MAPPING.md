# SK-2310g2 I/O Controller - Complete I/O Assignment Reference

**Device:** SK-2310g2 Supervisory I/O Controller (LS-773 based)
**Author:** NickyDoes
**Source:** CNC-SK-2310g2 Manual, Doc # 710231005 / Rev. D, 03/05/2020
**Date:** 2025-11-12
**Status:** 📋 Documentation - Preparing for Implementation

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
| **Safety/Guarding** | 1 (servo fault) | 3 (guard lock, safety bridge, covers) | 0 | 0 | CN3, CN9, CN10 |
| **System Status** | 2 (at home, test mode) + 5 LEDs | 3 (home enable, test inhibit, system lock) | 0 | 0 | Internal |
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

**Status Response Format (LS-773):**
```
response = send_command(CMD_READ_STATUS, [0xFF, 0xFF])
byte0 = response[1]  # Application inputs (bits 0-7)
byte1 = response[2]  # System status + diagnostic (bits 8-15 and diagnostic code)
```

**Important:** The LS-773 response includes Position, Velocity, and Home fields from the standard LDCN motion controller protocol. These fields are **vestigial** for the SK-2310g2 I/O controller and should be **ignored**. The SK-2310g2 has no motors or motion control capabilities. Use `byte0` and `byte1` for actual I/O state.

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
       [15:8][7:0]
```

**Command Format:**
```python
send_command(CMD_SET_OUTPUTS, [byte0, byte1])
# byte0 = outputs & 0xFF
# byte1 = (outputs >> 8) & 0xFF
```

### Byte0 - Application Outputs (Bits 0-7)

| Bit | Name | Connector | Function | Notes |
|-----|------|-----------|----------|-------|
| **0** | Output 0 | CN14 | Program Running Lamp | Non-dedicated (can be general purpose) |
| **1** | Output 1 | CN14 | Program Stopped Lamp | Non-dedicated (can be general purpose) |
| **2** | Output 2 | CN6 | Spindle ON | See Note 3 (safety constraint), Note 4 (jumpers) |
| **3** | Output 3 | CN6 | Spindle Direction | 0=CW, 1=CCW (can be general purpose) |
| **4** | Output 4 | CN6 | Spindle DC-brake | DC brake/speed control (can be general purpose) |
| **5** | Output 5 | CN7 | Tool Clamp | Solenoid control (can be general purpose) |
| **6** | Output 6 | CN7 | Spindle Motor Cooling | Coolant control (can be general purpose) |
| **7** | Output 7 | CN7 | Tool Cooling | Solenoid control (can be general purpose) |

**Note 3: CRITICAL SAFETY CONSTRAINT**

**Spindle ON (Bit 2)** and **Safety Link Bridge (Bit 12)** CANNOT be used simultaneously.

Requirements:
- If one is set to 1, the other MUST be 0
- To activate either output, the other MUST be turned off first
- The firmware will THROW AN ERROR if both are set to 1 simultaneously

This is a **hardware safety requirement** enforced by the controller.

**Note 4: Spindle Control Configuration**

See "Sample application – Spindle Option 1" and "Sample application – Spindle Option 2" in the manual for jumper configurations (J16, J20, J10-3) controlling spindle behavior with guards open.

### Byte1 - System Control Outputs (Bits 8-15)

| Bit | Name | Connector | Function | Notes |
|-----|------|-----------|----------|-------|
| **8** | Output 8 | CN7 | Tool Changer Unlock | Can be general purpose |
| **9** | Output 9 | CN9, CN10 | Guard Lock | 1=locked, 0=unlocked |
| **10** | Output 10 | Internal | Home Enable | Automation mode dependent |
| **11** | Output 11 | Internal | Test Mode Inhibit | 1=prevent manual override |
| **12** | Output 12 | CN3 Safety Bus | Safety Link Bridge | See Note 3 (safety constraint) |
| **13** | Output 13 | CN7 | Inverted Output | Logic HIGH when bit=0 |
| **14** | Output 14 | Internal | Reserved (set to 0) | Guards Lock/Unlock if J10-2 and J19 shorted (Note 5) |
| **15** | Output 15 | Internal | System Lock | Power ON/OFF if J21 shorted (Note 6) |

**Note 5: Guards Lock/Unlock (Output 14)**

J10-2 and J19 must both be shorted to enable automatic guard unlocking in Zero Speed automation mode.

**Note 6: Power ON/OFF (Output 15)**

J21 must be shorted to enable software power control. With J21 open (default), power can ONLY be controlled via physical button.

---

## Analog Inputs (3 channels)

### Reading Analog Inputs

Analog inputs are returned in the status response. Exact encoding TBD - requires hardware verification.

**Expected format:** 10-bit ADC values (0-1023) representing voltage ranges.

| Channel | Physical Connector | Range | Function | Application |
|---------|-------------------|-------|----------|-------------|
| **0** | CN6.10 | 0-10V | **Spindle Load** | Spindle current/load feedback from LS2315 drive |
| **1** | CN17.3 | 0-5V | **ADC2** | General purpose analog input |
| **2** | CN17.2 | 0-5V | **ADC3** | General purpose analog input |

### Analog Input Scaling

**Spindle Load (0-10V, 10-bit ADC):**
```python
adc_value = 0-1023
voltage = (adc_value / 1023.0) * 10.0  # 0-10V
load_percent = (voltage / 10.0) * 100.0  # 0-100%
```

**General Purpose (0-5V, 10-bit ADC):**
```python
adc_value = 0-1023
voltage = (adc_value / 1023.0) * 5.0  # 0-5V
```

---

## Analog Output (1 channel)

### Writing Analog Output

Analog output control command TBD - requires hardware verification. Likely uses `CMD_SET_PWM_IO` or a DAC command.

| Channel | Physical Connector | Range | Function | Application |
|---------|-------------------|-------|----------|-------------|
| **0** | CN6.11 | 0-10V | **Spindle Speed Command** | Speed command to LS2315 spindle drive |

### Analog Output Scaling

**Spindle Speed (0-10V):**
```python
speed_percent = 0-100  # Desired speed as percentage
voltage = (speed_percent / 100.0) * 10.0  # 0-10V
# Send voltage to DAC channel 0
```

The LS2315 spindle drive interprets:
- 0V = stopped
- 10V = maximum RPM (50K/60K/100K depending on LS2315 DIP switch configuration)

---

## Connector-to-I/O Quick Reference

### CN3 - Safety Bus Connector

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1 | Safety Link OUT | Output | (Hardware) | Generated based on guards/e-stop/safe state |
| 2 | Safety Link IN | Input | (Hardware) | Monitors daisy chain integrity (diagnostic 0x0A if LOW) |
| 3 | Enable/Stop | Output | (Hardware) | HIGH when power ON |
| 4 | ServoFAULT | Input | Input 10 (Byte1/Bit2) | Prevents power-on if HIGH |

### CN6 - Spindle Interface Connector

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 2 | Spindle Stopped | Input | Used in Spindle OFF computation | Required HIGH for guard unlock |
| 3 | Spindle Fault | Input | Input 3 (Byte0/Bit3) | General fault monitoring |
| 4 | Spindle At Speed | Input | Input 4 (Byte0/Bit4) | Speed reached confirmation |
| 5 | Spindle ON | Output | Output 2 (Byte0/Bit2) | Enable signal to LS2315 |
| 6 | Spindle Direction | Output | Output 3 (Byte0/Bit3) | 0=CW, 1=CCW |
| 7 | Spindle DC-brake | Output | Output 4 (Byte0/Bit4) | DC brake control |
| 10 | Spindle LOAD | Analog In | Analog Input 0 (0-10V) | Load feedback from LS2315 |
| 11 | Spindle SPEED | Analog Out | Analog Output 0 (0-10V) | Speed command to LS2315 |

**Note:** CN6 connects pin-for-pin to LS2315 CN7 (spindle drive connector).

### CN7 - General I/O Connector

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1 | Input 5 | Input | Input 5 (Byte0/Bit5) | Air Pressure OK |
| 3 | Input 6 | Input | Input 6 (Byte0/Bit6) | Tool length measurement switch |
| 5 | Input 7 | Input | Input 7 (Byte0/Bit7) | Tool changer closed |
| 7 | Output 5 | Output | Output 5 (Byte0/Bit5) | Tool clamp solenoid |
| 9 | Output 6 | Output | Output 6 (Byte0/Bit6) | Spindle motor cooling |
| 11 | Output 7 | Output | Output 7 (Byte0/Bit7) | Tool cooling |
| 13 | Output 8 | Output | Output 8 (Byte1/Bit0) | Tool changer unlock |

### CN8 - Safe Zone Sensor (Home Switch)

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1-2 | Home A1-A2 | Input | (Hardware) | Contact A - Closed in safe state |
| 3-4 | Home B1-B2 | Input | (Hardware) | Contact B - Open in safe state |

Result reflected in Input 8 (At Home / Safe State).

### CN9, CN10 - Guard Switches (Guard 1, Guard 2)

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1-2 | Guard Closed (A) | Input | (Hardware) | Dual-channel safety contact A |
| 3-4 | Guard Unlock | Output | Output 9 (Byte1/Bit1) | Solenoid unlock control |
| 5-6 | Guard Closed (B) | Input | (Hardware) | Dual-channel safety contact B |

Guard states reflected in diagnostic codes 0x14-0x1F. "Guard" = safety guard/guarding (Logosol term: "Cover").

### CN13 - Manual Override and Unlock Control

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 1-2 | Test Mode Switch | Input | Input 9 (Byte1/Bit1) | Manual override keyswitch |
| 3-4 | Acknowledge (ACK) Button | Input | (Hardware) | Hold-to-run enable button |
| 5-6 | Unlock Switch | Input | (Hardware) | Manual guard unlock button |
| 7 | Unlock Enable | Output | (Internal) | Auto unlock (J17, J19 dependent) |

"Test Mode" = Manual override mode. "ACK" (Acknowledge) = Enable button for motion with guards open.

### CN14 - General I/O Connector

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| TBD | Input 0 | Input | Input 0 (Byte0/Bit0) | Program run |
| TBD | Input 1 | Input | Input 1 (Byte0/Bit1) | Program stop |
| TBD | Output 0 | Output | Output 0 (Byte0/Bit0) | Program running lamp |
| TBD | Output 1 | Output | Output 1 (Byte0/Bit1) | Program stopped lamp |

### CN16 - Power Control Connector

| Pin | Signal | Type | Bit Assignment | Notes |
|-----|--------|------|----------------|-------|
| 5 | Monitor Loop (-) | Input | (Hardware) | Relay monitor return |
| 6 | Monitor Loop (+) | Output | (Hardware) | Relay monitor source |
| 8 | Power Enable (UM) | Output | (Hardware) | Motor voltage contactor control |
| 10 | Spindle ON | Output | (Hardware, same as CN6.5) | Wired to CN6.5 internally |

"UM" = Motor voltage (Logosol term from German "Ursprung der Spannung").

### CN17 - Analog Input Connector

| Pin | Signal | Type | Channel | Notes |
|-----|--------|------|---------|-------|
| 2 | ADC3 | Analog In | Analog Input 2 (0-5V) | General purpose |
| 3 | ADC2 | Analog In | Analog Input 1 (0-5V) | General purpose |

---

## Implementation Guidance for sk2310g2.py

When implementing high-level methods in [pyldcn/devices/sk2310g2.py](../pyldcn/devices/sk2310g2.py), use these I/O assignments as the reference.

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

---

## Testing Checklist

When testing on hardware, verify these I/O assignments:

### Digital Inputs
- [ ] Input 0 (Program Run) - CN14
- [ ] Input 1 (Program Stop) - CN14
- [ ] Input 3 (Spindle Fault) - CN6.3
- [ ] Input 4 (Spindle At Speed) - CN6.4
- [ ] Input 5 (Air Pressure) - CN7.1
- [ ] Input 6 (Measure Switch) - CN7.3
- [ ] Input 7 (Tool Changer Closed) - CN7.5
- [ ] Input 8 (At Home) - Internal/CN8
- [ ] Input 9 (Test Mode) - CN13.1-2
- [ ] Input 10 (Servo Fault) - CN3.4

### Digital Outputs
- [ ] Output 0 (Program Running Lamp) - CN14
- [ ] Output 1 (Program Stopped Lamp) - CN14
- [ ] Output 2 (Spindle ON) - CN6.5
- [ ] Output 3 (Spindle Direction) - CN6.6
- [ ] Output 4 (Spindle DC-brake) - CN6.7
- [ ] Output 5 (Tool Clamp) - CN7.7
- [ ] Output 6 (Spindle Motor Cooling) - CN7.9
- [ ] Output 7 (Tool Cooling) - CN7.11
- [ ] Output 8 (Tool Changer Unlock) - CN7.13
- [ ] Output 9 (Guard Lock) - CN9.3-4, CN10.3-4

### Analog I/O
- [ ] Analog Input 0 (Spindle Load) - CN6.10
- [ ] Analog Input 1 (ADC2) - CN17.3
- [ ] Analog Input 2 (ADC3) - CN17.2
- [ ] Analog Output 0 (Spindle Speed) - CN6.11

### Safety Constraints
- [ ] Verify Output 2 and Output 12 cannot both be 1 simultaneously
- [ ] Verify Spindle OFF computation (Input 2)
- [ ] Verify guard unlock conditions work correctly
- [ ] Verify diagnostic codes reflect guard states

---

## Related Documentation

- [SK-2310g2_supervisor.md](SK-2310g2_supervisor.md) - Safety system, diagnostic codes, jumper configuration
- [pyldcn/devices/io.py](../pyldcn/devices/io.py) - SK2310g2 class implementation
- [pyldcn/devices/sk2310g2.py](../pyldcn/devices/sk2310g2.py) - LS-773 status parsing utilities
- CNC-SK-2310g2 Manual, Doc # 710231005 / Rev. D - Official hardware manual

---

## Notes and Observations

### Current Understanding
- Physical I/O assignments are well-documented in the manual (page 19)
- Bit-level assignments are clearly defined
- Safety constraints are explicitly documented
- Connector pinouts are specified

### Needs Clarification (Hardware Testing Required)
1. **Analog I/O command formats** - How to read/write analog values via LDCN commands
2. **Status response format** - Exact encoding of analog values in status response
3. **Guard contact mapping** - Which input bits reflect guard door closed state (CN9.1-2, CN9.5-6)
4. **CN14 pinout** - Specific pin assignments for Inputs 0-1 and Outputs 0-1

### Design Decisions for Implementation
1. Use constants defined in [pyldcn/devices/io.py](../pyldcn/devices/io.py) for bit positions
2. Validate safety constraints in software before sending commands
3. Provide both low-level (bit manipulation) and high-level (named methods) interfaces
4. Return human-readable dictionaries from status reading methods
5. Raise clear exceptions when safety constraints would be violated

---

**Next Steps:**
1. Use this document to guide implementation of high-level methods in SK2310g2 class
2. Test each I/O assignment on hardware and mark verified items
3. Document any deviations or corrections discovered during testing
4. Add hardware-specific notes (timing requirements, quirks, etc.)
