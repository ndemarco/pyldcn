# SK-2310g2 Supervisory Controller Guide

**Author:** NickyDoes
**Source:** CNC-SK-2310g2 Manual, Doc # 710231005 / Rev. D, 03/05/2020

---

## Important Implementation Note (2025-11-12)

**Reading Status from SK-2310g2:**

The SK-2310g2 uses **LS-773 format**, NOT standard LDCN format. This is critical for correct parsing.

### Key Differences from Standard LDCN:

1. **Response byte order is different**: Byte0/Byte1 come FIRST, immediately after status byte
2. **NOP returns minimal status**: Only `[status_byte, checksum]` - no diagnostic code
3. **CMD_READ_STATUS required**: Must use CMD_READ_STATUS (0x03) with `[0xFF, 0xFF]` to get diagnostic

### LS-773 Response Format:
```
[status] [byte0] [byte1] [position] [ad] [velocity] [aux] [home] [dev_id] [pos_err] [pathbuf] [analog] [checksum]
 idx=0    idx=1   idx=2    idx=3-6   ...
```

**NOT** standard LDCN format:
```
[status] [position] [ad] [velocity] ... [byte0] [byte1] [analog] [checksum]  ← WRONG for SK-2310g2
```

### Correct Implementation:
```python
# Method 1: Use the dedicated parser (recommended)
from pyldcn.devices.sk2310g2 import parse_ls773_status
response = device.send_command(CMD_READ_STATUS, [0xFF, 0xFF])
status = parse_ls773_status(response)  # Returns complete status dict

# Method 2: Manual parsing
response = send_command(addr, CMD_READ_STATUS, [0xFF, 0xFF])
byte0 = response[1]  # Digital inputs
byte1 = response[2]  # Internal status + diagnostic
diagnostic = (byte1 >> 3) & 0x1F  # Extract diagnostic from Byte1 bits [7:3]
```

### Incorrect Approaches:
```python
# WRONG: Using NOP - no diagnostic code returned
response = send_command(addr, CMD_NOP)  # Returns only [status] [checksum]

# WRONG: Parsing as standard LDCN format
byte0 = response[18]  # Would read wrong data - this is pathbuf in LS-773 format
byte1 = response[19]  # Would read wrong data - this is analog data in LS-773 format
```

---

## Document Purpose

This document covers:
- Safety system architecture and jumper configuration
- Application-specific wiring and integration
- Diagnostic codes and troubleshooting
- LDCN command reference for safety monitoring

**For I/O hardware specifications** (digital/analog I/O, PWM, counter/timer), see [io_commands.md](io_commands.md).

**Prerequisites:** Read [safety_bus.md](safety_bus.md) for generic Safety Bus specification.

**Terminology Note:** Certain Logosol documentation terms may be confusing to US English speakers:
- **"Cover"** = Safety guard/guarding
- **"At Home"** = Machine in safe state
- **"Test Mode"** = Manual override mode (key-switch inhibited, allows limited unguarded operations, optionally with operator presence detection input enabled)
- **"ACK" (acknowledgewledge)** - Enable, enables motion when guarding is not ensured to be safe.
- **UM** or **Um** = Motor voltage (from German *Ursprung der Spannung* = potential difference)
---

## Quick Navigation

**Getting Started:**
- [Safety System Overview](#safety-system-overview)
- [Jumper Quick Reference](#jumper-quick-reference)
- [Configuration Recipes](#configuration-recipes)

**Usage:**
- [LDCN Command Reference](#ldcn-command-reference)
- [Diagnostic Codes](#diagnostic-codes)
- [Troubleshooting](#troubleshooting-by-symptom)
- [Testing Procedures](#testing-procedures)

**Reference:**
- [Python API Reference](#python-api-reference)
- [Wiring Examples](#wiring-examples)
- [Detailed Jumper Configuration](#detailed-jumper-configuration)
- [Connector Pinouts](#connector-pinouts)

---

# Python API Reference

The `pyldcn.devices.sk2310g2` module provides encoding/decoding utilities for SK-2310g2 status and I/O operations.

## Module Functions

**Status Parsing:**
- `parse_ls773_status(response)` - Parse LS-773 format response into status dictionary
- `format_status(status)` - Format status dict as human-readable multi-line string
- `format_led_pattern(diagnostic)` - Format diagnostic code as LED pattern (🟢⚫🟢🟢🟢)

**Data Access:**
- `DIAGNOSTIC_CODES[code]` - Lookup table mapping codes (0x00-0x1F) to condition descriptions

**Output Encoding:**
- `encode_output_byte0(outputs)` - Encode digital outputs 1-8 for SET_OUTPUTS command
- `encode_output_byte1(power_a, power_b)` - Encode power outputs A/B for SET_OUTPUTS command

**Status Dictionary Fields:**

Raw bytes: `status`, `byte0`, `byte1`

SK-2310g2 fields: `diagnostic`, `power_state`, `safe_state`, `manual_override`, `servo_fault`

Digital inputs: `input1`-`input6`, `spindle_stopped`, `spindle_fault`

LDCN motion fields: `position`, `velocity`, `home`, `device_id`, `version`

See module source for detailed function signatures and usage examples.

---

# Safety System Overview

## Role as Safety Master

The SK-2310g2 serves as the **safety system coordinator** in LDCN-based CNC systems:

**Core Safety Functions:**
1. **Emergency Stop Monitoring** - Five e-stop connections
2. **Dual Guarded Zones** - Two guard monitor circuits with positive lock
3. **Safe State Detection** - Machine is considered safe when guards are closed and drives stationary
3. **Power control and monitoring** - Motive power is enabled, disabled, and monitored.
4. **Restricted Manual Override** - Restricted manual mode with Operator Presence (hold-to-run) control
5. **Safety Link Bus** - Master Safety Link OUT based on all conditions
6. **ServoFAULT Detection** - Monitors attached drives for faults
7. **Power Supply Control** - A & B channel power control with monitoring loop
9. **Diagnostic Reporting** - Diagnostic LEDs and status codes indicate system state

---
## Emergency Stop
Operators command an emergency stop (e-stop) by activating an emergency stop switch. E-stop switches operate two electrically independent normally closed contacts. The 2310g2 supports e-stop switches via [any of four connectors](#cn3---safety-bus-connector),  monitored in parallel; activating any e-stop switch puts the 2310g2 in emergency stop state.

The integrity of the e-stop system is assured by:
1. Normally closed architecture - failure of a wire activates e-stop
1. dual circuit - failure of a single switch contact raises a [system alarm](#diagnostic-codes)

**E-stop state**
Enterint e-stop state:
- disconnects drive motor (UM) power (CN16.8)
- disables SafetyLINK drive enable (CN3.3)
- disconnects spindle power supply (? how)
- affects digital outputs and status code

**Timing Requirement:**
- Transition time between contacts must not exceed 100msec
- Exceeding this triggers [home sensor fault (diagnostic code 0x05)](#diagnostic-code-table)

**Related states:**
- Normal - contacts A & B were closed simultaneously
- Timing violation - contacts closed with >100msec interval
- Invalid state at power-up - e-stop contacts not in a valid state when system powered on (e.g., one contact open and one closed, indicating damaged switch or wiring fault). See [diagnostic code 0x06](#diagnostic-code-table)

## Guard Door Management
Up to two guarded zones may be configured. Select restricted operations can be allowed when one of the guard zones is open.

Guard doors can be in several states depending on configuration and operating mode:
- **Locked/Closed** - Normal production mode, guards secured and locked
- **Unlocked Safe State** - Machine in safe state, guards can be opened for access
- **Manual Override Unlock** - Manual override with keyswitch and enable button held
- **Automated Unlock** - Automatic unlock when safe conditions met (Zero Speed mode)

See [CMD_READ_OUTPUT](#cmd_read_output-0x0e---output-status) for Guard Lock status bit and [Jumper Configuration](#j19---guard-lockunlock-control-mode) for control modes.

## Safe State Concept with Restricted Manual Override
The machine can be in different states of operator protection, meant to allow reduced functions for setup or maintenenance. The safe state depends on restricted mode selection, normally via a key switch, an operator presence detection, and the overall configuration of the SK-2310g2

## Safety Link Bus
Each device is series-linked via a hardware bus (independent of the LDCN) to:
- form an electrical safety chain where any single device fault stops all devices
- share enable and fault states across all devices.

**Valid States:**
- **Normal** - All safety conditions met, system operational
- **Fault** (Safety Link OUT = LOW) - E-stop activated, guard open, or diagnostic fault
- **Chain Broken** ([diagnostic code 0x0A](#diagnostic-code-table)) - Downstream device in fault or disconnected

See [CN3 Safety Bus Connector](#cn3---safety-bus-connector) for signal details.


### Zero Speed Automation Mode

Alternative safe state detection without physical home switch. Machine is in safe state when all motion has stopped:

**Safe state active when:**
- All motors stopped >2 seconds (Zero Speed signal ON), AND
- Spindle stopped (Inputs/Byte0/Bit2 = 1), AND
- Guard Lock output cleared (Outputs/Byte1/Bit1 = 0)

**Jumper Configuration:**
- J10-1 and J10-4: Configure for Zero Speed operation (see [J10 section](#j10---at-home-safe-zone-detection))
- CN8 (Home connector): Not used in this mode

## Manual Override (Test Mode)
Manual override allows certain actions when guards are not safe.

The 

**Operation:** manual override capability is enabled by clearing output 11 (byte 1, bit 3). The operator then activates the physical manual override switch. The controller enters manual override mode. The software can disable manual override at any time by setting Byte 1, bit 3 to 1, providing programmatic control over when manual intervention is permitted.

When manual override is enabled, the lamp at CN15.9 illuminates.

**Requirements**
To enter manual override, all of the following conditions must be met:

- **Software Permission:** Output 11 (Byte 1, bit 3) must be cleared to 0. When set to 1, this bit inhibits manual override entry regardless of other conditions.
- **Hardware Switch Transition:** The manual override switch connected to CN15 (pins 11 and 13) must perform a valid dual-contact transition within 100ms. The contacts must change from pin 13=HIGH and pin 11=OPEN to pin 13=OPEN and pin 11=HIGH.

**Note:** The SK-2310g2 datasheet inconsistently notates this bit as "Outputs / Byte1 and Bit3" in the CN15 description (page 9), but correctly identifies it as "Outputs / Byte1 / Bit3" elsewhere in the document.

## Power Control and Monitoring
The integrated power supply delivers 24Vdc control power continuously. A separate, switched 80Vdc or 120Vdc power supply provides motive power to the drives and spindle. This supply must be enabled.

**Power State Determination**
Power state is indicated by the diagnostic code, NOT the status byte bit 3:
- **Power OFF**: Diagnostic codes 0x00-0x13 (power not available, except 0x05/0x0E special cases)
- **Ready to Power** (OFF): 0x14-0x17 (conditions met but not powered)
- **Power ON**: 0x13 (motor power supply under-voltage but ON), 0x18-0x1F (normal power-on states)
- **Special cases maintaining prior state**:
  - 0x05: Home/Test switch malfunction from guard switch fault (manual override fault causes power OFF)
  - 0x0E: Guard contact fault - maintains prior state

**Drive Power Supply Enable**
Drive power will be enabled when safety conditions are met and power button is pressed or software enables power (if J21 shorted).

**Drive Power Supply Monitoring**
When enabled, the power supply connects `loop source` to `loop input`. This loop is monitored by the SK-2310g2.


# Jumper Reference

## Safety-Critical Jumpers

| Jumper | Function | Default/Safe Position |
|--------|----------|----------------------|
| J16 | Spindle with guards open | 2-3 SHORT (spindle OFF when guards open) |
| J20 | Guards open in manual override | OPEN (spindle must stop first) |
| J10-3 | Spindle enable in manual override | OPEN (spindle disabled) |
| J10-1 | Safe state detection mode | SHORT (immediate mode) |
| J10-2 | Guard automation enable | OPEN (manual control) |
| J10-4 | Zero Speed safe state | SHORT (enable Zero Speed) |
| J19 | Guard lock/unlock control | OPEN (switch + enable) |
| J2 | Power-off delay | See J2 section |
| J14/J15 | Guard lock configuration | Must match |
| J21 | Software power-on | OPEN (button only) |
| J18 | Reserved | Leave OPEN |

## Jumper Interaction Warnings

Spindle operation with guards open (manual override mode):
- J10-3 = SHORT
- J16 = 1-2 SHORT
- J20 = SHORT

Manual Lock/Unlock buttons:
- J10-2 = SHORT
- J19 = SHORT

J14 and J15 must be set to the same position. A mismatch causes Guard 2 lock malfunction

Leave J18 OPEN - reserved for future use

---

# Configuration Recipes
TODO: Each recipe needs to be validated.

## Recipe 1: Standard Production CNC

**Use Case:** Maximum safety, guards require spindle stopped

**Jumper Configuration:**

| Jumper | Position | Function |
|--------|----------|----------|
| J10-1 | SHORT | Immediate safe state mode |
| J10-2 | OPEN | Manual unlock button |
| J10-3 | OPEN | Spindle disabled when guards open |
| J10-4 | SHORT | Enable Zero Speed |
| J16 | 2-3 SHORT | Spindle OFF when guards open |
| J17 | 2-3 SHORT | Unlock Enable output to CN13 pin 7 |
| J19 | OPEN | Unlock switch + Enable control |
| J20 | OPEN | Guards unlock only when spindle stopped |
| J21 | OPEN | Power ON button only |

**Safety Characteristics:**
- ✓ Spindle never runs with guards open
- ✓ Guards unlock only when spindle stopped
- ✓ Manual unlock button required
- ✓ Manual override has limited capabilities

---

## Recipe 2: Maintenance/Setup Mode

**Use Case:** Full access for supervised maintenance, allows spindle operation with guards open

**Jumper Configuration:**

| Jumper | Position | Function |
|--------|----------|----------|
| J10-1 | SHORT | Immediate safe state mode |
| J10-2 | OPEN | Manual unlock button |
| J10-3 | SHORT | Spindle enabled in manual override |
| J10-4 | SHORT | Enable Zero Speed |
| J16 | 1-2 SHORT | Spindle ON allowed with guards open in manual override |
| J17 | 2-3 SHORT | Unlock Enable output to CN13 pin 7 |
| J19 | OPEN | Unlock switch + Enable control |
| J20 | SHORT | Guards can open in manual override (even with spindle) |
| J21 | SHORT | Power ON via button or software |

**Safety Characteristics:**
- ⚠️ Spindle CAN run with guards open in manual override
- ⚠️ Requires manual override keyswitch + Acknowledge
- ⚠️ Return to Recipe 1 for production

---

## Recipe 3: Zero Speed Automation Mode

**Use Case:** TBD

**Jumper Configuration:**

| Jumper | Position | Function |
|--------|----------|----------|
| J10-1 | SHORT | Zero Speed mode - immediate |
| J10-2 | SHORT | Enable guard automation |
| J10-3 | OPEN | Spindle disabled when guards open |
| J10-4 | SHORT | Zero Speed mode - immediate |
| J16 | 2-3 SHORT | Spindle OFF when guards open |
| J17 | 2-3 SHORT | Unlock Enable output to CN13 pin 7 |
| J19 | SHORT | Automatic unlock via Enable output |
| J20 | OPEN | Guards unlock only when spindle stopped |
| J21 | SHORT | Power ON via button or software |

**Safety Characteristics:**
- ✓ Automatic guard unlock when all motors stopped
- ✓ No physical manual override switch required
- ✓ Spindle never runs with guards open

---

# SK-2310g2 Specific Command Reference

## Reading Status

### CMD_READ_STATUS (0x00) - Digital Inputs

**Example:**
```python
from pyldcn.devices.io import SK2310g2

# Read and decode status
status = device.read_status()
diagnostic = device.decode_diagnostic()
print(f"{diagnostic['description']} (Code: {diagnostic['code_hex']})")
```

**Command Format:**
```
[0x00, addr]
```

**Response Format:**
```
[NW, HW, Addr, Grp, stat, byte0, byte1, chk]
```

**Byte0 Digital Inputs (SK-2310g2):**

| Bit | Signal | Description | Note |
|-----|--------|-------------|------------------|
| 0 | Input1 | Digital Input 1 |
| 1 | Input2 | Digital Input 2 |
| 2 | Spindle Stopped | CN6.2 - HIGH when spindle stopped | Required for guard unlock |
| 3 | Spindle Fault | CN6.3 - General fault monitoring | Diagnostic use |
| 4 | Input3 | Digital Input 3
| 5 | Input4 | Digital Input 4
| 6 | Input5 | Digital Input 5
| 7 | Input6 | Digital Input 6

**Byte1 Internal Status (SK-2310g2):**

| Bit | Signal | Description | Note |
|-----|--------|-------------|------------------|
| 0 | Safe State | Safe zone detection | Guard unlock enabled |
| 1 | Manual Override | CN13.1-2 - Manual override active | Altered safety behavior |
| 2 | ServoFAULT | CN3.4 - Servo drive fault | Prevents power-on |
| 3-7 | Diagnostic Code | Bits [7:3] | See [Diagnostic Codes](#diagnostic-codes) |

---

### CMD_READ_OUTPUT (0x0E) - Output Status

**Example:**
```python
# Read current output states
inputs = device.read_input_states()
print(f"Spindle fault: {inputs['spindle_fault']}")
print(f"At home: {inputs['at_home']}")
```

**Command Format:**
```
[0x0E, addr]
```

**Response Format:**
```
[NW, HW, Addr, Grp, stat, byte0, byte1, chk]
```

**Byte0 Digital Outputs:**

| Bit | Signal | Description | Note |
|-----|--------|-------------|------------------|
| 0 | Output1 | Digital Output 1 |
| 1 | Output2 | Digital Output 2 |
| 2 | Spindle Command | Software spindle enable | 1 for spindle |
| 3 | Output4 (PWM) | PWM Output |
| 4 | Output5 | Digital Output 5 |
| 5 | Output6 | Digital Output 6 |
| 6 | Output7 | Digital Output 7 |
| 7 | Output8 | Digital Output 8 |

**Byte1 Internal Control:**

TODO: Determine what is meant by `Internal control`.

| Bit | Signal | Description | Note |
|-----|--------|-------------|------------------|
| 0 | Output9 | Internal control | |
| 1 | Guard Lock | 0=Unlocked, 1=Locked | Guard interlock status |
| 2 | Output11 | Internal control | |
| 3 | Output12 | Internal control | |
| 4 | Safety Link Bridge | 0=Normal, 1=Bridged | Must be 0 for spindle |
| 5 | Output14 | Internal control | |
| 6 | Output15 | Internal control | Short J19 to control guard state |
Short| 7 | Power ON Command | Software power control | Short J21 to enable |

---

### CMD_WRITE_OUTPUT (0x1E) - Control Outputs

**Example:**
```python
# Enable spindle forward
device.enable_spindle('forward')

# Set individual output bit
device.set_output_bit(OUTPUT_TOOL_CLAMP, True)
```

**Command Format:**
```
[0x1E, addr, byte0, byte1]
```

---

# Diagnostic Codes

## Status Response Format

The SK-2310g2 inherits the LS-773 status response format. When CMD_READ_STATUS (0x03) is sent with data byte `[0xFF, 0xFF]` to request all status fields, the response contains:

```
[status_byte] [byte0] [byte1] [position] [velocity] [home] [checksum]
```

### Status Byte (First Response Byte)

The status byte uses the LS-773 format:

| Bit | Name | Description |
|-----|------|-------------|
| 7-3 | (Defined) | Defined for SK-2310g2 but **NOT** used for diagnostic transmission |
| 2 | (Undefined) | Ignore |
| 1 | Checksum Error | 1 = Device detected checksum error in last command received |
| 0 | (Undefined) | Ignore |

**Important:** The diagnostic code is **NOT** transmitted in the status byte. It is transmitted in Byte1.

### Byte0 - Digital Inputs (Second Response Byte)

| Bit | Signal |
|-----|--------|
| 7 | Input 6 |
| 6 | Input 5 |
| 5 | Input 4 |
| 4 | Input 3 |
| 3 | Spindle Fault |
| 2 | Spindle Stopped |
| 1 | Input 2 |
| 0 | Input 1 |

### Byte1 - Internal Status (Third Response Byte)

| Bit | Signal | Description |
|-----|--------|-------------|
| 7-3 | **Diagnostic Code** | 5-bit diagnostic code (0x00-0x1F) displayed on LED panel |
| 2 | Servo Fault | 1 = Servo drive fault detected |
| 1 | Manual Override | 1 = Manual override mode active |
| 0 | Safe State | 1 = System in safe state |

**Diagnostic Code Extraction:**
```python
diagnostic = (byte1 >> 3) & 0x1F
```

**LED Mapping:** The 5 diagnostic bits map directly to the 5 LED indicators:
- Bit 7 → LED 5 (leftmost)
- Bit 6 → LED 4
- Bit 5 → LED 3
- Bit 4 → LED 2
- Bit 3 → LED 1 (rightmost)

**Example:** Diagnostic code 0x06 = binary `00110`
- Bit pattern: 0-0-1-1-0
- LED display: ⚫⚫🟢🟢⚫ (LEDs 3,2 ON; LEDs 5,4,1 OFF)
- Condition: "Power UP Home error"

## Diagnostic Code Table

**Encoding:** Byte1 bits [7:3] in CMD_READ_STATUS response

**LED Legend:** ⚫ = OFF, 🟢 = ON, 🟡 = Flashing

| Code | LED Pattern (5-4-3-2-1) | Condition | Power Ready† | Power A/B |
|------|-------------------------|-----------|--------------|-----------|
| 0x00 | 🟡🟡🟡🟡🟡 | Power OFF delay in progress | OFF | ON |
| 0x01 | ⚫⚫⚫⚫🟡 | Initializing | OFF | OFF |
| 0x02 | ⚫⚫⚫🟢⚫ | Control voltage shorted | OFF | OFF |
| 0x03 | ⚫⚫⚫🟢🟢 | Output shorted | OFF | OFF |
| 0x04 | ⚫⚫🟢⚫⚫ | Control voltage LOW (less than 18V) | OFF | OFF |
| 0x05 | ⚫⚫🟢⚫🟢 | Home/Test switch malfunction (both contacts ON) | Prior state | Prior state |
| 0x06 | ⚫⚫🟢🟢⚫ | Power UP Home error | OFF | OFF |
| 0x07 | ⚫⚫🟢🟢🟢 | Power UP manual override error | OFF | OFF |
| 0x08 | ⚫🟢⚫⚫⚫ | System LOCKED | OFF | OFF |
| 0x09 | ⚫🟢⚫⚫🟢 | Watchdog Stop | OFF | OFF |
| 0x0A | ⚫🟢⚫🟢⚫ | Safety Link Error | OFF | OFF |
| 0x0B | ⚫🟢⚫🟢🟢 | Guard Open Stop - Guards open, spindle not stopped (contacts OK) | OFF | OFF |
| 0x0B | ⚫🟡⚫🟡🟡 | Guard Open Stop - Guards open, spindle not stopped (contact fault) | OFF | OFF |
| 0x0C | ⚫🟢🟢⚫⚫ | Guard Open Stop - Guards open, not in safe zone (contacts OK) | OFF | OFF |
| 0x0C | ⚫🟡🟡⚫⚫ | Guard Open Stop - Guards open, not in safe zone (contact fault) | OFF | OFF |
| 0x0D | ⚫🟢🟢⚫🟢 | Guard Open Stop - Manual override without Enable button held (contacts OK) | OFF | OFF |
| 0x0D | ⚫🟡🟡⚫🟡 | Guard Open Stop - Manual override without Enable button held (contact fault) | OFF | OFF |
| 0x0E | ⚫🟡🟡🟡⚫ | Guard contact fault (one or more contacts malfunctioning) | Prior state | Prior state |
| 0x0F | ⚫🟢🟢🟢🟢 | Limit Switch Stop | OFF | OFF |
| 0x10 | 🟢⚫⚫⚫⚫ | Emergency Stop | OFF | OFF |
| 0x11 | 🟢⚫⚫⚫🟢 | Emergency Stop contact fault or Monitor Loop Open | OFF | OFF |
| 0x12 | 🟢⚫⚫🟢⚫ | Busy (≤6s) or Power button short/Monitor Loop Open (>6s) | OFF | OFF |
| 0x13 | 🟢⚫⚫🟢🟢 | Motor Power Supply under-voltage | ON | ON |
| 0x14 | 🟢⚫🟢⚫⚫ | Guard-1 Open; Guard-2 Open (ready to power) | OFF | OFF |
| 0x15 | 🟢⚫🟢⚫🟢 | Guard-1 Closed; Guard-2 Open (ready to power) | OFF | OFF |
| 0x16 | 🟢⚫🟢🟢⚫ | Guard-1 Open; Guard-2 Closed (ready to power) | OFF | OFF |
| 0x17 | 🟢⚫🟢🟢🟢 | Guard-1 Closed; Guard-2 Closed (ready to power) | OFF | OFF |
| 0x18 | 🟢🟢⚫⚫⚫ | Guard-1 Open; Guard-2 Open; Manual override | ON | ON |
| 0x19 | 🟢🟢⚫⚫🟢 | Guard-1 Closed; Guard-2 Open; Manual override | ON | ON |
| 0x1A | 🟢🟢⚫🟢⚫ | Guard-1 Open; Guard-2 Closed; Manual override | ON | ON |
| 0x1B | 🟢🟢⚫🟢🟢 | Guard-1 Closed; Guard-2 Closed; Manual override | ON | ON |
| 0x1C | 🟢🟢🟢⚫⚫ | Guard-1 Open; Guard-2 Open; Safe zone; Spindle stopped | ON | ON |
| 0x1D | 🟢🟢🟢⚫🟢 | Guard-1 Closed; Guard-2 Open; Safe zone; Spindle stopped | ON | ON |
| 0x1E | 🟢🟢🟢🟢⚫ | Guard-1 Open; Guard-2 Closed; Safe zone; Spindle stopped | ON | ON |
| 0x1F | 🟢🟢🟢🟢🟢 | **Normal operation** - All guards closed | ON | ON |

† **Power Ready:** Indicates if power enable signal allows power-on. The action to enable Power On depends is configured by [J21](#j21---power-on-control-method).

---
# Troubleshooting
## Symptom: Power Button Not Flashing (Won't Power On)

**Common Causes:**

| Diagnostic Code | Cause | Resolution |
|----------------|-------|------------|
| 0x1F | ServoFAULT active | Check servo drives, clear faults |
| 0x06 | Home sensor invalid at power-up | Position machine to valid home state |
| 0x0A | Safety Link broken | Check CN3.2 safety chain |
| 0x05 | Home/Test switch malfunction | Check CN8 or CN13 wiring |

---

## Symptom: Diagnostic 0x0A (Safety Link Error)

**Meaning:** Safety Link IN (CN3.2) is LOW, indicating broken safety chain

**Diagnostic Steps:**

1. **Check downstream devices:**
   - Verify all Safety Bus devices powered on
   - Check wiring continuity on Safety Link chain

2. **Read Safety Bus status:**
   ```python
   status = read_status(addr)
   # Check if any slave devices reporting fault
   ```

3. **Isolate fault:**
   - Disconnect downstream devices one by one
   - Re-test after each disconnection
   - Fault clears when bad device removed

**Common Causes:**
- Slave device powered off
- Broken wire in Safety Link chain
- Slave device in fault state
- E-stop activated on slave device

---

## Symptom: Spindle Won't Start (Diagnostic 0x1F)

**Meaning:** ServoFAULT prevents spindle operation

**Diagnostic Command Sequence:**

```python
# 1. Check ServoFAULT input
status = read_status(addr)
byte1 = status[6]
servo_fault = (byte1 & 0x04) != 0

if servo_fault:
    print("ServoFAULT active (CN3.4 HIGH)")
    # Check servo drive status

# 2. Check spindle command output
outputs = read_output(addr)
byte0 = outputs[5]
byte1 = outputs[6]
spindle_cmd = (byte0 & 0x04) != 0
safety_bridge = (byte1 & 0x10) != 0

print(f"Spindle Command: {spindle_cmd}")
print(f"Safety Link Bridge: {safety_bridge} (must be 0)")
```

**Required Conditions for Spindle Enable:**
1. Outputs/Byte0/Bit2 = 1 (spindle command)
2. Outputs/Byte1/Bit4 = 0 (Safety Link Bridge = 0)
3. Inputs/Byte1/Bit2 = 0 (ServoFAULT = 0)
4. Power = ON
5. Guards CLOSED (unless J16=1-2 and manual override active)

---

## Symptom: Guards Won't Unlock

**Diagnostic Command Sequence:**

```python
# 1. Check safe state status
status = read_status(addr)
byte0 = status[5]
byte1 = status[6]
safe_state = (byte1 & 0x01) != 0
spindle_stopped = (byte0 & 0x04) != 0
manual_override = (byte1 & 0x02) != 0

print(f"Safe State: {safe_state}")
print(f"Spindle Stopped: {spindle_stopped}")
print(f"Manual Override: {manual_override}")

# 2. Check guard lock output
outputs = read_output(addr)
byte1_out = outputs[6]
guard_lock = (byte1_out & 0x02) != 0

print(f"Guard Lock Output: {guard_lock} (1=locked, 0=unlocked)")
```

**Required Conditions for Guard Unlock:**

**Normal Operation:**
- Safe State = 1
- Spindle Stopped = 1
- Power OFF or in safe state

**Manual Override (J20=OPEN):**
- Manual Override = 1
- Acknowledge pressed (CN13.3-4)
- Spindle Stopped = 1

**Manual Override (J20=SHORT):**
- Manual Override = 1
- Acknowledge pressed (CN13.3-4)
- (Spindle state irrelevant)

**Check Jumper Configuration:**
- J10-1, J10-2, J10-4: Safe state detection settings
- J19: Guard control mode (OPEN or SHORT)
- J20: Manual override guard unlock requirements

---
# Connectors and Jumpers

## CN3 - Safety Bus Connector

| Pin | Signal | Type | Description | Related Jumpers |
|-----|--------|------|-------------|-----------------|
| 1 | Safety Link | Output | Generates based on guards/e-stop/safe state | J10, J16, J20 |
| 2 | Safety Link | Input | Monitors daisy chain integrity | (Diagnostic 0x0A if LOW) |
| 3 | Enable/Stop | Output | HIGH when power ON | (J2 power-off delay) |
| 4 | ServoFAULT | Input | (Inputs/Byte1/Bit2) - Prevents power-on if HIGH | (Diagnostic 0x00 if HIGH) |

## CN6 - Spindle Interface

| Pin | Signal | Type | Description | Related Jumpers |
|-----|--------|------|-------------|-----------------|
| 2 | Spindle Stopped | Input | (Inputs/Byte0/Bit2) - Required for spindle-dependent safety | J10, J20 |
| 3 | Spindle Fault | Input | (Inputs/Byte0/Bit3) - General purpose fault monitoring | - |
| 5 | Spindle Enable | Output | Same as CN16.10 | J16, J10-3, J20 |

## CN8 - Safe Zone Sensor

| Pin | Signal | Type | Description |
|-----|--------|------|-------------|
| 1 | Home A1 | Input | Home sensor contact A, pin 1. Closed in Safety Zone |
| 2 | Home A2 | Input | Home sensor contact A, pin 2 |
| 3 | Home B1 | Input | Home sensor contact B, pin 1. Open in Safety Zone |
| 4 | Home B2 | Input | Home sensor contact B, pin 2 |

## CN9, CN10 - Guard Switches (Guard 1, Guard 2)

| Pin | Signal | Type | Description |
|-----|--------|------|-------------|
| 1-2 | Guard Closed (A) | Input | Guard position sensor contact A |
| 3-4 | Guard Unlock | Output | Unlock solenoid |
| 5-6 | Guard Closed (B) | Input | Guard position sensor contact B |

## CN13 - Manual Override and Unlock Control

| Pin | Signal | Type | Description | Related Jumpers |
|-----|--------|------|-------------|-----------------|
| 1-2 | Manual Override Switch | Input | (Inputs/Byte1/Bit1) - Must be keyswitch | J10-3, J16, J20 |
| 3-4 | Acknowledge Button | Input | Dual-contact enable - must be held during operation | J10-3, J16, J20 |
| 5-6 | Unlock Switch | Input | Manual guard unlock button | J19 |
| 7 | Unlock Enable | Output | Automatic unlock control | J17, J19 |

## CN16 - Power Control Connector

| Pin | Signal | Type | Description | Related Jumpers |
|-----|--------|------|-------------|-----------------|
| 5 | Monitor Loop (-) | Input | Relay contact monitor return | J5 routing |
| 6 | Monitor Loop (+) | Output | Relay contact monitor source | J5 routing |
| 8 | Power Enable | Output | Main power contactor control | J5 routing |
| 10 | Spindle ON | Output | Spindle enable output (wired to CN6.5) | J16, J10-3, J20 |

For complete connector pinouts, see SK-2310g2 Manual pages 6-10.


# Related Documentation

**pyldcn Documentation:**
- [io_commands.md](io_commands.md) - I/O hardware specifications (digital/analog I/O, PWM, counter/timer)
- [safety_bus.md](safety_bus.md) - Generic Safety Bus protocol specification
- [ldcn_protocol.md](ldcn_protocol.md) - Generic LDCN network commands

**SK-2310g2 Manual Reference:**
- Page 5: Jumper configuration summary
- Pages 6-10: Complete connector pinouts
- Pages 11-18: Sample application wiring diagrams
- Page 19: Digital I/O table
- Page 20: Diagnostic code table
- Pages 21-22: Zero Speed automation mode
- Pages 23-24: Distribution boards

---

# Safety Warnings

1. ⚠️ **Do not bypass safety interlocks** - Hardware interlocks are fail-safe and must not be defeated

2. ⚠️ **Manual override is for supervised maintenance only** - Return to production configuration after maintenance

3. ⚠️ **Jumper J18 must remain OPEN** - Reserved for future use, improper configuration may cause malfunction

4. ⚠️ **J14 and J15 must match** - Mismatch causes Guard 2 lock malfunction

5. ⚠️ **Regular testing required** - Test safety functions periodically per [Testing Procedures](#testing-procedures)

6. ⚠️ **Qualified personnel only** - Installation and maintenance by trained technicians only

---

# References

- CNC-SK-2310g2 Supervisor I/O Controller Manual, Doc # 710231005 / Rev. D, 03/05/2020
- LS-2310g2-Supervisor-IO-Controller.pdf (24-page official manual)
- LDCN Protocol Specification
- Safety Bus Protocol Specification
