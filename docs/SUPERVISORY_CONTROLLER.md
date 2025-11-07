# SK-2310g2 Supervisory Controller Guide

**Author:** NickyDoes
**Source:** CNC-SK-2310g2 Manual, Doc # 710231005 / Rev. D, 03/05/2020

---

## Document Purpose

This document covers:
- Safety system architecture and jumper configuration
- Application-specific wiring and integration
- Diagnostic codes and troubleshooting
- LDCN command reference for safety monitoring

**For I/O hardware specifications** (digital/analog I/O, PWM, counter/timer), see [io_commands.md](io_commands.md).

**Prerequisites:** Read [safety_bus.md](safety_bus.md) for generic Safety Bus specification.

**Terminology Note:** The Logosol documentation uses non-standard terminology:
- **"Cover"** = Safety guard/guarding (dual-zone guarding with interlocked access doors)
- **"At Home"** = Machine in safe state (guards closed, drives disabled/stationary, safe for power-off or guard access)
- **"Test Mode"** = Manual override mode (key-switch inhibited, allows limited guarded operations with ACK button held)


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
- [Wiring Examples](#wiring-examples)
- [Detailed Jumper Configuration](#detailed-jumper-configuration)
- [Connector Pinouts](#connector-pinouts)

---

# Safety System Overview

## Role as Safety Master

The SK-2310g2 serves as the **safety system coordinator** in LDCN-based CNC systems:

**Core Safety Functions:**
1. **Emergency Stop Monitoring** - 5 dual-contact e-stop connections
2. **Guard Interlock Management** - Two dual-zone safety guards with lock/unlock control
3. **Safe State Detection** - "At Home" (guards closed, drives stationary)
4. **Manual Override Control** - Key-switch inhibited manual mode with ACK button enable
5. **Safety Link Generation** - Master Safety Link OUT based on all conditions
6. **ServoFAULT Detection** - Monitors all motor drives for faults
7. **Power Supply Control** - Relay-based contactor power enable with monitoring loop
8. **Spindle Interlocks** - Prevents spindle operation in unsafe conditions
9. **Diagnostic Reporting** - Diagnostic codes and LEDs indicate system state

---

## At Home (Safe State) Concept

**"At Home"** indicates the machine is in a **safe state** where guards can be unlocked for operator access. This requires drives disabled/stationary, and optionally the machine positioned at a safe location.

The SK-2310g2 uses **dual-contact safety sensing** to determine safe state:

### Standard Home Sensor Mode

**Dual-Contact Safety Switch:**
- **Contact A (Normally Closed)**: Closed when machine is in Safety Zone
- **Contact B (Normally Open)**: Open when machine is in Safety Zone

**Valid States:**

| Machine Position | Contact A | Contact B | At Home Status |
|------------------|-----------|-----------|----------------|
| In Safety Zone | CLOSED | OPEN | 1 (TRUE) |
| Outside Safety Zone | OPEN | CLOSED | 0 (FALSE) |

**Critical Timing Requirement:**
- Transition time between contacts must be **<100msec**
- Exceeding this triggers home sensor fault (Diagnostic 0x05)

**Fault Conditions:**
- Both contacts ON simultaneously (Diagnostic 0x05)
- Timing violation >100msec (Diagnostic 0x05)
- Invalid state at power-up (Diagnostic 0x06)

### Zero Speed Automation Mode

Alternative safe state detection without physical home switch. Machine is "At Home" (safe state) when all motion has stopped:

**At Home = 1 when:**
- All motors stopped >2 seconds (Zero Speed signal ON), AND
- Spindle stopped (Inputs/Byte0/Bit2 = 1), AND
- Guard Lock output cleared (Outputs/Byte1/Bit1 = 0)

**Jumper Configuration:**
- J10-1 and J10-4: Configure for Zero Speed operation (see [J10 section](#j10---at-home-safe-zone-detection))
- CN8 (Home connector): Not used in this mode

---
# Jumper Quick Reference

## Safety-Critical Jumpers

| Jumper | Function | Safety Impact | Default/Safe Position |
|--------|----------|---------------|----------------------|
| **J16** | Spindle with guards open | 🔴 CRITICAL | 2-3 SHORT (spindle OFF when guards open) |
| **J20** | Guards open in manual override | 🔴 CRITICAL | OPEN (spindle must stop first) |
| **J10-3** | Spindle enable in manual override | 🔴 CRITICAL | OPEN (spindle disabled) |
| **J10-1** | At Home (safe state) detection mode | 🟡 HIGH | SHORT (immediate mode) |
| **J10-2** | Guard automation enable | 🟡 HIGH | OPEN (manual control) |
| **J10-4** | Zero Speed safe state | 🟡 HIGH | SHORT (enable Zero Speed) |
| **J19** | Guard lock/unlock control | 🟡 HIGH | OPEN (switch + enable) |
| J2 | Power-off delay | 🟢 MEDIUM | See J2 section |
| J14/J15 | Guard 2 lock polarity | 🟢 MEDIUM | Must match |
| J21 | Software power-on | 🟢 LOW | OPEN (button only) |
| **J18** | Reserved | 🔴 CRITICAL | **MUST REMAIN OPEN** |

## Jumper Interaction Warnings

⚠️ **Spindle operation with guards open** (manual override mode) requires THREE jumpers:
- J10-3 = SHORT
- J16 = 1-2 SHORT
- J20 = SHORT

⚠️ **Manual Lock/Unlock buttons** requires TWO jumpers:
- J10-2 = SHORT
- J19 = SHORT

⚠️ **J14 and J15 must be set to the same position** - mismatch causes Guard 2 lock malfunction

⚠️ **J18 must remain OPEN** - reserved for future use

---

# Configuration Recipes

## Recipe 1: Standard Production CNC

**Use Case:** Maximum safety, covers require spindle stopped

**Jumper Configuration:**

| Jumper | Position | Function |
|--------|----------|----------|
| J10-1 | SHORT | Immediate At Home mode |
| J10-2 | OPEN | Manual unlock button |
| J10-3 | OPEN | Spindle disabled when covers open |
| J10-4 | SHORT | Enable Zero Speed |
| J16 | 2-3 SHORT | Spindle OFF when covers open |
| J17 | 2-3 SHORT | Unlock Enable output to CN13 pin 7 |
| J19 | OPEN | Unlock switch + Enable control |
| J20 | OPEN | Covers unlock only when spindle stopped |
| J21 | OPEN | Power ON button only |

**Safety Characteristics:**
- ✓ Spindle never runs with covers open
- ✓ Covers unlock only when spindle stopped
- ✓ Manual unlock button required
- ✓ Test Mode has limited capabilities

---

## Recipe 2: Maintenance/Setup Mode

**Use Case:** Full access for supervised maintenance, allows spindle with covers open

**Jumper Configuration:**

| Jumper | Position | Function |
|--------|----------|----------|
| J10-1 | SHORT | Immediate At Home mode |
| J10-2 | OPEN | Manual unlock button |
| J10-3 | **SHORT** | Spindle enabled in Test Mode |
| J10-4 | SHORT | Enable Zero Speed |
| J16 | **1-2 SHORT** | Spindle ON allowed with covers open in Test Mode |
| J17 | 2-3 SHORT | Unlock Enable output to CN13 pin 7 |
| J19 | OPEN | Unlock switch + Enable control |
| J20 | **SHORT** | Covers can open in Test Mode (even with spindle) |
| J21 | SHORT | Power ON via button or software |

**Safety Characteristics:**
- ⚠️ Spindle CAN run with covers open in Test Mode
- ⚠️ Requires Test Mode keyswitch + Acknowledge (dual-action)
- ⚠️ Use only for supervised maintenance
- ⚠️ Return to Recipe 1 for production

---

## Recipe 3: Zero Speed Automation Mode

**Use Case:** Automated systems without fixed home position

**Jumper Configuration:**

| Jumper | Position | Function |
|--------|----------|----------|
| J10-1 | SHORT | Zero Speed mode - immediate |
| J10-2 | SHORT | Enable cover automation |
| J10-3 | OPEN | Spindle disabled when covers open |
| J10-4 | SHORT | Zero Speed mode - immediate |
| J16 | 2-3 SHORT | Spindle OFF when covers open |
| J17 | 2-3 SHORT | Unlock Enable output to CN13 pin 7 |
| J19 | SHORT | Automatic unlock via Enable output |
| J20 | OPEN | Covers unlock only when spindle stopped |
| J21 | SHORT | Power ON via button or software |

**Safety Characteristics:**
- ✓ Automatic cover unlock when all motors stopped
- ✓ No physical home switch required
- ✓ Software power control enabled
- ✓ Spindle never runs with covers open

---

# LDCN Command Reference

## Reading Safety Status

### CMD_READ_STATUS (0x00) - Digital Inputs

**Command Format:**
```
[0x00, addr]
```

**Response Format:**
```
[NW, HW, Addr, Grp, stat, byte0, byte1, chk]
```

**Byte0 Digital Inputs (SK-2310g2):**

| Bit | Signal | Description | Safety Relevance |
|-----|--------|-------------|------------------|
| 0 | Input1 | Digital Input 1 | Application-specific |
| 1 | Input2 | Digital Input 2 | Application-specific |
| 2 | **Spindle Stopped** | CN6.2 - HIGH when spindle stopped | Required for cover unlock |
| 3 | Spindle Fault | CN6.3 - General fault monitoring | Diagnostic use |
| 4 | Input3 | Digital Input 3 | Application-specific |
| 5 | Input4 | Digital Input 4 | Application-specific |
| 6 | Input5 | Digital Input 5 | Application-specific |
| 7 | Input6 | Digital Input 6 | Application-specific |

**Byte1 Internal Status (SK-2310g2):**

| Bit | Signal | Description | Safety Relevance |
|-----|--------|-------------|------------------|
| 0 | **At Home** | Safe zone detection | Cover unlock enabled |
| 1 | **Test Mode** | CN13.1-2 - Test mode active | Altered safety behavior |
| 2 | **ServoFAULT** | CN3.4 - Servo drive fault | Prevents power-on |
| 3-7 | **Diagnostic Code** | Bits [7:3] = diagnostic code | See [Diagnostic Codes](#diagnostic-codes) |

**Example - Pre-Operation Safety Check:**
```python
status = read_status(sk2310g2_addr)
byte0 = status[5]
byte1 = status[6]

spindle_stopped = (byte0 & 0x04) != 0  # Bit 2
at_home = (byte1 & 0x01) != 0          # Bit 0
test_mode = (byte1 & 0x02) != 0        # Bit 1
servo_fault = (byte1 & 0x04) != 0      # Bit 2
diagnostic_code = (byte1 >> 3) & 0x1F  # Bits [7:3]

if servo_fault:
    raise SafetyError("ServoFAULT detected")
if diagnostic_code != 0x00:
    raise SafetyError(f"Diagnostic code: 0x{diagnostic_code:02X}")
```

---

### CMD_READ_OUTPUT (0x0E) - Output Status

**Command Format:**
```
[0x0E, addr]
```

**Response Format:**
```
[NW, HW, Addr, Grp, stat, byte0, byte1, chk]
```

**Byte0 Digital Outputs:**

| Bit | Signal | Description | Safety Relevance |
|-----|--------|-------------|------------------|
| 0 | Output1 | Digital Output 1 | Application-specific |
| 1 | Output2 | Digital Output 2 | Application-specific |
| 2 | **Spindle Command** | Software spindle enable | Must be 1 for spindle |
| 3 | Output4 (PWM) | PWM Output | Application-specific |
| 4 | Output5 | Digital Output 5 | Application-specific |
| 5 | Output6 | Digital Output 6 | Application-specific |
| 6 | Output7 | Digital Output 7 | Application-specific |
| 7 | Output8 | Digital Output 8 | Application-specific |

**Byte1 Internal Control:**

| Bit | Signal | Description | Safety Relevance |
|-----|--------|-------------|------------------|
| 0 | Output9 | Internal control | Application-specific |
| 1 | **Cover Lock** | 0=Unlocked, 1=Locked | Cover interlock status |
| 2 | Output11 | Internal control | Application-specific |
| 3 | Output12 | Internal control | Application-specific |
| 4 | **Safety Link Bridge** | 0=Normal, 1=Bridged | Must be 0 for spindle |
| 5 | Output14 | Internal control | Application-specific |
| 6 | Output15 | Internal control | Can control covers if J19=SHORT |
| 7 | **Power ON Command** | Software power control | Requires J21=SHORT |

---

### CMD_WRITE_OUTPUT (0x1E) - Control Outputs

**Command Format:**
```
[0x1E, addr, byte0, byte1]
```

**Example - Unlock Covers:**
```python
# Read current outputs
outputs = read_output(sk2310g2_addr)
byte0 = outputs[5]
byte1 = outputs[6]

# Clear Cover Lock bit (Byte1 Bit1)
byte1 &= ~0x02

# Write updated outputs
write_output(sk2310g2_addr, byte0, byte1)
```

**Example - Command Spindle:**
```python
# Set Spindle Command bit (Byte0 Bit2)
byte0 |= 0x04

# Ensure Safety Link Bridge is 0 (Byte1 Bit4)
byte1 &= ~0x10

write_output(sk2310g2_addr, byte0, byte1)
```

**Safety Warnings:**
- ⚠️ Always read outputs before writing to preserve other bits
- ⚠️ Verify safety conditions met before commanding spindle or unlocking covers
- ⚠️ Hardware interlocks override software commands

---

## Safety Monitoring Sequences

### Pre-Operation Safety Check

```python
def pre_operation_check(addr):
    """Verify all safety conditions before starting operation"""

    # Read status
    status = read_status(addr)
    byte0 = status[5]
    byte1 = status[6]

    # Extract safety signals
    spindle_stopped = (byte0 & 0x04) != 0
    at_home = (byte1 & 0x01) != 0
    test_mode = (byte1 & 0x02) != 0
    servo_fault = (byte1 & 0x04) != 0
    diagnostic_code = (byte1 >> 3) & 0x1F

    # Check for faults
    if servo_fault:
        raise SafetyError("ServoFAULT active")

    if diagnostic_code != 0x00:
        raise SafetyError(f"Diagnostic code 0x{diagnostic_code:02X}")

    # Verify covers closed
    # (Read cover switch inputs - application-specific)

    # Verify emergency stops OK
    # (Read e-stop chain status - application-specific)

    return True
```

### Continuous Safety Monitoring

```python
def safety_monitor_loop(addr, interval_ms=100):
    """Continuous monitoring of safety status"""

    while True:
        status = read_status(addr)
        byte1 = status[6]

        diagnostic_code = (byte1 >> 3) & 0x1F

        if diagnostic_code != 0x00:
            # Handle diagnostic code
            handle_diagnostic(diagnostic_code)

        time.sleep(interval_ms / 1000.0)
```

---

# Diagnostic Codes

## Diagnostic Code Table

**Encoding:** Byte1 bits [7:3] in CMD_READ_STATUS response

| Code | Binary | LED Pattern (5-4-3-2-1) | Condition | Power Enable | Power A/B |
|------|--------|-------------------------|-----------|--------------|-----------|
| 0x00 | 00000 | OFF-OFF-OFF-OFF-OFF | **Normal operation** | ON | ON |
| 0x01 | 00001 | OFF-OFF-OFF-OFF-ON | Low voltage fault | Prior state | Prior state |
| 0x02 | 00010 | OFF-OFF-OFF-ON-OFF | High voltage fault | Prior state | Prior state |
| 0x03 | 00011 | OFF-OFF-OFF-ON-ON | Over temperature | Prior state | Prior state |
| 0x04 | 00100 | OFF-OFF-ON-OFF-OFF | Safety bus fault | Prior state | Prior state |
| 0x05 | 00101 | OFF-OFF-ON-OFF-ON | Home/Test switch malfunction (both ON) | Prior state | Prior state |
| 0x06 | 00110 | OFF-OFF-ON-ON-OFF | Power UP Home error | OFF | OFF |
| 0x0A | 01010 | OFF-ON-OFF-ON-OFF | Safety Link broken | OFF | OFF |
| 0x1F | 11111 | ON-ON-ON-ON-ON | ServoFAULT active | OFF | OFF |

**Complete diagnostic code table:** See SK-2310g2 Manual page 20.

---

## Diagnostic Bit Decoding

```python
def decode_diagnostic(byte1):
    """Extract and decode diagnostic code from status byte1"""

    diagnostic_code = (byte1 >> 3) & 0x1F

    diagnostic_messages = {
        0x00: "Normal operation",
        0x01: "Low voltage fault",
        0x02: "High voltage fault",
        0x03: "Over temperature",
        0x04: "Safety bus fault",
        0x05: "Home/Test switch malfunction (both contacts ON)",
        0x06: "Power UP Home error (invalid home state at startup)",
        0x0A: "Safety Link broken (CN3.2 LOW)",
        0x1F: "ServoFAULT active (CN3.4 HIGH)",
    }

    return diagnostic_code, diagnostic_messages.get(diagnostic_code, "Unknown diagnostic code")
```

---

# Troubleshooting by Symptom

## Symptom: Power Button Not Flashing (Won't Power On)

**Diagnostic Command Sequence:**

```python
# 1. Read status
status = read_status(addr)
byte1 = status[6]
diagnostic_code = (byte1 >> 3) & 0x1F
servo_fault = (byte1 & 0x04) != 0

# 2. Check diagnostic code
if diagnostic_code != 0x00:
    print(f"Diagnostic code: 0x{diagnostic_code:02X}")
    # See diagnostic code table

# 3. Check ServoFAULT
if servo_fault:
    print("ServoFAULT active (CN3.4 HIGH)")
    print("Check servo drive status")
```

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
5. Covers CLOSED (unless J16=1-2 and Test Mode)

---

## Symptom: Covers Won't Unlock

**Diagnostic Command Sequence:**

```python
# 1. Check At Home status
status = read_status(addr)
byte0 = status[5]
byte1 = status[6]
at_home = (byte1 & 0x01) != 0
spindle_stopped = (byte0 & 0x04) != 0
test_mode = (byte1 & 0x02) != 0

print(f"At Home: {at_home}")
print(f"Spindle Stopped: {spindle_stopped}")
print(f"Test Mode: {test_mode}")

# 2. Check cover lock output
outputs = read_output(addr)
byte1_out = outputs[6]
cover_lock = (byte1_out & 0x02) != 0

print(f"Cover Lock Output: {cover_lock} (1=locked, 0=unlocked)")
```

**Required Conditions for Cover Unlock:**

**Normal Operation:**
- At Home = 1
- Spindle Stopped = 1
- Power OFF or At Home

**Test Mode (J20=OPEN):**
- Test Mode = 1
- Acknowledge pressed (CN13.3-4)
- Spindle Stopped = 1

**Test Mode (J20=SHORT):**
- Test Mode = 1
- Acknowledge pressed (CN13.3-4)
- (Spindle state irrelevant)

**Check Jumper Configuration:**
- J10-1, J10-2, J10-4: At Home detection settings
- J19: Cover control mode (OPEN or SHORT)
- J20: Test mode cover unlock requirements

---

# Testing Procedures

## Initial Commissioning Test Sequence

### 1. Power-On Test

**Objective:** Verify basic power and communication

**Procedure:**
```python
# 1.1 Apply 24V power to SK-2310g2
# 1.2 Verify LED indicators
# 1.3 Test LDCN communication
status = read_status(sk2310g2_addr)
print(f"Status received: {status}")

# 1.4 Check diagnostic code
byte1 = status[6]
diagnostic = (byte1 >> 3) & 0x1F
print(f"Diagnostic code: 0x{diagnostic:02X}")

# Expected: 0x00 (normal) or 0x06 (home sensor not valid)
```

### 2. Home Sensor Test

**Objective:** Verify dual-contact home sensor operation and timing

**Procedure:**
```python
# 2.1 Position machine outside home
status = read_status(addr)
at_home = (status[6] & 0x01) != 0
assert not at_home, "At Home should be 0"

# 2.2 Move to home position slowly
# Monitor for diagnostic 0x05 (timing violation)

# 2.3 Verify At Home = 1
status = read_status(addr)
at_home = (status[6] & 0x01) != 0
assert at_home, "At Home should be 1"

# 2.4 Move away from home
status = read_status(addr)
at_home = (status[6] & 0x01) != 0
assert not at_home, "At Home should be 0"
```

**Expected Results:**
- At Home follows machine position
- No diagnostic 0x05 (timing fault)
- Transition time <100msec

### 3. Cover Interlock Test

**Objective:** Verify cover lock/unlock operation and interlocks

**Procedure:**
```python
# 3.1 Verify covers locked when not At Home
outputs = read_output(addr)
cover_lock = (outputs[6] & 0x02) != 0
assert cover_lock, "Covers should be locked"

# 3.2 Move to home position
# (At Home should become 1)

# 3.3 Stop spindle
status = read_status(addr)
spindle_stopped = (status[5] & 0x04) != 0
assert spindle_stopped, "Spindle should be stopped"

# 3.4 Command cover unlock
outputs = read_output(addr)
byte0 = outputs[5]
byte1 = outputs[6]
byte1 &= ~0x02  # Clear Cover Lock bit
write_output(addr, byte0, byte1)

# 3.5 Verify covers unlocked
outputs = read_output(addr)
cover_lock = (outputs[6] & 0x02) != 0
assert not cover_lock, "Covers should be unlocked"

# 3.6 Open covers manually
# 3.7 Verify Safety Link OUT = HIGH (covers open but At Home + spindle stopped)
```

### 4. Spindle Interlock Test

**Objective:** Verify spindle cannot run with covers open (J16=2-3)

**Procedure:**
```python
# 4.1 Unlock and open covers (from step 3)

# 4.2 Command spindle ON
outputs = read_output(addr)
byte0 = outputs[5]
byte1 = outputs[6]
byte0 |= 0x04    # Set Spindle Command bit
byte1 &= ~0x10   # Clear Safety Link Bridge
write_output(addr, byte0, byte1)

# 4.3 Verify spindle does NOT start
# (Hardware interlock prevents Spindle ON output)

# 4.4 Close covers

# 4.5 Verify spindle NOW starts
# (Spindle ON output should go HIGH)

# 4.6 Stop spindle
byte0 &= ~0x04
write_output(addr, byte0, byte1)
```

**Expected Results:**
- Spindle does NOT run with covers open (J16=2-3)
- Spindle runs when covers closed
- Hardware interlock overrides software command

### 5. Test Mode Access Test

**Objective:** Verify test mode operation and interlocks

**Procedure:**
```python
# 5.1 Close covers, lock, move away from home

# 5.2 Activate Test Mode keyswitch (CN13.1-2)
status = read_status(addr)
test_mode = (status[6] & 0x02) != 0
assert test_mode, "Test Mode should be active"

# 5.3 Press Acknowledge button (CN13.3-4)

# 5.4 Stop spindle

# 5.5 Verify covers can unlock
# (J20=OPEN requires spindle stopped)

# 5.6 If J16=1-2 and J10-3=SHORT:
#     Command spindle with covers open
#     Verify spindle DOES run (DANGEROUS - supervised only)
```

### 6. Safety Link Chain Test

**Objective:** Verify Safety Link OUT generation and daisy chain

**Procedure:**
```python
# 6.1 Close covers, At Home, spindle stopped, power ON
# 6.2 Verify Safety Link OUT (CN3.1) = HIGH

# 6.3 Open e-stop on downstream device
# 6.4 Verify Safety Link IN (CN3.2) = LOW
# 6.5 Verify diagnostic code = 0x0A

# 6.6 Close e-stop
# 6.7 Verify diagnostic code returns to 0x00
```

### 7. ServoFAULT Test

**Objective:** Verify servo fault detection and response

**Procedure:**
```python
# 7.1 Trigger fault on servo drive
# 7.2 Verify CN3.4 (ServoFAULT) goes HIGH

# 7.3 Read status
status = read_status(addr)
servo_fault = (status[6] & 0x04) != 0
assert servo_fault, "ServoFAULT should be active"

# 7.4 Verify diagnostic code = 0x1F

# 7.5 Clear servo fault
# 7.6 Verify diagnostic code returns to 0x00
```

---

## Periodic Maintenance Testing

# Wiring Examples

## Safety Home Switch Wiring

**Dual-Contact Safety Switch Connection to CN8:**

```
Safety Switch            CN8 Connector
--------------          --------------
Contact A1    -------->  Pin 1 (Home A1)
Contact A2    -------->  Pin 2 (Home A2)
Contact B1    -------->  Pin 3 (Home B1)
Contact B2    -------->  Pin 4 (Home B2)
```

**Switch States:**

| Machine Position | Contact A | Contact B |
|------------------|-----------|-----------|
| In Safety Zone | CLOSED | OPEN |
| Outside Safety Zone | OPEN | CLOSED |

**Design Recommendations:**

1. **Switch Selection:**
   - Automation-grade safety switches
   - Positive opening action (direct break)
   - Contact rating: 40Vdc, 0.5A minimum
   - Mechanical life: >1 million operations

2. **Installation:**
   - Use shielded cable (<10m typical)
   - Route away from high-current motor cables
   - Ground shield at controller end only

3. **Optional H1 Delay Circuit:**
   - Add if switch contact bounce observed
   - Provides debouncing and timing validation
   - Recommended for cable runs >5 meters

See SK-2310g2 Manual page 13 for complete wiring diagram.

---

## Cover Switch and Lock Wiring

**Dual-Contact Cover Switch + Lock Solenoid:**

```
Cover 1: CN9 connector
Cover 2: CN10 connector

Pins 1-4: Dual-contact cover position sensor
Pins 5-6: Lock solenoid outputs
```

**Polarity Selection (Cover 2 only):**
- J14/J15 = 1-2 SHORT: Power when unlocked (spring-return lock)
- J14/J15 = 2-3 SHORT: Power when locked (electromagnetic lock)

See SK-2310g2 Manual page 14 for complete wiring diagram.

---

## Test Mode and Unlock Control Wiring

**CN13 Connector:**

```
Pin 1-2: Test Mode Keyswitch (must be keyswitch for safety)
Pin 3-4: Acknowledge Button (dual-contact momentary)
Pin 5-6: Unlock Switch (momentary button)
Pin 7: Unlock Enable Output (J17 routing)
```

**Jumper Configuration:**
- J17 = 2-3 SHORT: Routes Unlock Enable output to pin 7 (recommended)

See SK-2310g2 Manual page 16-17 for complete wiring diagram.

---

## Spindle Control Wiring

**CN6 Connector:**

```
Pin 2: Spindle Stopped input (HIGH when stopped)
Pin 3: Spindle Fault input
Pin 5: Spindle Enable output (same as CN16.10)
```

**Two Configuration Options:**

### Option 1: Safe Operation Mode (J16=2-3)
- Spindle DISABLED when covers open
- Spindle Enable CANNOT be turned ON in Test Mode
- See SK-2310g2 Manual page 11

### Option 2: Maintenance Mode (J16=1-2)
- Spindle ENABLED in Test Mode with Acknowledge
- Requires J10-3=SHORT and J20=SHORT
- See SK-2310g2 Manual page 12

---

# Detailed Jumper Configuration

## J16 - Spindle Enable with Guards Open 🔴 CRITICAL

**Function:** Controls whether spindle can run when guards are open in Manual Override mode

**Positions:**

| Position | Behavior | Safety Level | Use Case |
|----------|----------|--------------|----------|
| **2-3 SHORT** | Spindle DISABLED when guards open | ✓ SAFE | **Production operation (recommended)** |
| **1-2 SHORT** | Spindle ENABLED with Manual Override + ACK held | ⚠️ DANGER | **Supervised maintenance only** |

**Logic:**

```
J16 = 2-3 SHORT (Safe Mode):
  Spindle ON output (CN6.5, CN16.10) = HIGH when:
    - Outputs/Byte0/Bit2 = 1 (software spindle command)
    AND Power = ON
    AND Guards CLOSED
    AND Safety Link Bridge = 0
    AND ServoFAULT = 0

J16 = 1-2 SHORT (Manual Override Mode):
  Spindle ON output = HIGH when:
    - Outputs/Byte0/Bit2 = 1
    AND Power = ON
    AND (Guards CLOSED OR (Manual Override keyswitch AND ACK button held))
    AND Safety Link Bridge = 0
    AND ServoFAULT = 0
```

**Configuration Requirements for J16=1-2:**
- J16 = 1-2 SHORT (spindle enable in manual override)
- J10-3 = SHORT (manual override spindle enable)
- J20 = SHORT (guards can open without spindle stopped requirement)
- Manual Override keyswitch ON (CN13.1-2 closed)
- ACK button continuously held (CN13.3-4 closed)

**Safety Warnings:**
- ⚠️ J16=1-2 allows spindle operation with guards open
- ⚠️ Only use during supervised maintenance with lockout/tagout
- ⚠️ Return to J16=2-3 for production operation
- ⚠️ Manual Override requires keyswitch AND ACK button held (dual-action)

---

## J20 - Guards in Manual Override Mode 🔴 CRITICAL

**Function:** Controls when guards can be unlocked/opened in Manual Override mode

**Positions:**

| Position | Behavior | Spindle Requirement | Use Case |
|----------|----------|---------------------|----------|
| **OPEN** | Guards open ONLY when spindle stopped | Spindle MUST be stopped | **Production (recommended)** |
| **SHORT** | Guards open anytime with ACK held | Spindle can be running† | **Maintenance (supervised)** |

† If J16=1-2 and J10-3=SHORT

**Logic:**

```
J20 = OPEN (Safe Mode):
  Guard unlock in Manual Override when:
    - Manual Override keyswitch active AND ACK button held
    AND Spindle Stopped (Inputs/Byte0/Bit2 = 1)

J20 = SHORT (Manual Override Mode):
  Guard unlock when:
    - Manual Override keyswitch active AND ACK button held
    (Spindle state irrelevant)
```

**Interaction with J10-3 and J16:**

| J10-3 | J20 | J16 | Guard Opening | Spindle in Manual Override |
|-------|-----|-----|---------------|----------------------------|
| OPEN | OPEN | 2-3 | When spindle stopped | Disabled with guards open |
| SHORT | OPEN | 2-3 | When spindle stopped, auto-unlock | Disabled with guards open |
| SHORT | SHORT | 2-3 | Anytime with ACK held | Disabled with guards open |
| SHORT | SHORT | 1-2 | Anytime with ACK held | **ENABLED with guards open** |

---

## J10 - At Home (Safe State) Detection

**Function:** Controls how "At Home" (safe state) is detected and what operations it enables

**Sub-Jumpers:**

| Jumper | Function | Common Setting |
|--------|----------|----------------|
| J10-1 | At Home (safe state) detection mode | SHORT (immediate mode) |
| J10-2 | Guard automation enable | SHORT (if using automation) |
| J10-3 | Manual override guard/spindle control | OPEN (production), SHORT (maintenance) |
| J10-4 | Enable Zero Speed safe state | SHORT (enable Zero Speed) |

### J10-1: At Home Detection Mode

**Positions:**

```
J10-1 = SHORT (Immediate Mode):
  At Home (Inputs/Byte1/Bit0) = 1 when:
    - Zero Speed signal ON (all motors stopped >2sec)
    AND Spindle Stopped (Inputs/Byte0/Bit2 = 1)
    AND Output9/Byte1/Bit1 = 0 (covers unlocked)

J10-1 = OPEN (Latched Mode):
  At Home = 1 when:
    - Zero Speed ON AND Spindle Stopped
    AND Output9 transitioned from 1 → 0 (active unlock)
```

**Use Case:**
- J10-1=SHORT: At Home whenever conditions met
- J10-1=OPEN: At Home only after intentional unlock action

### J10-2: Guard Automation Enable

**Positions:**

```
J10-2 = OPEN:
  Manual guard control only
  Unlock switch (CN13.5-6) must be pressed

J10-2 = SHORT:
  Automatic guard unlock when At Home (safe state)
  Requires J19=SHORT for full automation
```

**Configuration for Full Automation:**
- J10-2 = SHORT
- J19 = SHORT
- J10-1 = SHORT
- J10-4 = SHORT

### J10-3: Manual Override Control 🔴 CRITICAL

**Function:** Controls guard and spindle operation in Manual Override mode

**Positions:**

```
J10-3 = OPEN (Restricted Manual Override):
  - Guards open ONLY when spindle stopped
  - Spindle CANNOT run with guards open

J10-3 = SHORT (Full Manual Override):
  - Guards open anytime with ACK held (if J20=SHORT)
  - Spindle CAN run with guards open (if J16=1-2)
```

### J10-4: Enable Zero Speed At Home

**Positions:**

```
J10-4 = SHORT:
  Enable Zero Speed mode for At Home detection
  Works with J10-1 to determine immediate vs latched behavior

J10-4 = OPEN:
  Disable Zero Speed mode
  Use physical home switch (CN8) instead
```

---

## J19 - Cover Lock/Unlock Control Mode

**Function:** Selects between manual switch control and automatic cover unlock

**Positions:**

```
J19 = OPEN (Semi-Automatic):
  Cover unlock controlled by:
    - Unlock switch (CN13.5-6) AND
    - Unlock Enable output state

J19 = SHORT (Fully Automatic):
  Cover unlock controlled by Unlock Enable output only
  Requires J10-2=SHORT for At Home automation
  Alternative: Output14 (Byte1/Bit6) for software control
```

---

## J17 - Unlock Enable Output Routing

**Function:** Routes the negative side of the Unlock Enable output

**Positions:**

| Position | CN13 Pin 7 Connected To | Use Case |
|----------|------------------------|----------|
| 1-2 SHORT | GND | Simple unlock circuits |
| **2-3 SHORT** | Unlock Enable output (negative) | **Recommended - standard implementation** |

---

## J21 - Power ON Control Method

**Function:** Enables software control of power-on function

**Positions:**

```
J21 = OPEN (Button Only):
  Power ON only by physical Power ON button

J21 = SHORT (Button or Software):
  Power ON by:
    - Physical Power ON button, OR
    - Software: Byte1/Bit7 transition from "1" to "0" when Power is OFF
```

---

## J14 and J15 - Cover 2 Lock Solenoid Polarity

**Function:** Controls the polarity of Cover 2 lock solenoid drive

**⚠️ Both jumpers must be set to the same position**

**Positions:**

| Position | CN10 Lock Output Behavior | Use Case |
|----------|---------------------------|----------|
| 1-2 SHORT | Powered when Door is UNLOCKED | Spring-return lock with unlock solenoid |
| 2-3 SHORT | Powered when Door is LOCKED | Electromagnetic lock or positive-lock solenoid |

**Note:** Cover 1 does not have polarity selection jumpers; its behavior is fixed.

---

## J2 - Power OFF Delay and Motor Power Monitoring

**Function:** Controls power-off delay timing

See SK-2310g2 Manual page 4 for pin assignments.

---

## J5 - Power Control Pin Routing

**Function:** Routes power control signals to CN16 multifunction pins

### CN16 Pin 3 Routing

| Jumper Position | CN16 Pin 3 Connected To |
|----------------|------------------------|
| Open (default) | Not connected |
| J5 5-6 SHORT | GND |
| J5 6-7 SHORT | CN16 pin 8 (Power Enable) |
| J5 7-8 SHORT | CN16 pin 7 |
| J5 8-9 SHORT | CN16 pin 10 (Spindle ON) |

### CN16 Pin 7 Routing

| Jumper Position | CN16 Pin 7 Connected To |
|----------------|------------------------|
| Open (default) | Not connected |
| J5 1-2 SHORT | CN16 pin 4 (Power Control B) |
| J5 2-3 SHORT | CN16 pin 3 |
| J5 3-4 SHORT | GND |

---

## J4 - CN4 Pin 6 Power Output

**Function:** Controls whether +24V power is provided on CN4 pin 6 (LDCN SLAVE connector)

```
J4 = OPEN: CN4 pin 6 not connected
J4 = SHORT: CN4 pin 6 connected to +24V (powers LDCN network)
```

---

## J6 and J7 - Analog Input Protection Resistors

**Function:** Control protective resistors on analog input power rails

```
J6 (Positive Rail):
  OPEN: 100Ω protective resistor between +5V and CN17 pin4
  SHORT: Direct connection to +5V

J7 (Negative Rail):
  OPEN: 100Ω protective resistor between GND and CN17 pin1
  SHORT: Direct connection to GND
```

**Use Case:**
- OPEN: Protection for external potentiometers
- SHORT: Direct connection for low-impedance sources

---

## J11 and J12 - Home Sensor and LED Configuration

**Function:** Configures home sensor signal routing and diagnostic LED outputs

**Configuration:** Application-specific (see SK-2310g2 Manual page 5 and sample applications pages 13, 21-22)

**J11:** Routes home sensor contacts, controls LED-1 and LED-2
**J12:** Routes safety bus signals, controls LED-3

---

# Connector Pinouts

## CN3 - Safety Bus Connector

| Pin | Signal | SK-2310g2 Role | Related Jumpers |
|-----|--------|----------------|-----------------|
| 1 | Safety Link OUT | **Output** - Generates based on covers/e-stop/At Home | J10, J16, J20 |
| 2 | Safety Link IN | **Input** - Monitors daisy chain integrity | (Diagnostic 0x0A if LOW) |
| 3 | Enable/Stop | **Output** - HIGH when power ON | (J2 power-off delay) |
| 4 | ServoFAULT | **Input** (Inputs/Byte1/Bit2) - Prevents power-on if HIGH | (Diagnostic 0x00 if HIGH) |

## CN6 - Spindle Interface

| Pin | Signal | Function | Related Jumpers |
|-----|--------|----------|-----------------|
| 2 | Spindle Stopped | Input (Inputs/Byte0/Bit2) - Required for spindle-dependent safety | J10, J20 |
| 3 | Spindle Fault | Input (Inputs/Byte0/Bit3) - General purpose fault monitoring | - |
| 5 | Spindle Enable | Output - Same as CN16.10 | J16, J10-3, J20 |

## CN8 - At Home (Safe Zone) Sensor

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | Home A1 | Input - Home sensor contact A, pin 1. Closed in Safety Zone |
| 2 | Home A2 | Input - Home sensor contact A, pin 2 |
| 3 | Home B1 | Input - Home sensor contact B, pin 1. Open in Safety Zone |
| 4 | Home B2 | Input - Home sensor contact B, pin 2 |

## CN9, CN10 - Guard Switches (Guard 1, Guard 2)

| Pin | Signal | Function |
|-----|--------|----------|
| 1-4 | Guard Closed | Dual-contact guard position sensors |
| 5-6 | Guard Lock/Unlock | Lock solenoid outputs |

## CN13 - Manual Override and Unlock Control

| Pin | Signal | Function | Related Jumpers |
|-----|--------|----------|-----------------|
| 1-2 | Manual Override Switch | Input (Inputs/Byte1/Bit1) - Must be keyswitch | J10-3, J16, J20 |
| 3-4 | Acknowledge Button | Dual-contact enable - must be held during operation | J10-3, J16, J20 |
| 5-6 | Unlock Switch | Manual guard unlock button | J19 |
| 7 | Unlock Enable | Output - Automatic unlock control | J17, J19 |

## CN16 - Power Control Connector

| Pin | Signal | Function | Related Jumpers |
|-----|--------|----------|-----------------|
| 5 | Monitor Loop (-) | Relay contact monitor return | J5 routing |
| 6 | Monitor Loop (+) | Relay contact monitor source | J5 routing |
| 8 | Power Enable | Main power contactor control | J5 routing |
| 10 | Spindle ON | Spindle enable output (wired to CN6.5) | J16, J10-3, J20 |

For complete connector pinouts, see SK-2310g2 Manual pages 6-10.

---

**Frequency:** Every 6 months or after hardware changes

### Test Checklist

| Test | Pass/Fail | Notes |
|------|-----------|-------|
| Power-on diagnostic code 0x00 | [ ] | |
| Home sensor At Home detection | [ ] | |
| Home sensor transition timing | [ ] | |
| Cover lock/unlock operation | [ ] | |
| Spindle interlock (covers open) | [ ] | |
| Test Mode access | [ ] | |
| Safety Link chain integrity | [ ] | |
| ServoFAULT detection | [ ] | |
| Emergency stop response | [ ] | |
| LED indicators match status | [ ] | |

---

# Related Documentation

**pyldcn Documentation:**
- [io_commands.md](io_commands.md) - I/O hardware specifications (digital/analog I/O, PWM, counter/timer)
- [safety_bus.md](safety_bus.md) - Generic Safety Bus protocol specification
- [LDCN_PROTOCOL.md](LDCN_PROTOCOL.md) - Generic LDCN network commands

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

2. ⚠️ **Test Mode is for supervised maintenance only** - Return to production configuration after maintenance

3. ⚠️ **Jumper J18 must remain OPEN** - Reserved for future use, improper configuration may cause malfunction

4. ⚠️ **J14 and J15 must match** - Mismatch causes Cover 2 lock malfunction

5. ⚠️ **Regular testing required** - Test safety functions periodically per [Testing Procedures](#testing-procedures)

6. ⚠️ **Qualified personnel only** - Installation and maintenance by trained technicians only

---

# References

- CNC-SK-2310g2 Supervisor I/O Controller Manual, Doc # 710231005 / Rev. D, 03/05/2020
- LS-2310g2-Supervisor-IO-Controller.pdf (24-page official manual)
- LDCN Protocol Specification
- Safety Bus Protocol Specification
