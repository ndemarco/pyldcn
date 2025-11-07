# SK-2310g2 Safety System Implementation

This document provides comprehensive implementation guidance for the **CNC-SK-2310g2 Supervisor I/O Controller** as a safety system master in LDCN-based CNC machine control.

**Document Type:** Device-specific implementation guide

**Target Audiences:**
- **System Designers** - Selecting jumper configurations and safety architecture
- **Software Developers** - Control code, status communication, operator interface
- **Technicians** - Building, troubleshooting, and maintaining machines

**Primary Source:** CNC-SK-2310g2 Supervisor I/O Controller Manual, Doc # 710231005 / Rev. D, 03/05/2020

**Prerequisites:** Read `safety_bus.md` for generic Safety Bus specification before proceeding.

---

## Document Overview

### What This Document Covers

1. **SK-2310g2 Safety Architecture** - Role as safety master, relay-based implementation
2. **Jumper Configuration** - Complete reference for all 21 jumpers affecting safety
3. **Wiring Examples** - Practical connector.pin notation for common configurations
4. **LDCN Commands** - Safety-related commands with expected responses by configuration
5. **Configuration Recipes** - Pre-validated jumper settings for common machine types
6. **Troubleshooting** - Symptom-based diagnosis with LDCN command sequences
7. **Status Monitoring** - Software integration for operator displays and HMI

### How to Use This Document

**System Designers:**
- Start with "Configuration Recipes" (Section 8) to select base configuration
- Review "Jumper Configuration Reference" (Section 2) for customization
- Use "Wiring Examples" (Section 3) for schematic development

**Software Developers:**
- Review "LDCN Commands for Safety Monitoring" (Section 4)
- Use "Status Monitoring Integration" (Section 5) for HMI development
- Reference "Diagnostic Code Interpretation" (Section 6) for error handling

**Technicians:**
- Use "Wiring Examples" (Section 3) during installation
- Reference "Tr

oubleshooting by Symptom" (Section 7) during commissioning/maintenance
- Follow "Testing Procedures" (Section 9) for validation

---

# Section 1: SK-2310g2 Safety Architecture

## Role as Safety Master

The SK-2310g2 serves as the **safety system coordinator** in LDCN-based CNC systems:

**Core Functions:**
1. **Emergency Stop Monitoring** - 5 e-stop locations via dual-line monitoring
2. **Cover Interlock Management** - Dual-contact cover switches with lock/unlock control
3. **Safe Zone Detection** - "At Home" sensor with validation delays
4. **Test Mode Control** - Supervised maintenance mode with dual-action enable
5. **Safety Link Generation** - Master Safety Link OUT based on all conditions
6. **Safety Link Monitoring** - Detects broken safety chain from downstream devices
7. **ServoFAULT Aggregation** - Monitors all servo drives for faults
8. **Power Control** - Relay-based contactor control with monitoring loop
9. **Spindle Interlocks** - Prevents spindle operation in unsafe conditions
10. **Diagnostic Reporting** - 32 diagnostic codes for system state

**Hardware Safety Implementation:**
- 100% relay-based (not software-dependent)
- Fail-safe design (open circuit = unsafe)
- Independent of LDCN communication
- <50ms response time (relay switching)

---

## Safety Signal Summary

### CN3 - Safety Bus Connector

| Pin | Signal | SK-2310g2 Role | Related Jumpers |
|-----|--------|----------------|-----------------|
| CN3.1 | Safety Link OUT | **Output** - Generates based on covers/e-stop/At Home | J10, J16, J20 |
| CN3.2 | Safety Link IN | **Input** - Monitors daisy chain integrity | (Diagnostic 0x0A if LOW) |
| CN3.3 | Enable/Stop | **Output** - HIGH when power ON | (J2 power-off delay) |
| CN3.4 | ServoFAULT | **Input** (Inputs/Byte1/Bit2) - Prevents power-on if HIGH | (Diagnostic 0x00 if HIGH) |

### CN16 - Power Control Connector

| Pin | Signal | Function | Related Jumpers |
|-----|--------|----------|-----------------|
| CN16.5 | Monitor Loop (-) | Relay contact monitor return | J5 routing |
| CN16.6 | Monitor Loop (+) | Relay contact monitor source | J5 routing |
| CN16.8 | Power Enable | Main power contactor control | J5 routing |
| CN16.10 | Spindle ON | Spindle enable output (wired to CN6.5) | J16, J10-3, J20 |

### CN6 - Spindle Interface Connector

| Pin | Signal | Function | Related Jumpers |
|-----|--------|----------|-----------------|
| CN6.2 | Spindle Stopped | Input (Inputs/Byte0/Bit2) - Must be HIGH for spindle-dependent safety | J10, J20 |
| CN6.3 | Spindle Fault | Input (Inputs/Byte0/Bit3) - General purpose fault monitoring | - |
| CN6.5 | Spindle Enable | Output - Same as CN16.10 | J16, J10-3, J20 |

### CN13 - Test Mode and Unlock Control

| Pin | Signal | Function | Related Jumpers |
|-----|--------|----------|-----------------|
| CN13.1-2 | Test Mode Switch | Input (Inputs/Byte1/Bit1) - Must be keyswitch | J10-3, J16, J20 |
| CN13.3-4 | Acknowledge Button | Dual-contact enable for test mode | J10-3, J16, J20 |
| CN13.5-6 | Unlock Switch | Manual cover unlock button | J19 |
| CN13.7 | Unlock Enable | Output - Automatic unlock control | J17, J19 |

### CN8 - At Home (Safe Zone) Sensor

| Pin | Signal | Function | Related Jumpers |
|-----|--------|----------|-----------------|
| CN8 (various) | At Home Sensor | Dual-contact safe zone detection | J10-1, J10-4, J11, J12 |

### CN9, CN10 - Cover Switches

| Pin | Signal | Function | Related Jumpers |
|-----|--------|----------|-----------------|
| CN9/CN10 | Cover Closed | Dual-contact cover position sensors | All safety jumpers |
| CN9/CN10 | Cover Lock/Unlock | Lock solenoid outputs | J14, J15, J19 |

---

# Section 2: Jumper Configuration Reference

## Safety-Critical Jumpers Summary

| Jumper | Function | Safety Impact | Configuration Complexity |
|--------|----------|---------------|--------------------------|
| **J16** | Spindle with covers open | 🔴 CRITICAL | Simple (2 positions) |
| **J20** | Covers open in test mode | 🔴 CRITICAL | Simple (2 positions) |
| **J10** | At Home detection mode | 🟡 HIGH | Complex (4 sub-jumpers) |
| **J19** | Cover automation | 🟡 HIGH | Simple (2 positions) |
| J2 | Power-off delay | 🟢 MEDIUM | Simple (4 positions) |
| J14/J15 | Cover lock polarity | 🟢 MEDIUM | Simple (must match) |
| J21 | Software power-on | 🟢 LOW | Simple (2 positions) |

---

## J16 - Spindle Enable with Covers Open 🔴 CRITICAL

**Function:** Controls whether spindle can run when covers are open in Test Mode

**Safety Impact:** CRITICAL - Determines if rotating spindle is accessible

**Positions:**

| Position | Behavior | Safety Level | Use Case |
|----------|----------|--------------|----------|
| **2-3 SHORT** | Spindle DISABLED when covers open | ✓ SAFE | **Production operation (recommended)** |
| **1-2 SHORT** | Spindle ENABLED in Test Mode + Acknowledge | ⚠️ DANGER | **Supervised maintenance only** |

**Detailed Logic:**

```
J16 = 2-3 SHORT (Safe Mode):
  Spindle ON output (CN6.5, CN16.10) = HIGH when:
    - Outputs/Byte0/Bit2 = 1 (software spindle command)
    AND
    - Outputs/Byte1/Bit4 (Safety Link Bridge) = 0
    AND
    - Inputs/Byte1/Bit2 (ServoFAULT) = 0
    AND
    - Power = ON
    AND
    - Covers CLOSED

  Result: Spindle CANNOT run with covers open under any circumstances

J16 = 1-2 SHORT (Maintenance Mode):
  Spindle ON output = HIGH when:
    - Outputs/Byte0/Bit2 = 1
    AND
    - Outputs/Byte1/Bit4 = 0
    AND
    - Inputs/Byte1/Bit2 (ServoFAULT) = 0
    AND
    - Power = ON
    AND
    - (Covers CLOSED  OR  (Test Mode AND Acknowledge))

  Result: Spindle CAN run with covers open if:
    - Test Mode active (Inputs/Byte1/Bit1 = 1 from CN13.1-2)
    - Acknowledge pressed (CN13.3-4 closed)
    - J10-3 = SHORT (enables spindle in test mode)
```

**Configuration Requirements:**

For J16=1-2 (maintenance mode) to allow spindle with covers open:
- **J16 = 1-2 SHORT** (spindle enable in test mode)
- **J10-3 = SHORT** (test mode spindle enable)
- **J20 = SHORT** (covers can open in test mode without spindle stopped requirement)
- Test Mode keyswitch ON (CN13.1-2 closed)
- Acknowledge button pressed (CN13.3-4 closed)

**Wiring:**
- No external wiring changes required
- Jumper located on SK-2310g2 PCB

**LDCN Commands - Read Spindle Status:**
```python
# Read outputs to verify spindle command state
CMD_READ_OUTPUT (0x0E)
Response: [NW, HW, Addr, Grp, stat, byte0, byte1, chk]
- byte0 bit 2: Software spindle command (Outputs/Byte0/Bit2)
- byte1 bit 4: Safety Link Bridge (must be 0 for spindle)

# Read inputs to verify conditions
CMD_READ_STATUS (0x00)
Response: [NW, HW, Addr, Grp, stat, byte0, byte1, chk]
- byte1 bit 1: Test Mode (Inputs/Byte1/Bit1)
- byte1 bit 2: ServoFAULT (Inputs/Byte1/Bit2)

# Expected with J16=2-3, covers open, spindle commanded:
# Spindle ON output = LOW (spindle will not run)

# Expected with J16=1-2, test mode+ack, covers open, spindle commanded:
# Spindle ON output = HIGH (spindle runs - DANGER)
```

**Safety Warnings:**
- ⚠️ J16=1-2 allows spindle operation with covers open
- ⚠️ Only use during supervised maintenance with lockout/tagout
- ⚠️ Return to J16=2-3 for production operation
- ⚠️ Test Mode requires keyswitch AND acknowledge (dual-action)

---

## J20 - Covers in Test Mode 🔴 SAFETY CRITICAL

**Function:** Controls when covers can be unlocked/opened in Test Mode

**Safety Impact:** CRITICAL - Determines spindle stopped requirement for cover opening

**Positions:**

| Position | Behavior | Spindle Requirement | Use Case |
|----------|----------|---------------------|----------|
| **OPEN** | Covers open ONLY when spindle stopped | Spindle MUST be stopped | **Production (recommended)** |
| **SHORT** | Covers open anytime with Acknowledge | Spindle can be running† | **Maintenance (supervised)** |

† If J16=1-2 and J10-3=SHORT

**Detailed Logic:**

```
J20 = OPEN (Safe Mode):
  Cover unlock permitted in Test Mode when:
    - Test Mode active (Inputs/Byte1/Bit1 = 1)
    AND
    - Acknowledge pressed (CN13.3-4)
    AND
    - Spindle Stopped (Inputs/Byte0/Bit2 = 1)

  If spindle running: Covers remain locked until spindle stops
  Automatic unlock after spindle stops (if J10-3=SHORT)

J20 = SHORT (Maintenance Mode):
  Cover unlock permitted in Test Mode when:
    - Test Mode active
    AND
    - Acknowledge pressed

  Spindle state irrelevant - covers can open while spindle running
```

**Interaction with J10-3 and J16:**

| J10-3 | J20 | J16 | Cover Opening | Spindle in Test Mode |
|-------|-----|-----|---------------|----------------------|
| OPEN | OPEN | 2-3 | When spindle stopped | Disabled with covers open |
| SHORT | OPEN | 2-3 | When spindle stopped, auto-unlock after stop | Disabled with covers open |
| SHORT | SHORT | 2-3 | Anytime with Ack | Disabled with covers open |
| SHORT | SHORT | 1-2 | Anytime with Ack | **ENABLED with covers open** |

**LDCN Commands - Monitor Cover Unlock Status:**
```python
# Read cover status
CMD_READ_STATUS (0x00)
Response: [NW, HW, Addr, Grp, stat, byte0, byte1, chk]
- byte0 bit 2: Spindle Stopped (Inputs/Byte0/Bit2)
- byte1 bit 1: Test Mode (Inputs/Byte1/Bit1)

# Read outputs for unlock command
CMD_READ_OUTPUT (0x0E)
Response: [NW, HW, Addr, Grp, stat, byte0, byte1, chk]
- byte1 bit 1: Cover Lock (Outputs/Byte1/Bit1, 0=unlocked)

# Test scenario with J20=OPEN:
# 1. Test Mode ON, Ack pressed, spindle running
# Expected: Covers remain locked (Byte1/Bit1=1)
# 2. Stop spindle (Byte0/Bit2 → 1)
# Expected: Covers unlock automatically (Byte1/Bit1 → 0)

# Test scenario with J20=SHORT:
# 1. Test Mode ON, Ack pressed, spindle running
# Expected: Covers unlock immediately (Byte1/Bit1 → 0)
```

**Configuration Recommendations:**

**Production CNC:**
```
J20 = OPEN
J10-3 = OPEN
J16 = 2-3
Result: Maximum safety, covers require spindle stopped
```

**Setup/Maintenance:**
```
J20 = SHORT
J10-3 = SHORT
J16 = 1-2
Result: Full access, requires test mode + acknowledge
```

---

## J10 - At Home (Safe Zone) Detection and Automation

**Function:** Controls how "At Home" state is detected and what operations it enables

**Safety Impact:** HIGH - Affects cover unlock automation and safety link generation

**Complexity:** 4 sub-jumpers with interdependencies

**Sub-Jumpers:**

| Jumper | Function | Common Setting |
|--------|----------|----------------|
| J10-1 | At Home detection mode | SHORT (standard mode) |
| J10-2 | Cover automation enable | SHORT (if using automation) |
| J10-3 | Test mode cover/spindle control | OPEN (production), SHORT (maintenance) |
| J10-4 | Enable Zero Speed At Home | SHORT (enable) |

---

### J10-1: At Home Detection Mode

**Function:** Defines when "At Home" state is asserted

**Positions:**

```
J10-1 = SHORT (Standard Mode):
  At Home (Inputs/Byte1/Bit0) = 1 when:
    - Zero Speed signal ON (all motors stopped >2sec)
    AND
    - Spindle Stopped (Inputs/Byte0/Bit2 = 1)
    AND
    - Output9/Byte1/Bit1 = 0 (covers not locked)

  At Home cleared when:
    - Any motor moves (Zero Speed OFF)
    OR
    - Spindle starts
    OR
    - Covers locked (Output9 = 1)

J10-1 = OPEN (Transition Mode):
  At Home = 1 when:
    - Zero Speed ON
    AND
    - Spindle Stopped
    AND
    - Output9 transitioned from 1 → 0 (active unlock)

  Requires explicit unlock action, not just unlocked state
```

**Use Case:**
- J10-1=SHORT: Typical - At Home whenever conditions met
- J10-1=OPEN: Strict - At Home only after intentional unlock action

**LDCN Commands:**
```python
# Read At Home status
CMD_READ_STATUS (0x00)
Response: [NW, HW, Addr, Grp, stat, byte0, byte1, chk]
- byte1 bit 0: At Home (Inputs/Byte1/Bit0)
- byte0 bit 2: Spindle Stopped (Inputs/Byte0/Bit2)

# Read cover lock status
CMD_READ_OUTPUT (0x0E)
Response: [NW, HW, Addr, Grp, stat, byte0, byte1, chk]
- byte1 bit 1: Cover Lock (Outputs/Byte1/Bit1)

# With J10-1=SHORT:
# Park all axes, stop spindle, unlock covers
# Expected: At Home goes HIGH immediately

# With J10-1=OPEN:
# Park all axes, stop spindle, covers already unlocked
# Expected: At Home remains LOW
# Send unlock command (Byte1/Bit1: 1→0 transition)
# Expected: At Home goes HIGH after transition
```

---

### J10-2: Cover Automation Enable

**Function:** Enables automatic cover unlock when At Home conditions met

**Positions:**

```
J10-2 = OPEN:
  Manual cover control only
  Unlock switch (CN13.5-6) must be pressed
  Or LDCN command to clear Output9/Byte1/Bit1

J10-2 = SHORT:
  Automatic cover unlock when At Home
  Requires J19=SHORT for full automation
  Covers unlock automatically when:
    - At Home = 1
    AND
    - Spindle Stopped = 1
    AND
    - Power OFF or (Power ON and At Home)
```

**Configuration for Full Automation:**
- J10-2 = SHORT
- J19 = SHORT
- J10-1 = SHORT (standard At Home detection)
- J10-4 = SHORT (enable Zero Speed)

**LDCN Commands - Automatic Unlock:**
```python
# With J10-2=SHORT, J19=SHORT:
# Park axes at home, stop spindle
CMD_READ_STATUS:
  Expected: byte1 bit 0 = 1 (At Home)

# Cover lock automatically clears
CMD_READ_OUTPUT:
  Expected: byte1 bit 1 = 0 (Cover Lock cleared)

# No manual unlock command needed
```

---

### J10-3: Test Mode Control 🔴 SAFETY CRITICAL

**Function:** Controls cover and spindle operation in Test Mode

**Safety Impact:** CRITICAL - Determines test mode capabilities

**Positions:**

```
J10-3 = OPEN (Restricted Test Mode):
  Test Mode allows:
    - Covers open ONLY when spindle stopped
    - Spindle CANNOT run with covers open (even if J16=1-2)
    - Automatic cover unlock after spindle stops (if J20=OPEN)

J10-3 = SHORT (Full Test Mode):
  Test Mode allows:
    - Covers open anytime with Acknowledge (if J20=SHORT)
    - Spindle CAN run with covers open (if J16=1-2)
    - Full maintenance access
```

**Combined Effects:**

| J10-3 | J20 | J16 | Test Mode Behavior |
|-------|-----|-----|--------------------|
| OPEN | OPEN | 2-3 | **Production Safe**: Covers open only when spindle stopped, spindle never runs with covers open |
| SHORT | OPEN | 2-3 | Covers open when spindle stopped, auto-unlock, spindle disabled with covers open |
| SHORT | SHORT | 2-3 | Covers open anytime, spindle disabled with covers open |
| SHORT | SHORT | 1-2 | **Full Maintenance**: Covers open anytime, spindle can run with covers open |

**Recommended Configurations:**

**Production:**
```
J10-3 = OPEN
J20 = OPEN
J16 = 2-3
```

**Maintenance:**
```
J10-3 = SHORT
J20 = SHORT
J16 = 1-2
(Requires test mode keyswitch + acknowledge)
```

---

### J10-4: Enable Zero Speed At Home

**Function:** Enables "Zero Speed" signal for At Home detection

**Positions:**

```
J10-4 = SHORT:
  At Home uses Zero Speed signal from servo drives
  Zero Speed = ON when all drives stopped >2sec
  Recommended for most applications

J10-4 = OPEN:
  At Home does not use Zero Speed
  Alternative safe zone detection method required
  Special applications only
```

**Typical Setting:** J10-4 = SHORT (enabled)

**Zero Speed Signal:**
- Generated by LS-231SE servo drives
- Active when ALL axes stopped >2 seconds
- Wired to SK-2310g2 (connector varies by application)
- Used for safe cover opening during tool changes

**LDCN Monitoring:**
```python
# Zero Speed not directly readable via LDCN
# Inferred from At Home status:
CMD_READ_STATUS:
  byte1 bit 0: At Home (includes Zero Speed in logic)

# To verify Zero Speed is working:
# 1. Park all axes
# 2. Wait >2 seconds
# 3. Stop spindle
# 4. Check At Home status
# Expected: At Home = 1 (if covers unlocked and J10-4=SHORT)
```

---

## J19 - Cover Lock/Unlock Control Mode

**Function:** Selects manual vs. automatic cover control

**Safety Impact:** HIGH - Affects operator workflow and automation

**Positions:**

| Position | Control Mode | Unlock Method | Automation |
|----------|--------------|---------------|------------|
| **OPEN** | Manual | Unlock switch (CN13.5-6) + Unlock Enable | No automation |
| **SHORT** | Automatic | Unlock Enable output only | At Home automation (if J10-2=SHORT) |

**Detailed Logic:**

```
J19 = OPEN (Manual Mode):
  Cover unlock when:
    - Unlock switch pressed (CN13.5-6 closed)
    AND
    - Unlock Enable output active
    AND
    - Safety conditions met:
      - Spindle stopped AND Power OFF
      OR
      - Spindle stopped AND At Home
      OR
      - Test Mode AND Acknowledge (if J20 permits)

J19 = SHORT (Automatic Mode):
  Cover unlock when:
    - Unlock Enable output active (no switch required)
    AND
    - Safety conditions met (same as above)
    AND
    - J10-2 = SHORT (enable automation)

  Covers automatically unlock when At Home conditions met
  Covers automatically lock when motion starts
```

**Configuration Combinations:**

| J19 | J10-2 | Behavior | Use Case |
|-----|-------|----------|----------|
| OPEN | X | Manual unlock button required | Traditional CNC - operator controlled |
| SHORT | OPEN | Unlock Enable output, no automation | Custom control logic |
| SHORT | SHORT | **Full automation** | Modern CNC - automatic tool change |

**Wiring:**

**Manual Mode (J19=OPEN):**
```
CN13.5 ──┬── Unlock button (NO) ──┬── 24V
CN13.6 ──┘                         │
CN13.7 (Unlock Enable) ────────────┘
```

**Automatic Mode (J19=SHORT):**
```
CN13.7 (Unlock Enable) controls unlock directly
CN13.5-6 not used (or can be wired for manual override)
```

**LDCN Commands - Cover Control:**
```python
# Manual unlock (J19=OPEN):
# Operator presses unlock button on CN13.5-6
# Software monitors:
CMD_READ_OUTPUT:
  byte1 bit 1: Cover Lock (0=unlocked, 1=locked)

# Automatic unlock (J19=SHORT, J10-2=SHORT):
# Park axes at home position
CMD_READ_STATUS:
  byte1 bit 0: Wait for At Home = 1

# Covers unlock automatically
CMD_READ_OUTPUT:
  byte1 bit 1: Cover Lock → 0 (automatic)

# Lock covers before motion:
CMD_WRITE_OUTPUT (0x1E):
  Data: [byte0, byte1]
  byte1 bit 1: Set to 1 (lock covers)

# Start motion - covers remain locked
```

---

## J2 - Power OFF Delay and Motor Power Monitoring

**Function:** Configures power-off relay transition delay and motor power monitoring

**Safety Impact:** MEDIUM - Affects diagnostic code timing and power monitoring

**Positions:**

| Position | Power OFF Delay | Motor Power Monitoring |
|----------|-----------------|------------------------|
| J2-1 | 1 second | Monitored (if J2-4 OFF) |
| **J2-2** | **2 seconds (typical)** | Monitored (if J2-4 OFF) |
| J2-3 | 4 seconds | Monitored (if J2-4 OFF) |
| J2-4 ON | Delay per J2-1/2/3 | **NOT monitored** |
| J2-4 OFF | Delay per J2-1/2/3 | **Monitored** |

**Power OFF Delay:**
- Time required for safety relays to fully transition after power-off
- Diagnostic code remains 0x1F during delay, then updates to actual state
- Longer delay = more conservative, allows slower relays to settle

**Motor Power Monitoring:**
- J2-4 OFF: SK-2310g2 monitors motor power supply voltage (CN1.3 UM)
- J2-4 ON: Motor power not monitored (allows motor power off while control on)

**Typical Setting:**
```
J2-2 = SHORT (2-second delay)
J2-4 = OFF (monitor motor power)
```

**LDCN Commands - Power Off Sequence:**
```python
# Power off command (software or button)
CMD_WRITE_OUTPUT (0x1E):
  byte1 bit 7: 1 → 0 (if J21=SHORT and software power-on enabled)

# OR: Physical power button

# Immediately after power-off:
CMD_READ_DIAGNOSTIC (0x02):
  Response: 0x1F or previous code (during J2 delay)

# After J2 delay (e.g., 2 seconds):
CMD_READ_DIAGNOSTIC:
  Response: 0x00 or other code reflecting actual state

# J2 delay ensures relays have settled before diagnostic update
```

---

## J14 and J15 - Cover Lock Output Logic

**Function:** Configures cover lock solenoid output polarity

**Safety Impact:** MEDIUM - Must match lock solenoid wiring

**Requirement:** **J14 and J15 must be set identically**

**Positions:**

| Position | CN10 (Door 2) Lock Output | Solenoid Type |
|----------|---------------------------|---------------|
| **1-2 SHORT** | Powered when door **unlocked** | Fail-safe (spring-loaded locked) |
| **2-3 SHORT** | Powered when door **locked** (recommended) | Active locked (powered to lock) |

**Solenoid Types:**

**Fail-Safe (J14/J15 = 1-2):**
- Spring holds lock engaged (locked state)
- Solenoid energized to release (unlock)
- Loss of power = locked (safe)
- Requires continuous power to keep covers open

**Active Lock (J14/J15 = 2-3, recommended):**
- Spring returns to unlocked state
- Solenoid energized to engage lock
- Loss of power = unlocked (allows escape)
- Power only when locking needed

**Wiring:**

**Cover 1 (CN9):**
```
CN9.5 ─┬─ Lock solenoid (+) ─┬─ CN9.7 (24V)
CN9.6 ─┘                      │
CN9.3-4: Cover closed sensor ─┘
```

**Cover 2 (CN10):**
```
CN10.5 ─┬─ Lock solenoid (+) ─┬─ CN10.7 (24V)
CN10.6 ─┘                       │
CN10.1-2: Cover closed sensor ──┘
```

**Logic:**
```
J14/J15 = 2-3 (recommended):
  Output9/Byte1/Bit1 = 0: CN9/CN10 lock outputs OFF (unlocked)
  Output9/Byte1/Bit1 = 1: CN9/CN10 lock outputs ON (locked)

J14/J15 = 1-2:
  Output9/Byte1/Bit1 = 0: CN9/CN10 lock outputs ON (unlocked)
  Output9/Byte1/Bit1 = 1: CN9/CN10 lock outputs OFF (locked)
```

**LDCN Commands - Cover Lock Control:**
```python
# Lock covers (Output9/Byte1/Bit1 = 1):
CMD_WRITE_OUTPUT (0x1E):
  Data: [byte0, byte1]
  byte1 bit 1: Set to 1

# With J14/J15=2-3:
# CN9.5-6 and CN10.5-6 energize
# Lock solenoids engage
# Covers physically locked

# Unlock covers (Output9/Byte1/Bit1 = 0):
CMD_WRITE_OUTPUT (0x1E):
  byte1 bit 1: Clear to 0

# With J14/J15=2-3:
# CN9.5-6 and CN10.5-6 de-energize
# Lock solenoids release
# Covers can be opened
```

**Configuration Recommendation:**
```
J14 = 2-3 SHORT
J15 = 2-3 SHORT
(Powered to lock, fail-unlocked)
```

---

## J21 - Power ON Control Method

**Function:** Enables software power-on via LDCN command

**Safety Impact:** LOW - Convenience feature for remote/automated startup

**Positions:**

| Position | Power ON Methods |
|----------|------------------|
| **OPEN** | Physical power button only |
| **SHORT** | Physical power button OR software command |

**Software Power-On Logic:**
```
J21 = SHORT:
  Power ON when:
    - Outputs/Byte1/Bit7 (System Lock) transitions 1 → 0
    AND
    - Power is currently OFF
    AND
    - All safety conditions met (e-stop released, etc.)

  Sequence:
    1. System powered down: Byte1/Bit7 = 0
    2. Software sets Byte1/Bit7 = 1 (lock system)
    3. Software clears Byte1/Bit7 = 0 (1→0 transition triggers power-on)
    4. System powers on if safety conditions met
```

**LDCN Commands - Software Power-On:**
```python
# Verify power is OFF:
CMD_READ_DIAGNOSTIC (0x02):
  Response: 0x00, 0x04, 0x10, 0x14, etc. (not 0x1F)

# Verify System Lock state:
CMD_READ_OUTPUT (0x0E):
  byte1 bit 7: Current System Lock state

# Software power-on sequence (requires J21=SHORT):
# Step 1: Set System Lock
CMD_WRITE_OUTPUT (0x1E):
  Data: [0x00, 0x80]  # byte1 bit 7 = 1 (lock)

# Step 2: Clear System Lock (triggers power-on)
CMD_WRITE_OUTPUT (0x1E):
  Data: [0x00, 0x00]  # byte1 bit 7 = 0 (1→0 transition)

# Step 3: Verify power-on occurred:
CMD_READ_DIAGNOSTIC:
  Expected: 0x1F (normal operation) or 0x14/0x1C (covers open)

# If power-on fails, check:
# - Emergency stop released
# - Safety Link IN = HIGH
# - Control voltage ≥18V
# - ServoFAULT = LOW
```

**Use Cases:**
- Remote machine startup
- Automated startup sequences
- HMI power-on button (software-controlled)
- Recovery after brief power interruption

**Configuration:**
- Production: J21=OPEN or SHORT (user preference)
- Automated systems: J21=SHORT (required for software control)

---

# Section 3: Wiring Examples

## Example 1: Basic 3-Axis CNC with Spindle

**System Configuration:**
- SK-2310g2 supervisor (Address 1)
- LS-231SE drives: X-axis (Addr 2), Y-axis (Addr 3), Z-axis (Addr 4)
- LS-2315 spindle drive (non-LDCN, analog control)
- Single cover with dual-contact switch
- 2 e-stop locations
- At Home sensor at machine origin

**Jumper Settings (Production Mode):**
```
J2-2 = SHORT (2-second delay)
J10-1 = SHORT (standard At Home)
J10-2 = SHORT (cover automation)
J10-3 = OPEN (restricted test mode)
J10-4 = SHORT (enable Zero Speed)
J14/J15 = 2-3 (powered to lock)
J16 = 2-3 (spindle disabled with covers open)
J19 = SHORT (automatic cover control)
J20 = OPEN (covers open only when spindle stopped)
J21 = SHORT (software power-on enabled)
```

**SK-2310g2 Connections:**

### Safety Bus Chain
```
SK-2310g2 CN3.1 (Safety Link OUT) ──> LS-231SE X-axis CN3.2 (IN)
LS-231SE X-axis CN3.1 (OUT) ──> LS-231SE Y-axis CN3.2 (IN)
LS-231SE Y-axis CN3.1 (OUT) ──> LS-231SE Z-axis CN3.2 (IN)
LS-231SE Z-axis CN3.1 (OUT) ──> SK-2310g2 CN3.2 (Safety Link IN)

SK-2310g2 CN3.3 (Enable/Stop) ──> LS-231SE X/Y/Z CN3.3 (parallel)
LS-231SE X/Y/Z CN3.4 (ServoFAULT) ──> SK-2310g2 CN3.4 (OR'd together)
```

### Power Control
```
SK-2310g2 CN1.1 (GND) ──> 24V supply ground
SK-2310g2 CN1.2 (24V) ──> 24V supply +
SK-2310g2 CN1.3 (UM) ──> Motor power supply + (monitored if J2-4=OFF)

SK-2310g2 CN16.8 (Power Enable) ──> Main contactor coil ──> 24V return
SK-2310g2 CN16.9 (GND) ──> Common ground

# Monitor loop (J5 configured for CN16.6 ← CN16.8):
SK-2310g2 CN16.5 (Monitor (-)) ──> Contactor NC contact ──> CN16.6 (Monitor (+))
```

### Spindle Control
```
SK-2310g2 CN6.2 (Spindle Stopped) ──> LS-2315 Spindle At-Speed relay ──> 24V
SK-2310g2 CN6.3 (Spindle Fault) ──> LS-2315 Fault output ──> GND (active low)
SK-2310g2 CN6.5 (Spindle Enable) ──> LS-2315 Enable input
SK-2310g2 CN6.9 (Analog GND) ──> LS-2315 Analog ground
SK-2310g2 CN6.10 (Speed Command 0-10V) ──> LS-2315 Speed input
```

### Cover Interlock
```
SK-2310g2 CN9.1 (Cover A) ──> Cover switch contact A ──> 24V
SK-2310g2 CN9.2 (Cover B) ──> Cover switch contact B ──> 24V
SK-2310g2 CN9.5 (Lock +) ──┬──> Lock solenoid (+)
SK-2310g2 CN9.6 (Lock -) ──┘
SK-2310g2 CN9.7 (24V) ──> Lock solenoid (-) ──> GND
```

### E-Stop Circuit
```
# Dual-line monitoring, series connection:
24V ──> E-stop 1 (A1-B1) ──> E-stop 2 (A1-B1) ──> SK-2310g2 CN4.11 (EMG A1)
24V ──> E-stop 1 (A2-B2) ──> E-stop 2 (A2-B2) ──> SK-2310g2 CN4.12 (EMG B1)

# Second set of e-stops on CN11 or CN13 (see manual for CN assignments)
```

### At Home Sensor
```
SK-2310g2 CN8 pins (dual-contact) ──> At Home sensor ──> 24V
(Exact pins vary - see J11, J12 configuration and CN8 pinout in manual)
```

### Test Mode Control
```
SK-2310g2 CN13.1 ──> Test Mode keyswitch (NO) ──> SK-2310g2 CN13.2 ──> 24V
SK-2310g2 CN13.3 ──> Acknowledge button (NO) ──> SK-2310g2 CN13.4 ──> 24V
SK-2310g2 CN13.5 ──> Unlock button (NO) ──> SK-2310g2 CN13.6 ──> 24V (if J19=OPEN)
SK-2310g2 CN13.7 (Unlock Enable) ──> (used for automation if J19=SHORT)
```

**LDCN Software Initialization:**
```python
from pyldcn import LDCNNetwork

# Initialize network
network = LDCNNetwork('/dev/ttyUSB0')
network.open()
network.initialize()  # Discover all devices
network.set_baud_rate(125000)

# Expected devices:
# Address 1: SK-2310g2
# Address 2: LS-231SE (X-axis)
# Address 3: LS-231SE (Y-axis)
# Address 4: LS-231SE (Z-axis)

# Check SK-2310g2 safety status:
sk2310 = network.devices[0]  # Address 1
diag = sk2310.read_diagnostic()
print(f"Diagnostic: 0x{diag:02X}")
# Expected: 0x14 (covers open, ready) or 0x1F (normal operation)

# Check ServoFAULT:
status = sk2310.read_status()
servo_fault = (status['digital_inputs'] >> 10) & 0x01  # Byte1 Bit2
print(f"ServoFAULT: {servo_fault}")
# Expected: 0 (no faults)

# Check Safety Link IN:
# Diagnostic 0x0A indicates Safety Link error
if diag == 0x0A:
    print("Safety Link chain broken - check servo drives")
```

**Expected Behavior:**
1. Power-on: All safety conditions met → diagnostic 0x1F
2. Open covers: Axes parked at home → diagnostic 0x1C (At Home), covers unlock automatically
3. Close covers, start spindle: Spindle runs, covers locked
4. Test Mode: Keyswitch + Acknowledge → covers can open when spindle stopped (J20=OPEN)
5. E-stop: Immediate power loss, diagnostic 0x10

---

## Example 2: Maintenance Configuration with Full Test Access

**Purpose:** Setup/maintenance mode allowing spindle operation with covers open

**⚠️ WARNING:** This configuration allows spindle with covers open. Only use during supervised maintenance with lockout/tagout.

**Jumper Changes from Production:**
```
J10-3 = SHORT (was OPEN) - Enable test mode spindle/cover control
J16 = 1-2 (was 2-3) - Spindle enabled in test mode
J20 = SHORT (was OPEN) - Covers open anytime in test mode
```

**All Other Jumpers:** Same as Example 1

**Wiring:** Same as Example 1

**Test Mode Operation:**
```
1. Close all covers, power system ON
2. Turn test mode keyswitch ON (CN13.1-2 closes)
3. Press and hold Acknowledge button (CN13.3-4)
4. System allows:
   - Cover opening (even with spindle running if J20=SHORT)
   - Spindle start with covers open
5. Release Acknowledge: System reverts to safe mode
```

**LDCN Commands - Test Mode Verification:**
```python
# Read test mode status:
status = sk2310.read_status()
test_mode = (status['digital_inputs'] >> 9) & 0x01  # Byte1 Bit1
print(f"Test Mode: {test_mode}")
# Expected: 1 (active)

# Verify covers open permitted:
outputs = sk2310.read_outputs()
cover_lock = (outputs >> 9) & 0x01  # Byte1 Bit1
print(f"Cover Lock: {cover_lock}")
# Expected: 0 (unlocked) even with spindle running

# Start spindle with covers open:
# (Requires test mode ON, acknowledge pressed, J16=1-2, J10-3=SHORT)
sk2310.write_output(byte0=0x04, byte1=0x00)  # Byte0 Bit2 = spindle ON
# Expected: Spindle starts successfully

# Diagnostic should remain 0x1F or appropriate state (not fault)
diag = sk2310.read_diagnostic()
print(f"Diagnostic: 0x{diag:02X}")
# Expected: NOT 0x0A (safety link error) or power-off codes
```

**Safety Procedures:**
1. **Lockout/Tagout:** Lock power disconnect, tag "Maintenance in Progress"
2. **Supervised Operation:** Qualified technician present at all times
3. **Limited Duration:** Return to production configuration when done
4. **Verification:** Test all interlocks before returning to production

**Returning to Production:**
```
1. Power OFF, open SK-2310g2 enclosure
2. Change jumpers:
   J10-3 = OPEN
   J16 = 2-3
   J20 = OPEN
3. Close enclosure, power ON
4. Test all interlocks:
   - Open covers → spindle disabled
   - E-stop → immediate shutdown
   - ServoFAULT → power-on prevented
5. Document configuration change and date
```

---

## Example 3: Dual-Zone Machine with Two SK-2310g2 Controllers

**System Configuration:**
- Zone 1 (loading): SK-2310g2 #1 (Addr 1), X/Y axes, covers, e-stops
- Zone 2 (machining): SK-2310g2 #2 (Addr 2), Z/A/B axes, spindle, covers, e-stops
- Both zones participate in Safety Link chain
- Independent cover control per zone
- Any zone unsafe = entire system disabled

**Safety Link Chain:**
```
SK-2310g2 #1 CN3.1 (OUT) ──> SK-2310g2 #2 CN3.2 (IN)
SK-2310g2 #2 CN3.1 (OUT) ──> LS-231SE X-axis CN3.2 (IN)
... (servo chain continues)
LS-231SE B-axis CN3.1 (OUT) ──> SK-2310g2 #1 CN3.2 (IN) [closes loop]
```

**Chain Logic:**
```
SK-2310g2 #1 generates Safety Link OUT based on Zone 1 conditions:
  - Zone 1 covers closed OR (Zone 1 At Home AND spindle stopped)
  - Zone 1 e-stops released

SK-2310g2 #2 receives #1 OUT, combines with Zone 2 conditions:
  - Safety Link IN from #1 = HIGH (Zone 1 safe)
  AND
  - Zone 2 covers closed OR (Zone 2 At Home AND spindle stopped)
  AND
  - Zone 2 e-stops released
  
SK-2310g2 #2 OUT ──> servo chain ──> returns to #1 IN

System operates only if: Zone 1 safe AND Zone 2 safe AND all servos safe
```

**LDCN Commands - Multi-Zone Monitoring:**
```python
# Initialize both controllers:
sk1 = network.get_device(address=1)  # Zone 1
sk2 = network.get_device(address=2)  # Zone 2

# Check Zone 1 status:
diag1 = sk1.read_diagnostic()
status1 = sk1.read_status()
safety_link_in_1 = (diag1 != 0x0A)  # 0x0A = Safety Link error

# Check Zone 2 status:
diag2 = sk2.read_diagnostic()
status2 = sk2.read_status()
safety_link_in_2 = (diag2 != 0x0A)

# System ready when both zones ready:
system_ready = (diag1 == 0x1F) and (diag2 == 0x1F)

# If either zone faults:
if diag1 == 0x0A:
    print("Zone 1 Safety Link broken - check Zone 1 or servo chain")
if diag2 == 0x0A:
    print("Zone 2 Safety Link broken - check Zone 2")

# Zone 1 covers open independently:
# Affects Zone 1 Safety Link OUT only
# Zone 2 can continue if At Home or covers closed

# Zone 2 e-stop:
# Breaks Zone 2 Safety Link OUT
# Zone 1 detects Safety Link IN = LOW → shuts down
# Entire system disabled
```

**HMI Display:**
```python
def get_system_status():
    """Multi-zone system status for HMI"""
    zone1 = {
        'diagnostic': sk1.read_diagnostic(),
        'covers_closed': check_covers(sk1),
        'at_home': (sk1.read_status()['digital_inputs'] >> 8) & 0x01,
        'estop_ok': (sk1.read_diagnostic() != 0x10),
    }
    
    zone2 = {
        'diagnostic': sk2.read_diagnostic(),
        'covers_closed': check_covers(sk2),
        'spindle_stopped': (sk2.read_status()['digital_inputs'] >> 2) & 0x01,
        'estop_ok': (sk2.read_diagnostic() != 0x10),
    }
    
    system_safe = (zone1['diagnostic'] in [0x1F, 0x1C]) and \
                  (zone2['diagnostic'] in [0x1F, 0x1C])
    
    return {
        'zone1': zone1,
        'zone2': zone2,
        'system_safe': system_safe,
        'ready_for_operation': (zone1['diagnostic'] == 0x1F) and 
                                (zone2['diagnostic'] == 0x1F)
    }
```

---

# Section 4: LDCN Commands for Safety Monitoring

This section provides complete LDCN command sequences for monitoring and controlling SK-2310g2 safety functions.

## Command Reference Summary

| Command | Hex | Function | Safety Use |
|---------|-----|----------|------------|
| CMD_READ_STATUS | 0x00 | Read digital inputs | Monitor sensors (covers, e-stop, At Home, test mode, ServoFAULT, spindle stopped) |
| CMD_READ_DIAGNOSTIC | 0x02 | Read diagnostic code | Identify current safety state (0x00-0x1F) |
| CMD_READ_OUTPUT | 0x0E | Read digital outputs | Verify spindle command, cover lock, system lock states |
| CMD_WRITE_OUTPUT | 0x1E | Write digital outputs | Control spindle, cover lock, system lock |
| CMD_HARD_RESET | 0x03 | Reset device | Clear latched faults (use with caution) |

---

## Reading Safety Status

### CMD_READ_STATUS (0x00) - Digital Inputs

**Purpose:** Read all digital inputs including safety sensors

**Command Format:**
```
Request: [Addr, 0x00, checksum]
Response: [NW, HW, Addr, Grp, stat, byte0, byte1, checksum]
```

**Safety-Relevant Bits:**

**Byte0 (Inputs 0-7):**
| Bit | Signal | Connector | Function |
|-----|--------|-----------|----------|
| 0 | Input0 | CN7.1 | General purpose |
| 1 | Input1 | CN7.2 | General purpose |
| 2 | **Spindle Stopped** | **CN6.2** | **1=Stopped, 0=Running** |
| 3 | Spindle Fault | CN6.3 | 1=Fault, 0=OK |
| 4 | Spindle At Speed | CN6.4 | 1=At Speed, 0=Accelerating |
| 5-7 | Input5-7 | CN7.6-8 | General purpose |

**Byte1 (Inputs 8-15):**
| Bit | Signal | Connector | Function |
|-----|--------|-----------|----------|
| 0 | **At Home** | **CN8** | **1=At Home, 0=Not Home** |
| 1 | **Test Mode** | **CN13.1-2** | **1=Test Mode ON, 0=OFF** |
| 2 | **ServoFAULT** | **CN3.4** | **1=Fault, 0=OK** |
| 3-7 | Input11-15 | Various | General purpose |

**Python Example:**
```python
from pyldcn import LDCNNetwork

network = LDCNNetwork('/dev/ttyUSB0')
network.open()
network.initialize()
network.set_baud_rate(125000)

sk2310 = network.devices[0]  # Address 1

# Read all inputs
status = sk2310.read_status()
inputs = status['digital_inputs']  # 16-bit word

# Extract safety bits
byte0 = inputs & 0xFF
byte1 = (inputs >> 8) & 0xFF

spindle_stopped = (byte0 >> 2) & 0x01
at_home = (byte1 >> 0) & 0x01
test_mode = (byte1 >> 1) & 0x01
servo_fault = (byte1 >> 2) & 0x01

print(f"Spindle Stopped: {spindle_stopped}")
print(f"At Home: {at_home}")
print(f"Test Mode: {test_mode}")
print(f"ServoFAULT: {servo_fault}")

# Safety check before power-on:
safe_to_power_on = (servo_fault == 0)
print(f"Safe to power on: {safe_to_power_on}")
```

---

### CMD_READ_DIAGNOSTIC (0x02) - System State

**Purpose:** Read diagnostic code indicating overall safety state

**Command Format:**
```
Request: [Addr, 0x02, checksum]
Response: [NW, HW, Addr, Grp, stat, diagnostic, checksum]
```

**Diagnostic Code Interpretation:** See Section 6 for complete table

**Python Example:**
```python
# Read diagnostic code
diag = sk2310.read_diagnostic()

# Decode state
diagnostic_states = {
    0x00: "Power OFF delay / Not ready (ServoFAULT may be active)",
    0x04: "Control voltage LOW (<18V)",
    0x0A: "Safety Link Error (chain broken)",
    0x10: "Emergency Stop active",
    0x14: "Covers open, ready to power",
    0x1C: "At Home, spindle stopped, covers open",
    0x1F: "Normal operation - fully ready"
}

state = diagnostic_states.get(diag, f"Unknown state: 0x{diag:02X}")
print(f"System State: {state}")

# Decision logic based on diagnostic:
if diag == 0x1F:
    # Fully operational
    can_run_program = True
elif diag == 0x1C:
    # At Home - can move axes but not spindle
    can_jog_axes = True
    can_run_spindle = False
elif diag == 0x14:
    # Covers open - can't operate but can power on
    can_power_on = True
elif diag == 0x10:
    # E-stop - must be released first
    display_message("Release emergency stop")
elif diag == 0x0A:
    # Safety chain broken - check servo drives
    display_message("Safety chain fault - check servo drives")
elif diag == 0x00:
    # Not ready - check ServoFAULT
    status = sk2310.read_status()
    if ((status['digital_inputs'] >> 10) & 0x01):
        display_message("Servo fault - clear drive faults")
    else:
        display_message("Power-off delay in progress")
```

---

### CMD_READ_OUTPUT (0x0E) - Output Status

**Purpose:** Read current output states (commanded, not actual physical outputs)

**Command Format:**
```
Request: [Addr, 0x0E, checksum]
Response: [NW, HW, Addr, Grp, stat, byte0, byte1, checksum]
```

**Safety-Relevant Output Bits:**

**Byte0 (Outputs 0-7):**
| Bit | Signal | Function |
|-----|--------|----------|
| 2 | **Spindle ON Command** | **Software spindle enable (requires hardware conditions)** |
| 3 | Spindle Reverse | Spindle direction control |
| 0-1, 4-7 | Output0-1, 4-7 | General purpose |

**Byte1 (Outputs 8-15):**
| Bit | Signal | Function |
|-----|--------|----------|
| 1 | **Cover Lock** | **1=Locked, 0=Unlocked** |
| 4 | **Safety Link Bridge** | **Diagnostic mode (mutually exclusive with spindle)** |
| 7 | **System Lock** | **1=Locked (prevents power-on), 0=Unlocked** |
| 0, 2-3, 5-6 | Output8, 10-11, 13-14 | General purpose |

**Python Example:**
```python
# Read outputs
outputs_data = sk2310.read_outputs()
byte0 = outputs_data & 0xFF
byte1 = (outputs_data >> 8) & 0xFF

spindle_cmd = (byte0 >> 2) & 0x01
cover_lock = (byte1 >> 1) & 0x01
safety_bridge = (byte1 >> 4) & 0x01
system_lock = (byte1 >> 7) & 0x01

print(f"Spindle Command: {spindle_cmd}")
print(f"Cover Lock: {cover_lock}")
print(f"Safety Link Bridge: {safety_bridge}")
print(f"System Lock: {system_lock}")

# Verify spindle and Safety Link Bridge mutual exclusion:
if spindle_cmd and safety_bridge:
    print("ERROR: Spindle and Safety Bridge both ON - invalid state!")
```

---

## Controlling Safety Functions

### CMD_WRITE_OUTPUT (0x1E) - Control Outputs

**Purpose:** Write digital outputs to control safety functions

**Command Format:**
```
Request: [Addr, 0x1E, byte0, byte1, checksum]
Response: [NW, HW, Addr, Grp, stat, checksum]
```

**Safety Control Examples:**

**1. Enable Spindle**
```python
# Read current outputs
current_outputs = sk2310.read_outputs()
byte0 = current_outputs & 0xFF
byte1 = (current_outputs >> 8) & 0xFF

# Set spindle command bit (Byte0 Bit2)
byte0 |= 0x04  # Set bit 2

# Ensure Safety Link Bridge is OFF (Byte1 Bit4)
byte1 &= ~0x10  # Clear bit 4

# Write outputs
sk2310.write_output(byte0=byte0, byte1=byte1)

# Verify spindle started (check hardware conditions met):
# - J16 permits spindle (covers closed or test mode)
# - ServoFAULT = 0
# - Power ON
# Actual spindle ON output (CN6.5) depends on these conditions
```

**2. Lock Covers**
```python
# Read current outputs
current_outputs = sk2310.read_outputs()
byte0 = current_outputs & 0xFF
byte1 = (current_outputs >> 8) & 0xFF

# Set cover lock bit (Byte1 Bit1)
byte1 |= 0x02  # Set bit 1

sk2310.write_output(byte0=byte0, byte1=byte1)

# With J14/J15=2-3: Lock solenoids energize, covers locked
# Verify lock:
time.sleep(0.1)  # Allow relay switching
status = sk2310.read_status()
# Check cover closed sensors if wired
```

**3. Unlock Covers (Manual)**
```python
# Clear cover lock bit (Byte1 Bit1)
byte1 &= ~0x02  # Clear bit 1

sk2310.write_output(byte0=byte0, byte1=byte1)

# Covers unlock if safety conditions permit:
# - Spindle stopped AND (Power OFF OR At Home)
# OR
# - Test Mode AND Acknowledge
```

**4. Software Power-On (J21=SHORT required)**
```python
# Step 1: Set System Lock
sk2310.write_output(byte0=0x00, byte1=0x80)  # Byte1 Bit7 = 1

# Step 2: Clear System Lock (1→0 transition triggers power-on)
sk2310.write_output(byte0=0x00, byte1=0x00)  # Byte1 Bit7 = 0

# Wait for power-on sequence
time.sleep(0.5)

# Verify power-on occurred:
diag = sk2310.read_diagnostic()
if diag == 0x1F:
    print("Power-on successful")
elif diag == 0x14 or diag == 0x1C:
    print("Power-on successful - covers open")
else:
    print(f"Power-on failed - diagnostic: 0x{diag:02X}")
```

---

## Safety Monitoring Sequences

### Pre-Operation Safety Check

**Purpose:** Verify all safety conditions before starting operation

```python
def pre_operation_check(sk2310):
    """
    Comprehensive safety check before CNC operation
    Returns: (ready: bool, message: str)
    """
    # 1. Check diagnostic code
    diag = sk2310.read_diagnostic()

    if diag == 0x10:
        return (False, "Emergency stop active - release e-stop")
    elif diag == 0x0A:
        return (False, "Safety Link chain broken - check servo drives")
    elif diag == 0x04:
        return (False, "Control voltage LOW - check 24V power supply")
    elif diag == 0x00:
        # Check if ServoFAULT is the issue
        status = sk2310.read_status()
        if ((status['digital_inputs'] >> 10) & 0x01):
            return (False, "Servo fault active - clear drive faults before power-on")
        else:
            return (False, "Power-off delay in progress - wait")

    # 2. Check for full operational state
    if diag != 0x1F:
        if diag == 0x14:
            return (False, "Covers open - close covers for operation")
        elif diag == 0x1C:
            return (False, "At Home with covers open - close covers for full operation")
        else:
            return (False, f"System not ready - diagnostic: 0x{diag:02X}")

    # 3. Verify ServoFAULT clear
    status = sk2310.read_status()
    servo_fault = (status['digital_inputs'] >> 10) & 0x01
    if servo_fault:
        return (False, "ServoFAULT input HIGH - check servo drive status")

    # 4. Verify covers closed (if At Home not active)
    at_home = (status['digital_inputs'] >> 8) & 0x01
    if not at_home:
        # Covers should be closed for normal operation
        # (Cover sensors would need to be read if wired to digital inputs)
        pass

    # 5. All checks passed
    return (True, "System ready for operation")


# Usage:
ready, message = pre_operation_check(sk2310)
if ready:
    print("✓ Safety check passed - ready to run program")
else:
    print(f"✗ Safety check failed: {message}")
    # Display message to operator, wait for condition resolution
```

### Continuous Safety Monitoring

**Purpose:** Monitor safety status during operation

```python
import threading
import time

class SafetyMonitor:
    def __init__(self, sk2310, callback):
        self.sk2310 = sk2310
        self.callback = callback  # Function to call on safety fault
        self.running = False
        self.last_diag = None
        self.thread = None

    def start(self):
        """Start monitoring thread"""
        self.running = True
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop monitoring thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)

    def _monitor_loop(self):
        """Continuous monitoring loop"""
        while self.running:
            try:
                # Read diagnostic code
                diag = self.sk2310.read_diagnostic()

                # Detect state changes
                if diag != self.last_diag:
                    self._handle_state_change(self.last_diag, diag)
                    self.last_diag = diag

                # Check for critical faults
                if diag in [0x10, 0x0A]:
                    # E-stop or Safety Link fault - immediate notification
                    self.callback('critical_fault', diag)

                # Read inputs for additional monitoring
                status = self.sk2310.read_status()
                servo_fault = (status['digital_inputs'] >> 10) & 0x01

                if servo_fault and diag == 0x1F:
                    # ServoFAULT active but system still powered
                    # (Fault occurred during operation)
                    self.callback('servo_fault_during_operation', status)

            except Exception as e:
                self.callback('communication_error', str(e))

            time.sleep(0.1)  # 10Hz monitoring rate

    def _handle_state_change(self, old_diag, new_diag):
        """Handle diagnostic code changes"""
        changes = {
            (0x1F, 0x1C): "Covers opened during operation (At Home)",
            (0x1F, 0x14): "Covers opened (not At Home - should not happen)",
            (0x1F, 0x10): "Emergency stop activated",
            (0x1F, 0x0A): "Safety Link chain broken",
            (0x1C, 0x1F): "Covers closed, full operation restored",
            (0x14, 0x1F): "Covers closed, system operational",
        }

        key = (old_diag, new_diag)
        if key in changes:
            self.callback('state_change', changes[key])


# Usage:
def safety_fault_handler(event_type, data):
    if event_type == 'critical_fault':
        print(f"CRITICAL FAULT: Diagnostic 0x{data:02X}")
        # Stop all motion, display alarm
        emergency_stop_motion()
    elif event_type == 'servo_fault_during_operation':
        print("Servo fault detected during operation")
        # Pause program, investigate fault
        pause_operation()
    elif event_type == 'state_change':
        print(f"Safety state changed: {data}")
        # Update HMI display
        update_status_display(data)
    elif event_type == 'communication_error':
        print(f"Communication error: {data}")
        # Handle LDCN communication loss

monitor = SafetyMonitor(sk2310, safety_fault_handler)
monitor.start()

# ... run CNC program ...

monitor.stop()
```

---

# Section 5: Status Monitoring Integration

This section provides guidance for integrating SK-2310g2 safety status into HMI displays and operator interfaces.

## Operator Interface Requirements

**Minimum Required Display Elements:**
1. **System State** - Text description of diagnostic code
2. **E-Stop Status** - Clear indication if e-stop active
3. **Cover Status** - Locked/Unlocked, Closed/Open
4. **Servo Fault** - Indication if any drive has fault
5. **Safety Link** - Chain integrity status
6. **Power State** - ON/OFF
7. **Test Mode** - Active/Inactive

**Recommended Additional Elements:**
8. **At Home** - Safe zone indicator
9. **Spindle Status** - Running/Stopped, Commanded state
10. **Fault History** - Log of recent safety events

---

## HMI Status Display Example

### Complete Status Structure

```python
class SK2310g2Status:
    """Complete status representation for HMI display"""

    def __init__(self, sk2310):
        self.sk2310 = sk2310
        self.update()

    def update(self):
        """Read all status from device"""
        try:
            self.diagnostic = self.sk2310.read_diagnostic()
            status_data = self.sk2310.read_status()
            self.inputs = status_data['digital_inputs']
            outputs_data = self.sk2310.read_outputs()
            self.outputs = outputs_data

            self.communication_ok = True
        except Exception as e:
            self.communication_ok = False
            self.error = str(e)

    @property
    def system_state_text(self):
        """Human-readable system state"""
        states = {
            0x00: "Power OFF / Not Ready",
            0x04: "Control Voltage LOW",
            0x0A: "Safety Link Error",
            0x10: "Emergency Stop Active",
            0x14: "Covers Open - Ready to Power",
            0x1C: "At Home - Covers Open",
            0x1F: "Normal Operation",
        }
        return states.get(self.diagnostic, f"Unknown (0x{self.diagnostic:02X})")

    @property
    def estop_active(self):
        """Emergency stop status"""
        return self.diagnostic == 0x10

    @property
    def safety_link_ok(self):
        """Safety Link chain integrity"""
        return self.diagnostic != 0x0A

    @property
    def servo_fault(self):
        """Servo drive fault status"""
        return ((self.inputs >> 10) & 0x01) == 1

    @property
    def at_home(self):
        """At Home (safe zone) status"""
        return ((self.inputs >> 8) & 0x01) == 1

    @property
    def test_mode(self):
        """Test mode active"""
        return ((self.inputs >> 9) & 0x01) == 1

    @property
    def spindle_stopped(self):
        """Spindle stopped sensor"""
        return ((self.inputs >> 2) & 0x01) == 1

    @property
    def spindle_commanded(self):
        """Software spindle command state"""
        return ((self.outputs >> 2) & 0x01) == 1

    @property
    def cover_lock_commanded(self):
        """Cover lock output state"""
        return ((self.outputs >> 9) & 0x01) == 1

    @property
    def power_on(self):
        """System powered on"""
        return self.diagnostic in [0x1C, 0x1F]

    @property
    def ready_for_operation(self):
        """Fully ready for CNC operation"""
        return self.diagnostic == 0x1F and not self.servo_fault

    def to_dict(self):
        """Export status as dictionary for HMI"""
        return {
            'communication_ok': self.communication_ok,
            'system_state': self.system_state_text,
            'diagnostic_code': f"0x{self.diagnostic:02X}",
            'estop_active': self.estop_active,
            'safety_link_ok': self.safety_link_ok,
            'servo_fault': self.servo_fault,
            'at_home': self.at_home,
            'test_mode': self.test_mode,
            'spindle_stopped': self.spindle_stopped,
            'spindle_commanded': self.spindle_commanded,
            'cover_lock': self.cover_lock_commanded,
            'power_on': self.power_on,
            'ready': self.ready_for_operation,
        }


# Usage example:
status = SK2310g2Status(sk2310)

# Update periodically (e.g., 5Hz)
while True:
    status.update()

    # Display on HMI
    print(f"System: {status.system_state_text}")
    print(f"Ready: {'YES' if status.ready_for_operation else 'NO'}")
    print(f"E-Stop: {'ACTIVE' if status.estop_active else 'OK'}")
    print(f"Servo Fault: {'YES' if status.servo_fault else 'NO'}")

    time.sleep(0.2)
```

### Status LED Panel Example

```python
class StatusLEDPanel:
    """
    Represents physical LED panel or GUI equivalent
    Maps SK-2310g2 status to LED indicators
    """

    def __init__(self, status: SK2310g2Status):
        self.status = status

    def get_led_states(self):
        """
        Returns LED states for panel
        Format: {led_name: (color, state)}
        state: 'on', 'off', 'blink'
        """
        leds = {}

        # Power LED
        if self.status.power_on:
            leds['power'] = ('green', 'on')
        else:
            leds['power'] = ('red', 'off')

        # E-Stop LED
        if self.status.estop_active:
            leds['estop'] = ('red', 'blink')
        else:
            leds['estop'] = ('green', 'on')

        # Servo Fault LED
        if self.status.servo_fault:
            leds['servo_fault'] = ('red', 'on')
        else:
            leds['servo_fault'] = ('green', 'off')

        # Safety Link LED
        if not self.status.safety_link_ok:
            leds['safety_link'] = ('red', 'blink')
        else:
            leds['safety_link'] = ('green', 'on')

        # Ready LED
        if self.status.ready_for_operation:
            leds['ready'] = ('green', 'on')
        elif self.status.power_on:
            leds['ready'] = ('yellow', 'on')
        else:
            leds['ready'] = ('red', 'off')

        # Test Mode LED
        if self.status.test_mode:
            leds['test_mode'] = ('yellow', 'blink')
        else:
            leds['test_mode'] = ('green', 'off')

        # At Home LED
        if self.status.at_home:
            leds['at_home'] = ('blue', 'on')
        else:
            leds['at_home'] = ('blue', 'off')

        return leds


# Example HMI update function:
def update_hmi_display():
    status = SK2310g2Status(sk2310)
    status.update()

    panel = StatusLEDPanel(status)
    leds = panel.get_led_states()

    # Update GUI or physical LEDs
    for led_name, (color, state) in leds.items():
        set_led(led_name, color, state)

    # Update text displays
    set_text_display('system_state', status.system_state_text)
    set_text_display('diagnostic', status.to_dict()['diagnostic_code'])
```

---

## Alarm Management

### Fault Priority Levels

```python
class FaultPriority:
    CRITICAL = 0  # Immediate stop, operator intervention required
    HIGH = 1      # Stop at safe point, investigate
    MEDIUM = 2    # Warning, log event, continue
    LOW = 3       # Informational

class SafetyAlarm:
    """Safety alarm with priority and handling"""

    def __init__(self, code, priority, message, action):
        self.code = code
        self.priority = priority
        self.message = message
        self.action = action  # Required operator action
        self.timestamp = time.time()

    def __str__(self):
        priority_text = ['CRITICAL', 'HIGH', 'MEDIUM', 'LOW'][self.priority]
        return f"[{priority_text}] {self.message} - Action: {self.action}"


class AlarmManager:
    """Manages safety alarms and fault history"""

    # Alarm definitions
    ALARMS = {
        0x10: SafetyAlarm(
            0x10, FaultPriority.CRITICAL,
            "Emergency Stop Active",
            "Release e-stop button and press reset"
        ),
        0x0A: SafetyAlarm(
            0x0A, FaultPriority.CRITICAL,
            "Safety Link Chain Broken",
            "Check servo drive faults, verify wiring"
        ),
        0x04: SafetyAlarm(
            0x04, FaultPriority.HIGH,
            "Control Voltage LOW (<18V)",
            "Check 24V power supply"
        ),
        'servo_fault': SafetyAlarm(
            None, FaultPriority.HIGH,
            "Servo Drive Fault",
            "Check servo drive status via LDCN, clear faults"
        ),
    }

    def __init__(self):
        self.active_alarms = {}
        self.alarm_history = []

    def check_status(self, status: SK2310g2Status):
        """Check status and generate alarms"""
        new_alarms = {}

        # Check diagnostic code
        if status.diagnostic in self.ALARMS:
            alarm = self.ALARMS[status.diagnostic]
            new_alarms[alarm.code] = alarm

        # Check servo fault
        if status.servo_fault:
            alarm = self.ALARMS['servo_fault']
            new_alarms['servo_fault'] = alarm

        # Detect new alarms
        for code, alarm in new_alarms.items():
            if code not in self.active_alarms:
                self._activate_alarm(alarm)

        # Detect cleared alarms
        for code in list(self.active_alarms.keys()):
            if code not in new_alarms:
                self._clear_alarm(code)

        self.active_alarms = new_alarms

    def _activate_alarm(self, alarm):
        """Activate new alarm"""
        print(f"ALARM ACTIVATED: {alarm}")
        self.alarm_history.append({
            'alarm': alarm,
            'activated_at': time.time(),
            'cleared_at': None,
        })

        # Trigger appropriate response based on priority
        if alarm.priority == FaultPriority.CRITICAL:
            self._handle_critical_alarm(alarm)
        elif alarm.priority == FaultPriority.HIGH:
            self._handle_high_alarm(alarm)

    def _clear_alarm(self, code):
        """Clear active alarm"""
        print(f"ALARM CLEARED: {code}")
        # Update history
        for entry in reversed(self.alarm_history):
            if entry['alarm'].code == code and entry['cleared_at'] is None:
                entry['cleared_at'] = time.time()
                break

    def _handle_critical_alarm(self, alarm):
        """Handle critical priority alarm"""
        # Emergency stop all motion
        emergency_stop_motion()
        # Display prominent alarm message
        display_critical_alarm(alarm.message, alarm.action)
        # Sound alarm
        sound_alarm_buzzer()

    def _handle_high_alarm(self, alarm):
        """Handle high priority alarm"""
        # Stop motion at safe point
        controlled_stop_motion()
        # Display alarm message
        display_alarm(alarm.message, alarm.action)

    def get_active_alarms(self):
        """Get list of active alarms sorted by priority"""
        alarms = list(self.active_alarms.values())
        alarms.sort(key=lambda a: a.priority)
        return alarms

    def get_alarm_history(self, limit=100):
        """Get recent alarm history"""
        return self.alarm_history[-limit:]


# Usage:
alarm_mgr = AlarmManager()

while True:
    status.update()
    alarm_mgr.check_status(status)

    # Display active alarms on HMI
    active_alarms = alarm_mgr.get_active_alarms()
    if active_alarms:
        print(f"{len(active_alarms)} active alarms:")
        for alarm in active_alarms:
            print(f"  {alarm}")

    time.sleep(0.2)
```

---

# Section 6: Diagnostic Code Interpretation

Complete reference for all SK-2310g2 diagnostic codes.

## Diagnostic Code Table

| Code | Binary | State | E-Stop | Covers | At Home | Spindle | Meaning | Actions Required |
|------|--------|-------|--------|--------|---------|---------|---------|------------------|
| **0x00** | 00000 | **Power OFF / Not Ready** | Released | X | X | X | Power-off delay in progress OR ServoFAULT active | Wait for delay OR clear servo faults |
| 0x01 | 00001 | Initializing | Released | X | X | X | Brief startup state | Wait |
| 0x02 | 00010 | Control voltage shorted | Released | X | X | X | 24V supply fault | Check power supply wiring |
| 0x03 | 00011 | Output shorted | Released | X | X | X | Output circuit fault | Check output loads |
| **0x04** | 00100 | **Control voltage LOW** | Released | X | X | X | 24V supply <18V | Check 24V power supply |
| 0x05 | 00101 | Home switch malfunction | Released | X | X | X | Both At Home contacts ON | Check CN8 wiring |
| 0x06 | 00110 | Test mode switch malfunction | Released | X | X | X | Both Test Mode contacts ON | Check CN13.1-2 wiring |
| 0x07 | 00111 | Power UP Home error | Released | X | X | X | At Home sensor error at startup | Check At Home sensor |
| **0x08** | 01000 | **System LOCKED** | Released | X | X | X | Software lock active (Byte1/Bit7=1) | Clear System Lock via LDCN |
| 0x09 | 01001 | Watchdog Stop | Released | X | X | X | Watchdog timer expired (if J1-5=SHORT) | Reset device |
| **0x0A** | 01010 | **Safety Link Error** | Released | X | X | X | Safety Link IN = LOW (chain broken) | Check servo drives, verify wiring |
| **0x10** | 10000 | **Emergency Stop active** | **Active** | X | X | X | E-stop button pressed | Release e-stop, press reset |
| **0x14** | 10100 | **Covers open, ready to power** | Released | **Open** | No | X | Normal state with covers open | Close covers OR power on for At Home |
| **0x1C** | 11100 | **At Home, spindle stopped, covers open** | Released | **Open** | **Yes** | **Stopped** | Safe zone - covers can be open | Normal for tool changes |
| **0x1F** | 11111 | **Normal operation - ready** | Released | **Closed** | No | X | Fully operational | None - ready to run |

### Diagnostic Bit Decoding

**Diagnostic Code Format:** `Bit4 Bit3 Bit2 Bit1 Bit0`

| Bit | Meaning When = 1 | Meaning When = 0 |
|-----|------------------|------------------|
| Bit 0 | E-stop released / condition met | E-stop active / condition not met |
| Bit 1 | Covers closed | Covers open |
| Bit 2 | At Home (safe zone) | Not at home |
| Bit 3 | Spindle stopped | Spindle running (or don't care) |
| Bit 4 | Power ready / specific condition | Power not ready / specific condition |

**Note:** Bits indicate safety conditions, but exact interpretation varies by code. Use table above for accurate meaning.

---

## Diagnostic Code Decision Tree

```
Read Diagnostic Code
│
├─ 0x10 → E-Stop Active
│         Action: Release e-stop buttons
│
├─ 0x0A → Safety Link Error
│         Action: Walk safety chain, find broken link
│         → Check servo drive faults via LDCN
│         → Verify CN3 wiring
│
├─ 0x04 → Control Voltage LOW
│         Action: Measure 24V supply (CN1.2)
│         → Check for voltage drop under load
│         → Verify power supply capacity
│
├─ 0x08 → System LOCKED
│         Action: Read Outputs/Byte1/Bit7
│         → Clear System Lock via LDCN (write 0)
│
├─ 0x00 → Power OFF / Not Ready
│         Action: Check ServoFAULT (Inputs/Byte1/Bit2)
│         → If HIGH: Clear servo drive faults
│         → If LOW: Wait for J2 power-off delay (1-4 sec)
│
├─ 0x14 → Covers Open, Ready
│         Action: Close covers for full operation
│         OR
│         → Power on for At Home access
│
├─ 0x1C → At Home, Covers Open
│         Action: Normal for tool changes
│         → Close covers to return to full operation
│         → Can jog axes, cannot run spindle
│
└─ 0x1F → Normal Operation
          Action: None - system fully ready
```

---

# Section 7: Troubleshooting by Symptom

Systematic troubleshooting procedures using LDCN commands and physical checks.

## Symptom: Power Button Not Flashing (Won't Power On)

**Diagnostic:** Read diagnostic code = 0x00

**Meaning:** Power-off delay OR ServoFAULT preventing power-on

**Troubleshooting Sequence:**

```python
# Step 1: Read diagnostic
diag = sk2310.read_diagnostic()
print(f"Diagnostic: 0x{diag:02X}")
# Expected: 0x00

# Step 2: Check ServoFAULT input
status = sk2310.read_status()
servo_fault = (status['digital_inputs'] >> 10) & 0x01
print(f"ServoFAULT (Inputs/Byte1/Bit2): {servo_fault}")

if servo_fault == 1:
    print("Root cause: Servo drive fault preventing power-on")
    print("Actions:")
    print("  1. Query each servo drive via LDCN")
    print("  2. Check for limit switch faults")
    print("  3. Home all axes")
    print("  4. Clear drive faults")

    # Query servo drives
    for addr in [2, 3, 4]:  # X, Y, Z axes
        try:
            servo = network.get_device(address=addr)
            servo_status = servo.read_status()
            print(f"  Servo {addr}: status = 0x{servo_status:04X}")
            # Check for fault flags in status byte
        except:
            print(f"  Servo {addr}: Communication error")

else:
    print("Root cause: Power-off delay in progress")
    print("Actions:")
    print("  1. Wait for J2 delay (check jumper: 1/2/4 seconds)")
    print("  2. Retry power-on after delay")
```

**Common Resolutions:**
- Servo not homed: Home all axes, clear limit faults
- Servo amplifier disabled: Enable amplifier via servo drive commands
- Servo following error: Reset position error counters

---

## Symptom: Diagnostic 0x0A (Safety Link Error)

**Meaning:** Safety Link IN (CN3.2) is LOW - daisy chain broken

**Troubleshooting Sequence:**

```python
# Step 1: Verify SK-2310g2 Safety Link OUT is HIGH
# (Can't read directly via LDCN - must measure at CN3.1)
# If LOW: SK-2310g2 sees unsafe condition (covers, e-stop, etc.)

# Step 2: Check diagnostic for e-stop or cover issues
diag = sk2310.read_diagnostic()
if diag == 0x10:
    print("E-stop is active - release e-stop first")
    # Safety Link OUT will be LOW if e-stop active

# Step 3: Walk the safety chain
print("Walking safety chain...")
print("1. Disconnect CN3.2 (Safety Link IN) at SK-2310g2")
print("2. Measure voltage at last device CN3.1 (OUT)")
print("3. If 24V: Wiring break between last device and SK-2310g2")
print("4. If 0V: Walk backward through chain")

# Step 4: Query each servo drive for faults
for addr in [2, 3, 4]:
    servo = network.get_device(address=addr)
    servo_status = servo.read_status()

    # Check Safety Link / Enable status bit (bit 4)
    safety_ok = (servo_status >> 4) & 0x01
    print(f"Servo {addr}: Safety Link status = {safety_ok}")
    # 0 = Chain broken or drive disabled
    # 1 = Chain intact and drive enabled

    if not safety_ok:
        print(f"  → Servo {addr} breaking chain")
        print(f"  → Check drive status, limits, amplifier")
```

**Systematic Chain Walk:**
```
1. SK-2310g2 CN3.1 (OUT) → Measure voltage
   - 0V: SK-2310g2 problem (covers, e-stop)
   - 24V: Continue

2. Servo 1 CN3.2 (IN) → Should be 24V
   - 0V: Wiring break before Servo 1
   - 24V: Continue

3. Servo 1 CN3.1 (OUT) → Measure voltage
   - 0V: Servo 1 fault
   - 24V: Servo 1 OK, continue

4. Servo 2 CN3.2 (IN) → Should be 24V
   ...repeat for all devices
```

**Common Resolutions:**
- Loose CN3 connector: Reseat connectors
- Servo drive fault: Clear drive faults, check limits
- Wiring damage: Repair/replace safety chain cable

---

## Symptom: Spindle Won't Start (Diagnostic 0x1F)

**Meaning:** System reports ready but spindle doesn't start

**Troubleshooting Sequence:**

```python
# Step 1: Verify software spindle command
outputs = sk2310.read_outputs()
spindle_cmd = (outputs >> 2) & 0x01
print(f"Spindle Command (Outputs/Byte0/Bit2): {spindle_cmd}")

if spindle_cmd == 0:
    print("Software spindle command not sent")
    print("Action: Send spindle ON command")
    sk2310.write_output(byte0=0x04, byte1=0x00)

# Step 2: Check Safety Link Bridge mutual exclusion
safety_bridge = (outputs >> 12) & 0x01
print(f"Safety Link Bridge (Outputs/Byte1/Bit4): {safety_bridge}")

if safety_bridge == 1:
    print("ERROR: Safety Link Bridge is ON")
    print("Spindle and Safety Bridge are mutually exclusive")
    print("Action: Clear Safety Link Bridge")
    byte1 = (outputs >> 8) & 0xFF
    byte1 &= ~0x10  # Clear bit 4
    sk2310.write_output(byte0=(outputs & 0xFF), byte1=byte1)

# Step 3: Check ServoFAULT
status = sk2310.read_status()
servo_fault = (status['digital_inputs'] >> 10) & 0x01
print(f"ServoFAULT (Inputs/Byte1/Bit2): {servo_fault}")

if servo_fault == 1:
    print("Servo fault active - prevents spindle enable")
    print("Action: Clear servo drive faults")

# Step 4: Check covers (depends on J16 setting)
print("Check jumper J16:")
print("  J16 = 2-3: Spindle disabled with covers open")
print("  J16 = 1-2: Spindle enabled in test mode")

# Step 5: Measure actual spindle enable output
print("Measure CN6.5 (Spindle Enable) voltage:")
print("  Expected: 24V if all conditions met")
print("  If 0V: Hardware interlock preventing enable")
```

**Decision Matrix:**

| Spindle Cmd | Safety Br | ServoFAULT | Covers | J16 | Expected Result |
|-------------|-----------|------------|--------|-----|-----------------|
| 1 | 0 | 0 | Closed | 2-3 | Spindle ON |
| 1 | 0 | 0 | Open | 2-3 | Spindle OFF |
| 1 | 0 | 0 | Open | 1-2 | Spindle ON (if test mode) |
| 1 | 1 | 0 | Closed | X | Spindle OFF (mutual exclusion) |
| 1 | 0 | 1 | Closed | X | Spindle OFF (servo fault) |

---

## Symptom: Covers Won't Unlock

**Diagnostic:** May be 0x1F, 0x14, or 0x1C

**Troubleshooting Sequence:**

```python
# Step 1: Check At Home status
status = sk2310.read_status()
at_home = (status['digital_inputs'] >> 8) & 0x01
spindle_stopped = (status['digital_inputs'] >> 2) & 0x01
test_mode = (status['digital_inputs'] >> 9) & 0x01

print(f"At Home (Inputs/Byte1/Bit0): {at_home}")
print(f"Spindle Stopped (Inputs/Byte0/Bit2): {spindle_stopped}")
print(f"Test Mode (Inputs/Byte1/Bit1): {test_mode}")

# Step 2: Check cover lock command
outputs = sk2310.read_outputs()
cover_lock = (outputs >> 9) & 0x01
print(f"Cover Lock (Outputs/Byte1/Bit1): {cover_lock}")

if cover_lock == 1:
    print("Covers are commanded LOCKED")
    print("Action: Clear cover lock")
    byte1 = (outputs >> 8) & 0xFF
    byte1 &= ~0x02  # Clear bit 1
    sk2310.write_output(byte0=(outputs & 0xFF), byte1=byte1)

    # Wait and recheck
    time.sleep(0.2)
    status = sk2310.read_status()
    # Check if unlock conditions met

# Step 3: Check unlock conditions based on jumper settings
print("Verify unlock conditions (jumper-dependent):")
print("Standard operation (J19=OPEN or SHORT):")
print("  - Spindle Stopped AND (Power OFF OR At Home)")
print("Test Mode (J10-3=SHORT, J20 setting):")
print("  - J20=OPEN: Unlock when spindle stopped")
print("  - J20=SHORT: Unlock anytime with Acknowledge")

# Step 4: If J19=OPEN (manual mode), check unlock button
print("If J19=OPEN: Unlock button (CN13.5-6) must be pressed")
print("If J19=SHORT: Automatic unlock (check J10-2)")

# Step 5: Verify conditions met
if not spindle_stopped:
    print("Spindle must be stopped for unlock (check J20)")
    print("Action: Stop spindle, wait for stopped signal")

if not at_home and not test_mode:
    print("Must be At Home or Test Mode for unlock")
    print("Action: Move to home position OR enter test mode")
```

---

# Section 8: Configuration Recipes

Pre-validated jumper configurations for common machine types.

## Recipe 1: Standard Production CNC

**Use Case:** General-purpose CNC with normal safety requirements

**Machine Type:** 3-4 axis mill, router, or machining center

**Safety Philosophy:** Maximum safety, no spindle with covers open

**Jumper Settings:**
```
J2-2 = SHORT                # 2-second power-off delay
J2-4 = OFF                  # Monitor motor power
J10-1 = SHORT               # Standard At Home detection
J10-2 = SHORT               # Enable cover automation
J10-3 = OPEN                # Restricted test mode
J10-4 = SHORT               # Enable Zero Speed At Home
J14/J15 = 2-3               # Powered to lock covers
J16 = 2-3                   # Spindle DISABLED with covers open
J17 = 2-3                   # Unlock Enable to CN13.7
J19 = SHORT                 # Automatic cover control
J20 = OPEN                  # Covers open only when spindle stopped
J21 = SHORT                 # Software power-on enabled
```

**Behavior:**
- ✓ Covers unlock automatically when At Home
- ✓ Spindle never runs with covers open
- ✓ Test mode available but highly restricted
- ✓ Servo motion allowed with covers open if At Home
- ✓ Suitable for unsupervised operation

**LDCN Integration:**
```python
def configure_standard_production(sk2310):
    """Software configuration for Recipe 1"""
    # Set initial outputs
    sk2310.write_output(
        byte0=0x00,  # Spindle OFF, all outputs OFF
        byte1=0x02   # Covers locked (Bit1=1)
    )

    # Verify configuration
    diag = sk2310.read_diagnostic()
    if diag == 0x00:
        print("Awaiting power-on (check ServoFAULT)")
    elif diag == 0x14:
        print("Ready to power (covers open)")
    elif diag == 0x1F:
        print("Fully operational")
```

---

## Recipe 2: Maintenance/Setup Mode

**Use Case:** Machine setup, troubleshooting, supervised maintenance

**Machine Type:** Any CNC requiring full test access

**Safety Philosophy:** Full access with dual-action controls (keyswitch + button)

⚠️ **WARNING:** This configuration allows spindle operation with covers open. Only use during supervised maintenance with lockout/tagout.

**Jumper Settings:**
```
J2-2 = SHORT                # 2-second power-off delay
J2-4 = OFF                  # Monitor motor power
J10-1 = SHORT               # Standard At Home detection
J10-2 = SHORT               # Enable cover automation
J10-3 = SHORT               # FULL test mode control
J10-4 = SHORT               # Enable Zero Speed At Home
J14/J15 = 2-3               # Powered to lock covers
J16 = 1-2                   # Spindle ENABLED in test mode
J17 = 2-3                   # Unlock Enable to CN13.7
J19 = SHORT                 # Automatic cover control
J20 = SHORT                 # Covers open anytime in test mode
J21 = SHORT                 # Software power-on enabled
```

**Behavior:**
- ⚠️ Spindle CAN run with covers open in test mode
- ⚠️ Covers can open anytime in test mode
- ✓ Requires keyswitch AND acknowledge (dual-action)
- ✓ Maximum flexibility for diagnostics

**Safety Procedures:**
1. Lock out main power disconnect, tag "Maintenance in Progress"
2. Turn test mode keyswitch ON (CN13.1-2)
3. Press and hold Acknowledge button (CN13.3-4) during risky operations
4. Return to Recipe 1 when maintenance complete

**Returning to Production:**
```bash
# Physical jumper changes required:
J10-3: SHORT → OPEN
J16: 1-2 → 2-3
J20: SHORT → OPEN

# Power cycle after jumper changes
# Test all interlocks before production
```

---

## Recipe 3: Load/Unload Station (No Spindle)

**Use Case:** Loading station, part handling, no spindle

**Machine Type:** Gantry loader, pick-and-place, material handling

**Safety Philosophy:** Simple cover interlocks, no spindle considerations

**Jumper Settings:**
```
J2-2 = SHORT                # 2-second power-off delay
J2-4 = OFF                  # Monitor motor power
J10-1 = SHORT               # Standard At Home detection
J10-2 = SHORT               # Enable cover automation
J10-3 = OPEN                # Restricted test mode (not critical)
J10-4 = SHORT               # Enable Zero Speed At Home
J14/J15 = 2-3               # Powered to lock covers
J16 = 2-3                   # (Not used - no spindle)
J17 = 2-3                   # Unlock Enable to CN13.7
J19 = SHORT                 # Automatic cover control
J20 = OPEN                  # (Not critical - no spindle)
J21 = SHORT                 # Software power-on enabled
```

**Modifications:**
- CN6 (Spindle interface) not connected
- Inputs/Byte0/Bit2 (Spindle Stopped) tied HIGH (24V via resistor)
- At Home detection critical for safe loading/unloading

**LDCN Integration:**
```python
def loading_station_cycle(sk2310):
    """Automatic load/unload cycle"""
    # 1. Move to home position
    move_to_home()

    # 2. Wait for At Home
    while True:
        status = sk2310.read_status()
        at_home = (status['digital_inputs'] >> 8) & 0x01
        if at_home:
            break
        time.sleep(0.1)

    # 3. Covers unlock automatically (J19=SHORT, J10-2=SHORT)
    time.sleep(0.5)  # Allow unlock relay to settle

    # 4. Operator loads/unloads part
    # ... wait for load complete signal ...

    # 5. Close covers (sensor or command)
    # 6. Covers lock automatically when motion starts
    start_next_cycle()
```

---

## Recipe 4: Educational/Training Machine

**Use Case:** Vocational training, educational CNC

**Machine Type:** Small training CNC with frequent cover opening

**Safety Philosophy:** Balance between safety and learning access

**Jumper Settings:**
```
J2-2 = SHORT                # 2-second power-off delay
J10-1 = SHORT               # Standard At Home detection
J10-2 = OPEN                # MANUAL cover unlock (teaching procedure)
J10-3 = OPEN                # Restricted test mode
J10-4 = SHORT               # Enable Zero Speed At Home
J14/J15 = 2-3               # Powered to lock covers
J16 = 2-3                   # Spindle DISABLED with covers open
J17 = 2-3                   # Unlock Enable to CN13.7
J19 = OPEN                  # MANUAL unlock button required
J20 = OPEN                  # Covers open only when spindle stopped
J21 = OPEN                  # Power button only (no software)
```

**Behavior:**
- Students must press unlock button (teaches proper procedure)
- No automatic cover operations (explicit actions required)
- Spindle never runs with covers open
- Physical power button only (no software remote power)

**Training Emphasis:**
1. Proper shutdown procedure
2. Manual cover unlock at safe conditions
3. Visual confirmation before cover opening
4. Emergency stop recognition and response

---

# Section 9: Testing Procedures

Validation procedures for commissioning and maintenance.

## Initial Commissioning Test Sequence

**Prerequisites:**
- All wiring completed per schematics
- Jumpers configured per selected recipe
- 24V power supply verified (22-28V)
- No motors connected (for initial safety testing)

**Test 1: Power Supply and Communication**

```python
# 1. Apply 24V power to SK-2310g2
# 2. Connect LDCN interface

from pyldcn import LDCNNetwork

network = LDCNNetwork('/dev/ttyUSB0')
network.open()
network.initialize()  # Should discover SK-2310g2

sk2310 = network.devices[0]
print(f"Found: {sk2310.device_type} at address {sk2310.address}")

# 3. Read diagnostic
diag = sk2310.read_diagnostic()
print(f"Diagnostic: 0x{diag:02X}")

# Expected: 0x10 (e-stop active) or 0x14 (covers open)
# If 0x04: Check 24V voltage (must be ≥18V)

# PASS CRITERIA: Communication successful, reasonable diagnostic code
```

**Test 2: Emergency Stop Circuit**

```python
# 1. Release all e-stop buttons
# 2. Read diagnostic
diag = sk2310.read_diagnostic()
print(f"E-stop released diagnostic: 0x{diag:02X}")
# Expected: NOT 0x10

# 3. Press e-stop at each location
estop_locations = ['Main panel', 'Side panel', 'Pendant', 'Door 1', 'Door 2']
for location in estop_locations:
    input(f"Press e-stop at {location}, then press Enter")
    diag = sk2310.read_diagnostic()
    print(f"  Diagnostic: 0x{diag:02X}")
    assert diag == 0x10, f"E-stop at {location} not detected!"
    print(f"  ✓ {location} e-stop working")

    input("Release e-stop, then press Enter")
    time.sleep(0.2)
    diag = sk2310.read_diagnostic()
    assert diag != 0x10, f"E-stop at {location} not releasing!"
    print(f"  ✓ {location} e-stop releases")

print("✓ ALL E-STOP TESTS PASSED")
```

**Test 3: Cover Interlocks**

```python
# 1. Close all covers
# 2. Power on system
diag = sk2310.read_diagnostic()
print(f"Covers closed diagnostic: 0x{diag:02X}")
# Expected: 0x1F (normal operation)

# 3. Open each cover
covers = ['Main cover', 'Side cover']
for cover in covers:
    input(f"Open {cover}, then press Enter")
    diag = sk2310.read_diagnostic()
    print(f"  Diagnostic: 0x{diag:02X}")

    # Expected: 0x14 or 0x1C (depending on At Home)
    assert diag in [0x14, 0x1C], f"{cover} opening not detected!"
    print(f"  ✓ {cover} opening detected")

    input(f"Close {cover}, then press Enter")
    time.sleep(0.2)
    diag = sk2310.read_diagnostic()
    assert diag == 0x1F, f"{cover} closing not detected!"
    print(f"  ✓ {cover} closing detected")

print("✓ ALL COVER TESTS PASSED")
```

**Test 4: Safety Link Chain (with servo drives connected)**

```python
# 1. Verify chain intact
status = sk2310.read_status()
diag = sk2310.read_diagnostic()
print(f"Diagnostic: 0x{diag:02X}")
assert diag != 0x0A, "Safety Link chain broken before test!"

# 2. Disconnect Safety Link return at SK-2310g2 CN3.2
input("Disconnect CN3.2 at SK-2310g2, then press Enter")
time.sleep(0.2)
diag = sk2310.read_diagnostic()
print(f"Diagnostic with chain open: 0x{diag:02X}")
assert diag == 0x0A, "Safety Link chain break not detected!"
print("✓ Safety Link monitoring working")

input("Reconnect CN3.2, then press Enter")
time.sleep(0.2)
diag = sk2310.read_diagnostic()
assert diag != 0x0A, "Safety Link not restored!"
print("✓ Safety Link chain restored")

# 3. Trigger servo fault at each drive
for addr in [2, 3, 4]:  # X, Y, Z axes
    print(f"\nTesting servo drive at address {addr}:")
    input(f"  Trigger fault at servo {addr} (e.g., drive to limit), press Enter")

    time.sleep(0.5)
    diag = sk2310.read_diagnostic()
    print(f"  Diagnostic: 0x{diag:02X}")
    assert diag == 0x0A, f"Servo {addr} fault not breaking chain!"
    print(f"  ✓ Servo {addr} breaks chain when faulted")

    input("  Clear fault, then press Enter")
    time.sleep(0.5)
    diag = sk2310.read_diagnostic()
    assert diag != 0x0A, f"Servo {addr} fault not clearing!"
    print(f"  ✓ Servo {addr} chain restored")

print("\n✓ ALL SAFETY LINK TESTS PASSED")
```

**Test 5: ServoFAULT Input**

```python
# 1. Verify ServoFAULT clear
status = sk2310.read_status()
servo_fault = (status['digital_inputs'] >> 10) & 0x01
print(f"ServoFAULT initial: {servo_fault}")
assert servo_fault == 0, "ServoFAULT active before test!"

# 2. Power OFF
# (Power off command or physical button)

# 3. Trigger servo fault
input("Trigger servo fault (limit switch, etc.), then press Enter")
time.sleep(0.5)
status = sk2310.read_status()
servo_fault = (status['digital_inputs'] >> 10) & 0x01
print(f"ServoFAULT with fault: {servo_fault}")
assert servo_fault == 1, "ServoFAULT not detected!"
print("✓ ServoFAULT input detecting faults")

# 4. Attempt power-on (should fail)
# (Power button or software command)
time.sleep(2.0)  # Wait for power-off delay
diag = sk2310.read_diagnostic()
print(f"Diagnostic with ServoFAULT: 0x{diag:02X}")
assert diag == 0x00, "Power-on not prevented by ServoFAULT!"
print("✓ ServoFAULT prevents power-on")

# 5. Clear fault
input("Clear servo fault, then press Enter")
time.sleep(0.5)
status = sk2310.read_status()
servo_fault = (status['digital_inputs'] >> 10) & 0x01
assert servo_fault == 0, "ServoFAULT not clearing!"
print("✓ ServoFAULT clears after fault resolution")

print("\n✓ ALL SERVOFAULT TESTS PASSED")
```

---

## Periodic Maintenance Testing

**Frequency:** Monthly or after any safety-related maintenance

**Test Checklist:**

```python
def monthly_safety_test(sk2310):
    """Automated monthly safety validation"""
    results = {
        'communication': False,
        'diagnostic_response': False,
        'estop': False,
        'cover_detect': False,
        'safety_link': False,
        'servo_fault': False,
    }

    # 1. Communication test
    try:
        diag = sk2310.read_diagnostic()
        results['communication'] = True
        results['diagnostic_response'] = (diag != 0xFF)
    except:
        return results  # Fail - can't continue

    # 2. E-stop test (one location)
    print("Activate any e-stop...")
    start = time.time()
    while time.sleep - start < 10.0:
        diag = sk2310.read_diagnostic()
        if diag == 0x10:
            results['estop'] = True
            print("✓ E-stop detected")
            break
        time.sleep(0.2)

    input("Release e-stop and press Enter")
    time.sleep(0.5)
    diag = sk2310.read_diagnostic()
    results['estop'] = results['estop'] and (diag != 0x10)

    # 3. Cover detection test
    print("Open any cover...")
    start = time.time()
    while time.time() - start < 10.0:
        diag = sk2310.read_diagnostic()
        if diag in [0x14, 0x1C]:
            results['cover_detect'] = True
            print("✓ Cover opening detected")
            break
        time.sleep(0.2)

    input("Close cover and press Enter")

    # 4. Safety Link test
    status = sk2310.read_status()
    diag = sk2310.read_diagnostic()
    results['safety_link'] = (diag != 0x0A)

    # 5. ServoFAULT test
    servo_fault = (status['digital_inputs'] >> 10) & 0x01
    results['servo_fault'] = (servo_fault == 0)  # Should be clear

    # Generate report
    print("\n=== MONTHLY SAFETY TEST RESULTS ===")
    for test, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {test}: {status}")

    all_passed = all(results.values())
    print(f"\nOverall: {'PASS' if all_passed else 'FAIL'}")

    return results


# Run monthly test
results = monthly_safety_test(sk2310)
# Log results to maintenance database
```

---

## Related Documentation

- **Safety Bus Specification:** `safety_bus.md` - Generic LDCN Safety Bus interface
- **SK-2310g2 Homing:** `sk2310g2_homing_application.md` - At Home sensor implementation
- **Servo Commands:** `servo_commands.md` - LS-231SE fault conditions
- **I/O Commands:** `io_commands.md` - General I/O device commands
- **LDCN Protocol:** `protocol.md` - Low-level protocol reference

---

## References

**CNC-SK-2310g2 Supervisor I/O Controller Manual**
- Document: Doc # 710231005 / Rev. D
- Date: 03/05/2020
- Content Used:
  - Pages 4-5: Jumper reference table (J1-J21)
  - Pages 6-14: Connector pinouts and signal descriptions
  - Page 9: CN3 Safety Bus detailed specifications
  - Pages 10-14: Digital I/O tables, safety logic, truth tables
  - Page 15: Spindle enable logic, Safety Link Bridge mutual exclusion
  - Page 20: Diagnostic code table (0x00-0x1F)
  - Pages 16-22: Sample applications, jumper configuration examples
  - Page 21: At Home detection modes, J10 jumper effects

**Multi-Axis CNC Servo Controller**
- Document: Doc # 714000001 / Rev. F
- Date: 03/25/2011
- Content Used:
  - Generic Safety Bus topology and signal specifications
  - Servo drive Safety Link pass-through behavior

**LS-231SE Advanced Multifunctional Servo Drive Datasheet**
- Content Used:
  - Safety Link participant role
  - Fault conditions triggering chain break

---

*Document Version: 1.0*
*Last Updated: 2025-11-05*
*Document Type: Device-Specific Implementation Guide*
*Target Audiences: System Designers, Software Developers, Technicians*
