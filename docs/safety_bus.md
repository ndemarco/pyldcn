# Logosol Safety Bus

This document describes the **Safety Bus** interface specification used across Logosol devices for implementing safety interlocks in CNC machine control applications.

**Primary Source:** Multi-Axis CNC Servo Controller Doc # 714000001 / Rev. F, 03/25/2011

---

## Overview

The Safety Bus is a **daisy-chain safety interlock system** that connects multiple Logosol devices to create a coordinated safety chain. All devices in the chain must report safe conditions for the system to operate. It complements the Logisol Device Control Network (LDCN), delivering a reliable, fail-secure connection between safety critical devices. The Safety Bus reacts immediately but simply to a safety event. LDCN can later deliver richer information about the event.

**Key Safety Features:**

- 100% relay contact-based implementation
- Fail-safe design (open circuit = unsafe condition)
- Coordinated emergency stop across all devices
- Power supply and spindle control interlocks
- Servo fault monitoring

**Devices with Safety Bus Support:**

- **SK-2310g2** - Supervisor I/O Controller (typically safety master)
- **LS-231SE** - Servo Drive (safety chain participant)
- **LS-2315** - Spindle Drive (safety interface)
- **Others** - Any device with compatible signals can participate in safety chain

---

## Purpose and Limitations

### Purpose

The Safety Bus provides:

1. **Coordinated E-Stop** - Single emergency stop affects all devices simultaneously
1. **Cover Interlock Distribution** - Cover safety status propagates through entire system
1. **Fault Isolation** - Any device fault can disable entire system
1. **Fail-Safe Operation** - Open circuit equals unsafe condition
1. **Simplified Wiring** - Daisy-chain reduces point-to-point wiring complexity

### Limitations

1. **Not a Communication Bus** - Safety Bus is relay-based only. LDCN carries richer data
1. **No Fault Localization** - Fault localization requires diagnosis over LDCN via status requests
1. **Propagation Delay** - Relay switching time affects response (typically <50ms per device)
1. **Master Device Required** - One device must generate Safety Link OUT based on system conditions

---

## Safety Bus Connector Specification

**Connector Type:** 4-pin (check device documentation)

### Pin Definitions

CLAUDE: Confirm 4 - ServoFAULT is truly input/output. I believe it is output for all devices except the supervising device.

| Pin | Signal | Type | Direction | Electrical Spec |
|-----|--------|------|-----------|-----------------|
| 1 | Safety Link OUT | Output | Device → Next device | Relay contact, 24Vdc, 0.15A max |
| 2 | Safety Link IN | Input | Previous device → Device | 24Vdc monitoring |
| 3 | Enable/Stop | Output | Device → Loads | Relay contact, 24Vdc, 0.15A max |
| 4 | ServoFAULT | Input/Output | Device-specific | Digital I/O, 24Vdc |

---

## Signal Descriptions

### Pin 1 - Safety Link OUT

**Function:** Safety Bus source output to next device in chain

**Signal Type:** Relay contact output, 24Vdc, 0.15A max

**Electrical Characteristics:**

- HIGH = Relay contact closed, 24V present
- LOW = Relay contact open, 0V (or floating)
- Switching time: Typically 10-50ms (relay dependent)
- Contact rating: Minimum 0.15A @ 24Vdc

**Behavior by Device Role:**

**Safety Master (e.g., SK-2310g2):**

- Generates Safety Link OUT based on system safety conditions
- Monitors covers, emergency stops, safe zones, test modes
- Complex logic (device-specific - see device documentation)

**Safety Participant (e.g., LS-231SE):**

- If no device faults, passes Safety Link IN to Safety Link OUT
- Simple AND gate: OUT = IN && (no local faults)

**Use Case:** Propagates safety chain integrity to downstream devices

---

### Pin 2 - Safety Link IN

**Function:** Safety Bus status from upstream device in chain

**Signal Type:** Digital input, 24Vdc monitoring

**Electrical Characteristics:**

- HIGH (OK): 18-30Vdc nominal
- LOW (FAULT): open circuite (<5Vdc)
- Input impedance: Typically 10-100kΩ (device-specific)

**Logic:**

```text
Safety Link IN = HIGH (OK):
  - Safety chain is intact
  - All upstream devices report safe conditions
  - System operation permitted

Safety Link IN = LOW (OPEN - FAULT):
  - Safety chain is broken
  - One or more upstream devices experiencing unsafe conditions
  - System power and spindle power must be immediately disabled
```text

**Critical Behavior:**

- **Fail-safe design:** Open circuit (broken wire) = FAULT condition
- **Immediate response:** LOW input triggers emergency shutdown
- **No latching:** Returns to normal when chain restored (device-dependent)

**Use Case:** Monitors safety status of all upstream devices in chain

---

### Pin 3 - Enable/Stop

**Function:** System enable/stop line source

**Signal Type:** Relay contact output, 24Vdc, 0.15A max

**Electrical Characteristics:**

- HIGH: Relay contact closed, 24V present
- LOW: Relay contact open, 0V
- Switching time: Typically 10-50ms

**Typical Logic (device-dependent):**

```text
Enable/Stop = HIGH when:
  - Power is ON
  - No stop conditions active
  - Safety Link IN = HIGH (if device monitors it)

Enable/Stop = OPEN (LOW) when:
  - Emergency stop active
  - Power OFF
  - Safety interlock open
  - System fault condition
```text

**Typical Use Cases:**

- Enable signal to servo amplifier, spindle drive
- Power supply contactor control
- External safety device monitoring
- Interlock with machine safety systems

---

### Pin 4 - ServoFAULT

**Function:** Servo fault monitoring

**Signal Type:** Digital I/O, 24Vdc

**Roles:**

**As INPUT (e.g., SK-2310g2):**

- Monitors servo drive fault outputs (OR'd together)
- HIGH = One or more servo drives has fault
- LOW = All drives operating normally
- Used to prevent power-on when servo faults exist

**As OUTPUT (e.g., LS-231SE):**

- Reports servo drive fault status
- HIGH = Drive has fault (overload, position error, limit)
- LOW = Drive operating normally
- Can be OR'd with other drives to single fault input

**Example Faults Reported:**

- Motor overload/overcurrent
- Following error / position error
- Limit switch activation
- Encoder fault
- Amplifier disable/fault
- Communication timeout

**Use Case:** Centralized health monitoring of multiple drives

---

## Safety Bus Topology

### Daisy-Chain Architecture

The Safety Bus implements a **series safety chain** where all devices must be safe for system operation.

```text
     ┌─────────────┐         ┌─────────────┐         ┌─────────────┐
     │  Master     │         │ Participant │         │ Participant │
     │  Device     │         │  Device 1   │         │  Device 2   │
     │             │         │             │         │             │
     │ CN3:        │         │ CN3:        │         │ CN3:        │
     │  Pin1 OUT ──┼────────>│  Pin2 IN    │         │  Pin2 IN    │
  ┌──│  Pin2 IN    │         │  Pin1 OUT ──┼────────>│  Pin1 OUT   │──┐
  │  │  Pin3 Enable│         │  Pin3 Enable│         │  Pin3 Enable│  │
  │  └─────────────┘         └─────────────┘         └─────────────┘  │
  │                                                                     │
  └─────────────────────────── Return Loop ───────────────────────────┘
```text

**Signal Flow:**

1. Master device generates Safety Link OUT based on system conditions
2. Signal propagates through Participant 1 (passes through if no faults)
3. Signal propagates through Participant 2 (passes through if no faults)
4. Signal returns to Master Safety Link IN
5. If loop is complete (all HIGH), system is safe to operate

**Logical Function:**

```text
System Safe = Master Conditions AND Device1 Safe AND Device2 Safe AND ... AND DeviceN Safe
```text

This implements a fail-safe **logical AND** of all safety conditions.

---

### Fault Propagation

**Scenario: Device 2 Develops Fault**

```text
Master OUT (HIGH) ──> Device1 IN (HIGH) ──> Device1 OUT (HIGH)
                      ──> Device2 IN (HIGH) ──> Device2 OUT (LOW - FAULT!)
                      ──> Master IN (LOW) = CHAIN BROKEN
```text

**Result:**

- Master detects LOW on Safety Link IN
- Master immediately disables:
  - Power Enable output
  - Spindle Enable output
  - Enable/Stop outputs
- All devices lose Enable/Stop signal
- System enters safe state

**Recovery:**

- Fault must be cleared at Device 2
CLAUDE: Confirm if all faults must be reset in addition to clearing the fault cause.
- Device 2 OUT returns to HIGH
- Master IN returns to HIGH
- System can be restarted (power button, e-stop reset, etc.)

---

### Multiple Master Configuration

Some systems may have multiple safety controllers (e.g., multiple SK-2310g2 units for multi-zone machines).

```text
     ┌──────────┐       ┌──────────┐       ┌──────────┐
     │ Master A │       │ Master B │       │ Servo 1  │
     │  Zone 1  │       │  Zone 2  │       │          │
     │ OUT ─────┼──────>│ IN       │       │ IN       │
  ┌──│ IN       │       │ OUT ─────┼──────>│ OUT      │──┐
  │  └──────────┘       └──────────┘       └──────────┘  │
  │                                                        │
  └────────────────────── Return ─────────────────────────┘
```text

**Operation:**

- Master A generates Safety Link based on Zone 1 conditions
- Master B receives it, combines with Zone 2 conditions, regenerates OUT
- Both zones must be safe for system operation
- Each master can monitor its own diagnostic code

---

## Device Roles and Responsibilities

### Safety Master (Supervisor)

**Typical Device:** SK-2310g2, custom I/O controller

**Responsibilities:**

- Generate Safety Link OUT based on system safety conditions:
  - Emergency stop monitoring (multiple locations)
  - Cover/guard position monitoring
  - Safe zone (At Home) detection
  - Test mode control
  - Operator controls (enable switches)
- Monitor Safety Link IN for chain integrity
- Control power contactors via Enable/Stop or dedicated outputs
- Monitor ServoFAULT input from all drives
- Report diagnostic codes for troubleshooting
- Provide operator interface (buttons, LEDs, displays)

**Example Logic (device-specific):**

```text
Safety Link OUT = Covers Closed OR (At Home AND Spindle Stopped) OR Test Mode
```text

---

### Safety Participant (Servo Drive)

**Typical Device:** LS-231SE

**Responsibilities:**

- Pass through Safety Link IN to OUT if no drive faults
- Break chain (OUT = LOW) if local fault occurs:
  - Motor overload
  - Position error exceeds threshold
  - Limit switch activated
  - Encoder fault
  - Amplifier disabled
- Report fault on CN3 Pin 4 (ServoFAULT output)
- Disable motor when Enable/Stop = LOW
- Maintain position tracking during safe stop

**Logic:**

```text
Safety Link OUT = Safety Link IN AND (No Drive Faults) AND (Amplifier Enabled)
ServoFAULT = Any Drive Fault Active
```text

---

### Safety Participant (Spindle Drive)

**Typical Device:** LS-2315

**Responsibilities:**

- Monitor Enable/Stop input (or dedicated spindle enable)
- Report spindle stopped status to master
- Report spindle fault to master
- May participate in Safety Link chain (application-dependent)
- Safe stop on loss of enable

**Typical Connections:**

- Enable input ← from Master spindle enable output
- Stopped output → to Master spindle stopped input
- Fault output → to Master spindle fault input

---

## Related Documentation

- **SK-2310g2 Safety Implementation:** `sk2310g2_safety.md` - Complete SK-2310g2 safety controller guide with jumper settings, wiring examples, and configuration
- **Servo Commands:** `servo_commands.md` - LS-231SE fault conditions and status monitoring
- **I/O Commands:** `io_commands.md` - General I/O device command reference
- **LDCN Protocol:** `protocol.md` - Low-level LDCN communication protocol

---

## References

**Multi-Axis CNC Servo Controller**

- Document: Doc # 714000001 / Rev. F
- Date: 03/25/2011
- Content Used:
  - Page 9: CN3 connector pinout and signal descriptions
  - Page 14: Sample application - Spindle control with safety interlocks
  - Page 16: Sample application - Emergency stop and power control
  - Page 19: Digital I/O table showing safety signal mappings

**LS-231SE Advanced Multifunctional Servo Drive Datasheet**

- Content Used:
  - CN3 Safety Bus connector specifications
  - Safety Link pass-through logic
  - Fault output specifications and triggering conditions
  - Amplifier enable input behavior

**LS-2315 High-Performance Spindle Drive**

- Content Used:
  - Spindle safety interface specifications
  - Enable/Stop input requirements
  - Spindle Stopped and Fault output signals

---
