# CNC-SK-2310g2 Homing Sample Application

**Source:** Doc # 710231005 / Rev. D, Page 13
**Date:** 2025-10-30

---

## Overview

The homing sample application demonstrates the proper wiring and configuration for a **Safety Zone Sensor** (home switch) used with the CNC-SK-2310g2 Supervisor I/O Controller. This implementation uses a dual-contact safety home switch to detect when the machine is in a safe position for cover opening and operator access.

---

## Safety Home Switch Concept

The home sensor uses a **dual-contact safety switch** design to provide redundant position detection:

- **Contact A (Normally Closed)**: Closed when machine is in Safety Zone
- **Contact B (Normally Open)**: Open when machine is in Safety Zone

This redundant contact design ensures fail-safe operation - both contacts must transition properly to validate the home position.

---

## Hardware Components

### CN8 Connector - HOME (Safety Zone Sensor)

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | Home A1 | Input - Home sensor contact A, pin 1. Closed in Safety Zone |
| 2 | Home A2 | Input - Home sensor contact A, pin 2 |
| 3 | Home B1 | Input - Home sensor contact B, pin 1. Open in Safety Zone |
| 4 | Home B2 | Input - Home sensor contact B, pin 2 |

**Critical Timing Requirement:**
Time for transfer from Contact A=OPEN to Contact B=CLOSED must be **less than 100msec**

### Safety Home Switch

The dual-contact "Safety Home" switch has the following contact states:

**In Safety Zone (At Home):**
- Contact A: CLOSED
- Contact B: OPEN

**Outside Safety Zone (Not At Home):**
- Contact A: OPEN
- Contact B: CLOSED

### Diagnostic LEDs

The system provides three diagnostic LEDs connected through jumper headers:

- **LED-1**: Controlled by jumper J11
-
- **LED-3**: Controlled by jumper J12 (connected to CN3 Safety Bus)

### Optional H1 Delay Circuit

An optional delay circuit (labeled "H1 DELAY" in the diagram) can be added to the home sensor circuit to provide:
- Debouncing of mechanical switch contacts
- Timing delay for contact transition validation
- Additional filtering of transient signals

---

## Jumper Configuration

This section documents all jumper settings based on analysis of sample application schematics (pages 11-22).

### J10 - Safety Mode and Cover Control (4-position header)

**Location:** Near CN8 (HOME connector) and safety control logic

J10 is a 4-position jumper header that controls fundamental safety behavior. Each pin can be configured independently with a shorting jumper.

#### J10-1 and J10-4: Zero Speed Mode Control (Pins 1 and 4)

These two jumpers work together to select the "At Home" detection method:

**Configuration Option 1: Home Sensor Mode (Standard)**
- J10-1: OPEN
- J10-4: OPEN
- **Behavior**: "At Home" status determined by physical home switch (CN8)
- **Use case**: CNC machines with defined safe zone position

**Configuration Option 2: Zero Speed Mode - Immediate**
- J10-1: SHORT
- J10-4: SHORT
- **Behavior**: System is "At Home" when:
  - "Zero Speed" signal is ON (all servo drives stopped >2 sec), AND
  - Spindle is stopped (Input2/Byte0/Bit2 = 1), AND
  - Output9/Byte1/Bit1 = 0 (Covers not locked)
- **Use case**: Automated systems without fixed safe zone

**Configuration Option 3: Zero Speed Mode - Latched**
- J10-1: OPEN
- J10-4: SHORT
- **Behavior**: System is "At Home" when:
  - "Zero Speed" signal is ON, AND
  - Spindle is stopped, AND
  - After Output9 transitions from 1→0 (software controlled unlock)
- **Use case**: Systems requiring explicit software permission to unlock

**Reference:** Page 21, "ZERO SPEED Automation Grade Safety"

#### J10-2: Cover Lock/Unlock Button Control (Pin 2)

Controls whether separate Lock and Unlock buttons are used for cover control.

**J10-2 OPEN (Default):**
- Single unlock button on CN13 controls cover unlock
- Covers lock automatically when conditions not met
- **Use case**: Simple cover control with single unlock button

**J10-2 SHORT:**
- Separate Lock and Unlock buttons on CN22 control covers
- Requires J19 SHORT to enable this mode
- Manual control of both lock and unlock actions
- **Use case**: Systems requiring explicit lock/unlock commands

**Reference:** Page 5, J10 description

#### J10-3: Spindle Enable in Test Mode (Pin 3)

Controls whether spindle can operate in Test Mode.

**J10-3 OPEN (Safe Mode):**
- Spindle CANNOT be enabled in Test Mode
- Covers CANNOT be opened unless spindle is stopped
- **Safety level**: High - spindle and cover opening are mutually exclusive
- **Use case**: High-risk spindle operations (grinding, high-speed machining)
- **Sample application**: Page 11, Spindle Control Option 1

**J10-3 SHORT (Maintenance Mode):**
- Spindle CAN be enabled in Test Mode with Acknowledge
- Covers can be opened in Test Mode with Acknowledge (if J20 SHORT)
- **Safety level**: Lower - allows spindle operation with covers open during testing
- **Use case**: Setup, testing, low-speed spindle operation with safeguards
- **Sample application**: Page 12, Spindle Control Option 2

**Reference:** Pages 11-12, Spindle Control Options

---

### J16 - Spindle ON Output Control (3-position selector)

**Location:** Near spindle control logic

J16 is a 3-position jumper (pins 1-2-3) that selects spindle enable behavior in different operating modes.

**J16 Position 2-3 SHORT (Safe Operation Mode):**
- Spindle ON output **DISABLED** when covers are open
- Spindle Enable output **CANNOT** be turned ON in Test Mode
- If spindle is already ON when Test Mode is activated, spindle state is preserved
- **Safety level**: High
- **Use case**: Normal production operation
- **Sample application**: Page 11, Spindle Control Option 1

**J16 Position 1-2 SHORT (Maintenance/Test Mode):**
- Spindle ON output **ENABLED** in Test Mode with Acknowledge
- Allows spindle operation with covers open during testing
- Must be combined with J10-3 SHORT and J20 SHORT for full functionality
- **Safety level**: Lower - requires operator acknowledgment
- **Use case**: Setup, testing, troubleshooting
- **Sample application**: Page 12, Spindle Control Option 2

**Logic:**
```
Spindle ON output HIGH when:
  J16 2-3: (Outputs/Byte0/Bit2=1) AND (Safety Link Bridge=0) AND
           (Power ON) AND (Covers closed)

  J16 1-2: (Outputs/Byte0/Bit2=1) AND (Safety Link Bridge=0) AND
           (Power ON) AND [(Covers closed) OR (Test Mode + Acknowledge)]
```

**Reference:** Pages 5, 11-12

---

### J20 - Test Mode Cover Unlock Conditions (2-position)

**Location:** Near cover control logic

J20 controls whether spindle must be stopped before covers can be unlocked in Test Mode.

**J20 OPEN (Spindle Stop Required):**
- Covers can be unlocked in Test Mode with Acknowledge **ONLY when Spindle is stopped**
- Automatic unlock after spindle stops (if other conditions met)
- **Safety level**: Higher
- **Use case**: Systems where spindle must always stop before access
- **Sample application**: Page 11, Spindle Control Option 1

**J20 SHORT (Test Mode Access):**
- Covers can be unlocked in Test Mode with Acknowledge **regardless of spindle state**
- Must be combined with J16 1-2 and J10-3 SHORT for spindle-on access
- **Safety level**: Lower - allows access while spindle may be running
- **Use case**: Low-speed testing, setup with rotating spindle
- **Sample application**: Page 12, Spindle Control Option 2

**Reference:** Pages 5, 11-12

---

### J17 - Unlock Enable Output Routing (3-position selector)

**Location:** Near CN13 (ACKNOWLEDGE AND COVER UNLOCK)

J17 routes the negative side of the Unlock Enable output.

**J17 Position 1-2 SHORT:**
- CN13 pin 7 connected to **GND**
- Unlock Enable output has fixed ground reference
- **Use case**: Simple unlock circuits with external control

**J17 Position 2-3 SHORT (Recommended):**
- CN13 pin 7 connected to **Unlock Enable output (negative)**
- Provides controlled negative side of unlock power
- Enables proper unlock solenoid drive
- **Use case**: Standard cover unlock implementation
- **Sample application**: Page 16, Acknowledge and Cover Unlock

**Reference:** Pages 5, 16

---

### J19 - Cover Lock/Unlock Control Mode (2-position)

**Location:** Near cover control logic

J19 selects between manual switch control and automatic cover unlock.

**J19 OPEN (Automatic Mode):**
- Cover Lock/Unlock controlled by:
  - Unlock switch connected to CN13 pins 5-6, AND
  - Unlock Enable output state
- **Use case**: Semi-automatic cover control with unlock button

**J19 SHORT (Manual Mode):**
- Cover Lock/Unlock controlled by Unlock Enable output only
- **Requires J10-2 SHORT** for separate Lock/Unlock buttons on CN22
- Alternative: Output14 (Byte1/Bit6) can be used for software control
- **Use case**: Fully manual or fully automatic cover control

**Reference:** Page 5

---

### J21 - Power ON Control Method (2-position)

**Location:** Near CN15 (MODE AND POWER ON)

J21 enables software control of power-on function.

**J21 OPEN (Button Only):**
- Power can only be turned ON by physical Power ON button
- Software cannot initiate power-on sequence
- **Use case**: Manual operation only

**J21 SHORT (Button or Software):**
- Power can be turned ON by:
  - Physical Power ON button, OR
  - Software: Byte1/Bit7 transition from "1" to "0" when Power is OFF
- **Use case**: Automated power control, remote operation

**Reference:** Page 5

---

### J14 and J15 - Cover 2 Lock Solenoid Polarity (3-position selector each)

**Location:** Near CN10 (Cover 2 control)

J14 and J15 are identical selectors that control the polarity of Cover 2 lock solenoid drive. Both jumpers should be set to the same position.

**J14/J15 Position 1-2 SHORT:**
- CN10 Lock output pins **powered when Door is unlocked**
- Solenoid energized to pull cover latch open
- De-energized when locked (spring returns latch)
- **Use case**: Spring-return lock with unlock solenoid

**J14/J15 Position 2-3 SHORT:**
- CN10 Lock output pins **powered when Door is locked**
- Solenoid energized to pull cover latch closed
- De-energized when unlocked
- **Use case**: Electromagnetic lock or positive-lock solenoid

**Note:** Cover 1 does not have polarity selection jumpers; its behavior is fixed.

**Reference:** Page 5

---

### J11 - Home Sensor and LED Configuration (Multi-pin header)

**Location:** Near CN8 (HOME connector) and diagnostic LEDs

J11 is a multi-position jumper header that configures home sensor signal routing and LED-1/LED-2 diagnostic outputs. Based on schematic analysis (page 13), this jumper:

- Routes home sensor contact signals to internal logic
- Controls LED-1 and LED-2 status indicators
- Provides signal conditioning for home sensor inputs

**Configuration:** "See Sample Applications CNC-SK-2310g2" (Page 5)

**Typical settings:**
- Home sensor mode: Specific J11 positions route CN8 contacts to home detection logic
- Zero Speed mode: Different J11 configuration routes Zero Speed signal
- LED outputs: Control which system states drive LED-1 and LED-2

**Visible from schematics (Page 13):**
- Connected to home sensor circuit (CN8)
- Connected to LED-1 and LED-2
- Part of safety state monitoring system
- Multiple pin positions for different routing options

**Reference:** Pages 5, 13

---

### J12 - Home Sensor and LED-3 Configuration (Multi-pin header)

**Location:** Near CN3 (SAFETY BUS) and LED-3

J12 is a multi-position jumper header that configures additional home sensor functionality and LED-3 diagnostic output. Based on schematic analysis (page 13), this jumper:

- Routes safety bus signals
- Controls LED-3 status indicator
- Provides additional home sensor signal conditioning

**Configuration:** "See Sample Applications CNC-SK-2310g2" (Page 5)

**Visible from schematics (Page 13):**
- Connected to CN3 Safety Bus
- Connected to LED-3
- Part of home sensor validation circuit
- Coordinates with J11 for complete home detection

**Reference:** Pages 5, 13

---

### J6 and J7 - Analog Input Protection Resistors (2-position each)

**Location:** Near CN17 (ANALOG INPUTS)

These jumpers control protective resistors on the analog input power rails.

**J6 - Positive Rail (+5V or +POT):**
- **OPEN**: 100Ω protective resistor connected between +5V and CN17 pin4
- **SHORT**: Direct connection to +5V (bypass resistor)

**J7 - Negative Rail (GND or -POT):**
- **OPEN**: 100Ω protective resistor connected between GND and CN17 pin1
- **SHORT**: Direct connection to GND (bypass resistor)

**Use case:**
- OPEN: Protection for external potentiometers against short circuits
- SHORT: Direct connection for low-impedance analog sources

**Reference:** Pages 5, 10

---

### J5 - Power Control Pin Routing (Multi-position header)

**Location:** Near CN16 (POWER CONTROL)

J5 is a multi-position jumper block that routes various power control signals to CN16 multifunction pins.

#### CN16 Pin 3 Routing (J5 positions 5-6, 6-7, 7-8, 8-9):

| Jumper Position | CN16 Pin 3 Connected To |
|----------------|------------------------|
| Open (default) | Not connected |
| J5 5-6 SHORT | GND |
| J5 6-7 SHORT | CN16 pin 8 (Power Enable) |
| J5 7-8 SHORT | CN16 pin 7 |
| J5 8-9 SHORT | CN16 pin 10 (Spindle ON) |

#### CN16 Pin 7 Routing (J5 positions 1-2, 2-3, 3-4):

| Jumper Position | CN16 Pin 7 Connected To |
|----------------|------------------------|
| Open (default) | Not connected |
| J5 1-2 SHORT | CN16 pin 4 (Power Control B) |
| J5 2-3 SHORT | CN16 pin 3 |
| J5 3-4 SHORT | GND |

**Use case:** Flexible routing of power control signals to match external relay/contactor wiring requirements.

**Reference:** Page 5

---

### J4 - CN4 Pin 6 Power Output (2-position)

**Location:** Near CN4 (LDCN SLAVE)

J4 controls whether +24V power is provided on CN4 pin 6 (LDCN SLAVE connector).

**J4 OPEN:**
- CN4 pin 6 not connected
- No power output to LDCN slave devices
- **Use case**: External power supply for slave devices

**J4 SHORT:**
- CN4 pin 6 connected to +24V
- Provides power to daisy-chained LDCN devices
- **Use case**: Power LDCN network from SK-2310g2

**Reference:** Page 4

---

### J18 - Reserved (must remain OPEN)

**Location:** On main board

**J18 OPEN (Required):**
- Reserved for future use
- Must remain OPEN per manufacturer specification

**Reference:** Page 5

---

## Jumper Configuration Summary Tables

### Typical Configuration: Safe Production Mode

| Jumper | Position | Function |
|--------|----------|----------|
| J10-1 | OPEN | Home sensor mode (not Zero Speed) |
| J10-2 | OPEN | Single unlock button mode |
| J10-3 | OPEN | Spindle disabled when covers open |
| J10-4 | OPEN | Home sensor mode (not Zero Speed) |
| J16 | 2-3 SHORT | Spindle OFF when covers open |
| J17 | 2-3 SHORT | Unlock Enable output to CN13 pin 7 |
| J19 | OPEN | Unlock switch + Enable control |
| J20 | OPEN | Covers unlock only when spindle stopped |
| J21 | OPEN | Power ON button only (no software control) |

**Use case:** Normal production operation with maximum safety interlocks.

---

### Typical Configuration: Test/Maintenance Mode

| Jumper | Position | Function |
|--------|----------|----------|
| J10-1 | OPEN | Home sensor mode |
| J10-2 | OPEN | Single unlock button mode |
| J10-3 | SHORT | Spindle enabled in Test Mode |
| J10-4 | OPEN | Home sensor mode |
| J16 | 1-2 SHORT | Spindle ON allowed with covers open in Test Mode |
| J17 | 2-3 SHORT | Unlock Enable output to CN13 pin 7 |
| J19 | OPEN | Unlock switch + Enable control |
| J20 | SHORT | Covers can open in Test Mode (even with spindle) |
| J21 | SHORT | Power ON via button or software |

**Use case:** Setup, testing, troubleshooting with controlled spindle access.

---

### Typical Configuration: Zero Speed Automation Mode

| Jumper | Position | Function |
|--------|----------|----------|
| J10-1 | SHORT | Zero Speed mode - immediate |
| J10-2 | OPEN | Single unlock button mode |
| J10-3 | OPEN | Spindle disabled when covers open |
| J10-4 | SHORT | Zero Speed mode - immediate |
| J16 | 2-3 SHORT | Spindle OFF when covers open |
| J17 | 2-3 SHORT | Unlock Enable output to CN13 pin 7 |
| J19 | SHORT | Automatic unlock via Enable output |
| J20 | OPEN | Covers unlock only when spindle stopped |
| J21 | SHORT | Power ON via button or software |

**Use case:** Automated systems without fixed home position, using motor stop detection.

**Reference:** Pages 21-22, Zero Speed Automation Mode

---

### J11 and J12 - Home Sensor Configuration

Controls LED-1, LED-2, LED-3 and home sensor signal routing (see page 5 for details).

**Note:** Page 5 states "See Sample Applications CNC-SK-2310g2" for all J11 and J12 settings. The exact pin configurations are application-specific and should be set according to the sample application schematics on pages 13, 21-22.

---

## Wiring Diagram Description

The sample application on page 13 shows:

1. **Safety Home Switch**: Dual-contact switch with separate A and B contact pairs
2. **CN8 Connection**: 4-pin connector to CNC-SK-2310g2
   - Pins 1-2: Contact A circuit
   - Pins 3-4: Contact B circuit
3. **24V Power**: Provided from CNC-SK-2310g2 to power the home switch circuit
4. **LED Indicators**: Three diagnostic LEDs (J11, CN3, J12) for status monitoring
5. **Optional H1 Delay**: Timing circuit for contact validation
6. **Home Error Output**: Signal line indicating home sensor fault condition

---

## Contact Transition Validation

The CNC-SK-2310g2 monitors both contact pairs and validates the transition timing:

**Valid Home Entry Sequence:**
1. Machine moves toward home position
2. Contact A transitions: OPEN → CLOSED
3. Within 100msec, Contact B must transition: CLOSED → OPEN
4. If timing is correct, "At Home" status is set
5. If timing exceeds 100msec, home sensor fault is triggered

**Valid Home Exit Sequence:**
1. Machine moves away from home position
2. Contact A transitions: CLOSED → OPEN
3. Within 100msec, Contact B must transition: OPEN → CLOSED
4. "At Home" status is cleared

---

## Home Sensor Fault Detection

The following conditions trigger a home sensor fault:

| Fault Condition | Description | Diagnostic Code |
|----------------|-------------|-----------------|
| Both contacts ON | Both A and B contacts are closed simultaneously | 0x05 |
| Timing violation | Transition time between contacts exceeds 100msec | 0x05 |
| Power UP Home error | Home sensor not in valid state at power-up | 0x06 |
| Contact malfunction | One or both contacts stuck or not responding | Varies |

**Diagnostic Code 0x05:**
- Byte1 bits [7:3] = `00101`
- LED pattern: LED5=ON, LED4=ON, LED3=OFF, LED2=ON, LED1=OFF
- **Condition 1**: Home switch malfunction (both contacts are ON)
- **Condition 2**: Test mode switch malfunction (both contacts are ON)
- Power Enable: Prior state maintained
- Power A & B relays: Prior state maintained

**Diagnostic Code 0x06:**
- Byte1 bits [7:3] = `00110`
- LED pattern: LED5=ON, LED4=ON, LED3=OFF, LED2=OFF, LED1=ON
- Power Enable: OFF
- Power A & B relays: OFF

See page 20 for complete diagnostic code table.

---

## At Home Status (Input8 / Byte1 / Bit0)

The "At Home" status bit is controlled by the home sensor and other safety conditions:

**At Home = 1 (TRUE) when:**
- Home sensor Contact A is CLOSED, AND
- Home sensor Contact B is OPEN, AND
- Contact transition timing was valid (<100msec), AND
- Test Mode is NOT active

**At Home = 0 (FALSE) when:**
- Machine is outside Safety Zone, OR
- Home sensor fault detected, OR
- Test Mode with Acknowledge is active (Note 2, page 19)

**Application:**
The "At Home" status is used to determine when work zone covers can be safely unlocked and opened. See cover control documentation for interaction with cover unlock logic.

---

## Integration with Safety System

The home sensor integrates with the CNC-SK-2310g2 safety system:

### Safety Bus (CN3)

The home sensor state affects the Safety Bus output:

**Safety Link OUT (CN3 pin 1) = HIGH when:**
- Covers are closed, OR
- Safe Zone (At Home) AND Spindle Stopped, OR
- Test Mode AND Acknowledge

### Cover Control

When "At Home" status is active:
- Covers can be unlocked (if other safety conditions met)
- Spindle must be stopped
- Power can be OFF or in Test Mode with Acknowledge

### Test Mode Interaction

**Note from page 19, Input Table:**
*"At Home is set 0 when Test Mode with Acknowledge is active"*

This means:
- Entering Test Mode with Acknowledge forces "At Home" = 0
- This prevents certain automated safety unlock sequences during test/maintenance
- Manual control of covers is still possible through Test Mode controls

---

## Alternative: Zero Speed Automation Mode

The CNC-SK-2310g2 also supports an alternative safety mode that does **not** use the physical home switch:

**Zero Speed Mode** (page 21):
- Uses servo drive "Zero Speed" signal instead of home switch
- All motors must be stopped for >2 seconds
- Spindle must be stopped
- Cover Lock output must be cleared

**Jumper Configuration for Zero Speed Mode:**
- J10-1 and J10-4: Configure for Zero Speed operation
- CN8 (Home connector): Not used in this mode
- See page 21-22 for complete Zero Speed mode documentation

---

## Common Applications

### 1. CNC Machine Tool Safety Zone

- Home position = Tool change position
- All axes retracted to safe clearances
- Spindle stopped and locked
- Work zone door can open when At Home

### 2. Automated Assembly System

- Home position = Load/unload station
- All motion axes parked
- Conveyors stopped
- Safety guards can open for part access

### 3. Pick-and-Place Robot

- Home position = Service position
- Robot arm fully retracted
- End effector in safe orientation
- Access panels unlock when At Home

---

## Design Recommendations

### Switch Selection

1. **Use automation-grade safety switches:**
   - Positive opening action (direct break)
   - Dual redundant contacts
   - Contact rating: 40Vdc, 0.5A minimum
   - Mechanical life: >1 million operations

2. **Recommended switch types:**
   - Mechanical limit switch with safety contacts
   - Magnetic safety switch (coded)
   - Inductive safety switch (dual channel)

### Installation

1. **Mount switch in protected location:**
   - Away from chips, coolant, debris
   - Protected from mechanical damage
   - Accessible for maintenance

2. **Use shielded cable for CN8 connection:**
   - Keep cable runs short (<10m typical)
   - Route away from high-current motor cables
   - Ground shield at controller end only

3. **Add optional H1 delay circuit if:**
   - Switch contact bounce is observed
   - Electrical noise causes false triggers
   - Cable length exceeds 5 meters

### Testing

1. **Power-up verification:**
   - Check LED indicators show correct home state
   - Verify diagnostic code matches expected condition
   - Test both home entry and exit sequences

2. **Safety validation:**
   - Verify covers unlock only when At Home
   - Test emergency stop with covers open
   - Confirm motor power cuts when covers open outside home

3. **Fault injection testing:**
   - Disconnect one contact - verify fault detection
   - Simulate slow transition - verify timing fault
   - Check diagnostic codes match fault conditions

---

## Critical Jumper Interactions

Some jumpers must be configured together to achieve desired behavior:

### Spindle Operation with Covers Open (Test Mode)
Requires **THREE** jumpers set correctly:
1. **J10-3 = SHORT**: Enables spindle in Test Mode
2. **J16 = 1-2 SHORT**: Allows Spindle ON output with covers open
3. **J20 = SHORT**: Allows cover opening in Test Mode

Missing any one of these three prevents spindle operation with covers open.

### Manual Cover Lock/Unlock Buttons
Requires **TWO** jumpers:
1. **J10-2 = SHORT**: Enables separate Lock/Unlock button mode
2. **J19 = SHORT**: Routes control to CN22 buttons

### Software Power Control
Requires **ONE** jumper:
1. **J21 = SHORT**: Enables Byte1/Bit7 power control

### Zero Speed Mode
Requires **TWO** jumpers for immediate unlock:
1. **J10-1 = SHORT**: Enables Zero Speed detection
2. **J10-4 = SHORT**: Immediate unlock on Zero Speed + Spindle Stop

For latched unlock (software controlled):
1. **J10-1 = OPEN**: Disables immediate mode
2. **J10-4 = SHORT**: Requires Output9 transition 1→0

---

## Related Documentation

- **SK-2310g2 Connector Pinouts**: Page 6-10
- **Jumper Settings Reference**: Page 5
- **Sample Application - Spindle Control Option 1**: Page 11
- **Sample Application - Spindle Control Option 2**: Page 12
- **Sample Application - Home Sensor Wiring**: Page 13
- **Sample Application - I/O Connector (Tool Changer)**: Page 13
- **Sample Application - Covers Wiring**: Page 14
- **Sample Application - Lamps**: Page 15
- **Sample Application - Emergency Stop**: Page 15
- **Sample Application - I/O Connector 2**: Page 16
- **Sample Application - Acknowledge and Cover Unlock**: Page 16
- **Sample Application - Test Mode and Power ON**: Page 17
- **Sample Application - Analog Inputs**: Page 17
- **Complete Wiring Diagram**: Page 18
- **Digital I/O Table**: Page 19
- **SK-2310g2 Diagnostic Codes**: Page 20
- **Zero Speed Automation Mode**: Page 21-22
- **Zero Speed Wiring Diagram**: Page 22
- **Distribution Boards**: Page 23-24

---

## Safety Warnings

1. **Do not bypass home sensor safety interlocks** - The dual-contact design is required for safe operation

2. **100msec timing is critical** - Contact transition must occur within this window to validate proper sensor operation

3. **Regular testing required** - Test home sensor function and fault detection periodically

4. **Qualified personnel only** - Installation and maintenance should be performed by trained technicians familiar with machine safety systems

---

## References

- CNC-SK-2310g2 Datasheet, Doc # 710231005 / Rev. D, 03/05/2020
- Page 13: Sample application – Home sensor wiring
- Page 7: CN8 connector pinout
- Page 5: Jumper configuration
- Page 19: Digital input assignments
- Page 20: Diagnostic codes

---

## Appendix: Complete Jumper Quick Reference

### All Jumpers at a Glance

| Jumper | Type | Function Summary | OPEN | SHORT/Position |
|--------|------|------------------|------|----------------|
| **J1** | 6-pin | LDCN Mode/Watchdog | Various LDCN configurations | See datasheet |
| **J2** | 4-pin | Power OFF Delay | Pin assignments | See page 4 |
| **J4** | 2-pos | CN4 Pin 6 Power | No +24V on CN4-6 | +24V on CN4-6 |
| **J5** | Multi | CN16 Pin Routing | Pins not connected | Route signals (see J5 section) |
| **J6** | 2-pos | ADC POT(+) | 100Ω protection | Direct +5V |
| **J7** | 2-pos | ADC POT(-) | 100Ω protection | Direct GND |
| **J10-1** | Pin 1 | Zero Speed Mode | Home sensor mode | Zero Speed enable |
| **J10-2** | Pin 2 | Cover Button Mode | Single unlock button | Separate Lock/Unlock (CN22) |
| **J10-3** | Pin 3 | Spindle in Test Mode | Spindle disabled | Spindle enabled in Test |
| **J10-4** | Pin 4 | Zero Speed Unlock | Normal operation | Zero Speed mode active |
| **J11** | Multi | Home/LED Config | Application specific | Application specific |
| **J12** | Multi | Safety Bus/LED-3 | Application specific | Application specific |
| **J14** | 3-pos | Cover 2 Lock Polarity | - | 1-2: Power when unlocked<br>2-3: Power when locked |
| **J15** | 3-pos | Cover 2 Lock Polarity | - | 1-2: Power when unlocked<br>2-3: Power when locked |
| **J16** | 3-pos | Spindle ON Control | - | 1-2: Test Mode enable<br>2-3: Covers-closed only |
| **J17** | 3-pos | Unlock Enable Route | - | 1-2: CN13-7 to GND<br>2-3: CN13-7 to Unlock Enable |
| **J18** | 2-pos | Reserved | **Required OPEN** | Do not use |
| **J19** | 2-pos | Cover Control Mode | Switch + Enable | Enable only (auto mode) |
| **J20** | 2-pos | Test Mode Unlock | Spindle must stop | Can unlock anytime |
| **J21** | 2-pos | Power ON Control | Button only | Button or software |

### Safety-Critical Jumper Warnings

⚠️ **J18 must remain OPEN** - Reserved for future use

⚠️ **J10-3, J16, J20** - These three jumpers control spindle access with covers open. Improper configuration can create dangerous conditions.

⚠️ **J14/J15** - Both must be set to same position. Mismatch can cause Cover 2 lock malfunction.

⚠️ **J10-1/J10-4** - Configure together for Zero Speed mode. Mismatch causes unpredictable home detection.

### Most Common Configurations

**Standard CNC Machine (Safe):**
- J10: All OPEN (1,2,3,4)
- J16: 2-3
- J17: 2-3
- J19: OPEN
- J20: OPEN
- J21: OPEN

**Test/Setup Mode:**
- J10-3: SHORT (spindle in test)
- J16: 1-2
- J20: SHORT
- J21: SHORT (optional)

**Zero Speed Automation:**
- J10-1: SHORT
- J10-4: SHORT
- J19: SHORT
- J21: SHORT
