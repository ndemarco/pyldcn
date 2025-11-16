# LS-231SE Diagnostic and I/O

**Author:** NickyDoes
**Source:** LS-231SE Multifunctional Servo Drive Datasheet (Doc # 712231004 / Rev. A, 05/05/2011)
**Date:** 2025-11-13

---

## General Introduction

This document describes the diagnostic and I/O functionality of the LS-231SE servo drive in LDCN mode.

---

## LDCN Mode (MODEbit[C,B,A] = 000)

The LS-231SE operates in LDCN mode when the mode selection bits are set to `000`. This is the standard configuration for LDCN protocol communication.

### Mode Selection

- **MODEbit[C,B,A] = 000**: LDCN Dual loop mode (standard LDCN mode)
- Other mode bit combinations (001-111) select different operational modes (Analog, PWM, Step & Direction, etc.)

---

## Diagnostic Bit Definitions

### Status Byte Bits (Primary Status)

| Bit | Name | Description |
|-----|------|-------------|
| **0** | Move_done | Clear during trapezoidal move or velocity acceleration |
| **3** | Power/DE | Drive Enable status (PIC_AE bit). High when power driver is enabled |
| **4** | Pos_error | Position error exceeded limit or servo disabled |
| **5** | Home Source | Home switch input or diagnostic indicator |
| **6** | Limit2 | Forward limit switch or diagnostic indicator |

### Auxiliary Status Byte Bits

| Bit | Name | Description |
|-----|------|-------------|
| **0** | Index | Complement of encoder index input or diagnostic indicator |
| **2** | Servo | Position servo loop enabled |

### Stop Command Bit

| Bit | Name | Description |
|-----|------|-------------|
| **0** | Stop Cmd | Stop motor command bit from control byte |

### Drive Enable (DE)

- **Pic_ae≡DE**: Power driver enable signal
- **INbit12**: Drive enable status bit
- Returns `1` when PIC_AE bit is SET (power driver enabled)
- Returns `0` when PIC_AE bit is CLEARED (power driver disabled)

---

## LDCN Mode State and Diagnostics Table

The following tables show the relationship between status bits and drive conditions in LDCN mode (MODEbit[C,B,A] = 000).

**Legend**:

- `1` = Bit set (HIGH)
- `0` = Bit clear (LOW)
- `X` = Don't care (either state)

### Part 1: Status and Control Bits

| Condition | Description | Limit2<br>(Bit 6) | Home<br>(Bit 5) | Pos_err<br>(Bit 4) | Power<br>(Bit 3) | Move_done<br>(Bit 0) | Servo<br>(Aux Bit 2) | Index<br>(Aux Bit 0) | Stop Cmd | Pic_ae<br>(DE) |
|-----------|-------------|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| **No Motor Power after LDCN Init** | Initial state after LDCN initialization, motor power not enabled | 0 | 1 | 1 | 0 | 1 | 0 | 1 | 0 | 0 |
| **AxisOFF** | Axis disabled, amplifier off | 1 | 1 | 1 | 1 | 0 | 0 | X | 0 | 0 |
| **ServoON** | Normal operation, servo loop active | 0 | 0 | 0 | 1 | X | 1 | X | 1 | 1 |
| **ServoOFF** | Servo loop disabled, motor holding position | 0 | 0 | 0 | 1 | 1 | 0 | X | 1 | 1 |
| **ErrHALL** | Hall sensor error detected on brushless motor | X | X | 1 | 0 | 1 | 0 | 1 | 0 | 0 |
| **ErrEEPROM** | EEPROM checksum error on power-up | 1 | 1 | 1 | 0 | 0 | 0 | 0 | 0 | 0 |
| **No Motor Power** | Motor power supply missing or insufficient | 1 | 0 | 1 | 0 | 1 | 0 | 0 | 0 | 0 |
| **Overheat** | Drive temperature exceeded safe limit | 0 | 0 | 0 | 0 | 1 | 0 | 1 | 0 | 0 |
| **Disabled** | Drive disabled by command or external signal | 1 | 0 | 1 | 0 | 1 | 0 | 1 | 0 | 0 |
| **Master EncoderERROR** | Master encoder signal lost or invalid | 0 | 0 | 0 | 0 | 1 | 0 | 1 | 1 | 1 |
| **Brake or Output Short** | Brake output or other output shorted to power | 1 | 1 | 1 | 0 | 1 | 0 | 0 | 0 | 0 |
| **Stopped** | Motor stopped by stop command | 0 | 0 | 0 | 1 | 1 | 0 | X | 1 | 1 |
| **MotorShort** | Motor windings shorted together or to ground | 1 | 0 | 1 | 0 | 1 | 0 | 0 | 0 | 0 |
| **Motor PowerDROP** | Motor power voltage dropped below threshold | 0 | 0 | 1 | 0 | X | 0 | 1 | 1 | 1 |
| **OverLOAD** | Current limit exceeded, excessive mechanical load | 1 | 0 | 1 | 0 | X | 0 | 0 | 1 | 1 |
| **EncoderERR (Reset required)** | Load encoder error, requires hard reset | X | X | 1 | 1 | 1 | 0 | X | 0 | 0 |
| **PositionERROR** | Position following error exceeded limit | 0 | 1 | 1 | 0 | 1 | 0 | 1 | 0 | 0 |
| **Limit2** | Forward limit switch triggered | 1 | X | 1 | 1 | 0 | 0 | X | 0 | 1 |
| **Home Source** | Home switch triggered during homing | 1 | 1 | 1 | 0 | 0 | 0 | 1 | 0 | 0 |
| **Encoder** | Encoder-related status condition | X | 1 | 1 | 1 | 0 | 1 | 0 | 0 | 1 |

### Part 2: LED Indicators and Physical Outputs

| Condition | Orange LED | Green LED | Red LED | FAULT Relay | BRAKE Output |
|-----------|:----------:|:---------:|:-------:|:-----------:|:------------:|
| **No Motor Power after LDCN Init** | ON | ON | OFF | OFF | Released |
| **AxisOFF** | OFF | ON | ON | OFF | Closed (CN8pin9 High) |
| **ServoON** | ON | ON | ON | OFF | Closed (CN8pin9 High) |
| **ServoOFF** | OFF | Alternate | ON | OFF | Released |
| **ErrHALL** | Blink | Blink | Fast Blink | Closed | Closed |
| **ErrEEPROM** | Blink | Blink | ON | Closed | Closed |
| **No Motor Power** | Alternate | OFF | Blink | Closed | Closed |
| **Overheat** | Alternate | Blink | Blink | Closed | Closed |
| **Disabled** | ON | Blink | Blink | Closed | Closed |
| **Master EncoderERROR** | OFF | Blink | Blink | Closed | Closed |
| **Brake or Output Short** | ON | ON | Blink | Closed | Closed |
| **Stopped** | Blink | Alternate | Blink | Closed | Closed |
| **MotorShort** | Blink | Blink | ON | Closed | Closed |
| **Motor PowerDROP** | ON | OFF | Blink | Closed | Closed |
| **OverLOAD** | Blink | ON | Blink | Closed | Closed |
| **EncoderERR (Reset required)** | Blink | OFF | OFF | Closed | Closed |
| **PositionERROR** | ON | Blink | ON | Closed | Closed |
| **Limit2** | Blink | Blink | ON | Closed | Closed |
| **Home Source** | OFF | ON | Blink | Closed | Released |
| **Encoder** | OFF | Blink | Blink | Closed | Released |

---

## LED Indicator Descriptions

The LS-231SE has three LED indicators for drive status:

### Orange LED

- **ON**: Normal power, various operational states
- **OFF**: Servo disabled, axis off, specific error conditions
- **Blink**: Error conditions (Hall, EEPROM, motor short, position error)
- **Alternate**: Motor power issues, servo off mode

### Green LED

- **ON**: Normal operation, servo on, various operational states
- **OFF**: Motor power drop, encoder error, specific fault conditions
- **Blink**: Error conditions, disabled state, master encoder error
- **Alternate**: Servo off mode, stopped state

### Red (FAULT) LED

- **OFF**: Normal operation, no faults, encoder error
- **ON**: Servo on, axis off, various fault conditions
- **Blink**: Motor power issues, brake short, overload, stopped
- **Fast Blink**: Hall error

---

## Brake Output States

The brake output (OUTbit0) controls an external motor brake:

| State | Description | Physical State |
|-------|-------------|----------------|
| **Released** | Brake not applied | Motor free to rotate |
| **Closed** | Brake applied | Motor mechanically locked |
| **CN8pin9 High** | Brake control signal high | Brake applied via CN8 pin 9 |
| **CN8pin9 Low** | Brake control signal low | Brake released via CN8 pin 9 |

**Brake Control**:

- Automatically applied (Closed) during most error conditions
- Released during normal operation (AxisOFF, ServoOFF with no errors)
- Can be manually controlled via I/O Control command (0x08)

---

## FAULT Relay Output

The FAULT relay (CN8pin9) provides an external fault indication:

| State | Condition |
|-------|-----------|
| **OFF** | Normal operation |
| **Closed** | Fault condition present |

The FAULT relay closes during most error conditions and is used to signal external safety systems or indicators.

---

## Diagnostic Procedures

### Normal Operation Indicators

- **Orange**: ON
- **Green**: ON or Alternate (ServoOFF)
- **Red**: ON (ServoON) or OFF (No Motor Power after Init)
- **Brake**: Released or Closed (CN8pin9 High)
- **Status Bit 3 (Power)**: 1 (for ServoON) or 0 (for AxisOFF)

### Error Detection

1. **Check LED patterns** - Blinking LEDs indicate specific errors
2. **Read status byte** - Check bits 4 (Pos_error), 5 (Home Source), 6 (Limit2)
3. **Read auxiliary status** - Check bit 2 (Servo)
4. **Check DE signal** - Verify power driver enable state

### Common Error Conditions

#### Hall Sensor Error (ErrHALL)

- **LEDs**: Orange Blink, Green Blink, Red Fast Blink
- **Cause**: Invalid hall sensor reading on brushless motor
- **Action**: Check hall sensor connections, verify motor type

#### Encoder Error (EncoderERR)

- **LEDs**: Orange Blink, Green OFF, Red OFF
- **Cause**: Encoder signal lost or invalid
- **Action**: Check encoder connections, verify encoder power, **requires reset**

#### Position Error (PositionERROR)

- **LEDs**: Orange ON, Green Blink, Red ON
- **Cause**: Position following error exceeded limit
- **Action**: Check mechanical load, verify gains, clear sticky bit

#### Motor Power Drop

- **LEDs**: Orange ON, Green OFF, Red Blink
- **Cause**: Motor power supply voltage drop
- **Action**: Check motor power supply, verify power connections

#### Overload

- **LEDs**: Orange Blink, Green ON, Red Blink
- **Cause**: Current limiting active, excessive load
- **Action**: Reduce load, check motor specifications, verify current limit setting

---

## Implementation Notes

### Reading Diagnostic State

To determine the current drive condition:

1. **Read Status Byte** (via NOP or Read Status command)
   - Extract bits 0, 3, 4, 5, 6
2. **Read Auxiliary Status Byte** (via Define Status bit 3)
   - Extract bits 0, 2
3. **Match bit pattern** to diagnostic table
4. **Identify condition** and take appropriate action

### Clearing Error Conditions

Some error conditions require explicit clearing:

1. **Sticky Bits** (current_limit, pos_error): Use Clear Bits command (0x0B)
2. **Encoder Error**: Requires hard reset (power cycle or Hard Reset command 0x0F)
3. **EEPROM Error**: Requires hard reset and possible EEPROM reinitialization

### Safety Considerations

- **FAULT relay** should be wired to safety circuit for emergency stop
- **Brake output** should be wired to motor brake for holding during faults
- Monitor **Status Bit 3 (Power)** to verify amplifier enable state
- Check **Red LED** for fault indication during operation

---

## References

- Logosol LS-231SE Multifunctional Servo Drive Datasheet (Doc # 712231004 / Rev. A, 05/05/2011)
  - Section: "LS-231SE Diagnostic and I/O"
  - Section: "LDCN mode state and diagnostics"
- [servo_status_reporting.md](servo_status_reporting.md) - Status byte and auxiliary status byte definitions
- [servo_commands.md](servo_commands.md) - Complete LS-231SE command reference
