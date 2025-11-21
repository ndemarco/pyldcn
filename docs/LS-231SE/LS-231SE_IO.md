# LS-231SE Digital I/O Mapping

**Source:** Logosol LS-231SE Advanced Multifunctional Servo Drive Datasheet (Doc # 712231004 / Rev. A, 05/05/2011), p. 32  

## I/O

### Digital Inputs

| Bit | Name        | Description |
|-----|-------------|-------------|
| 0   | StatusBit5  | Diagnostic bit routed from Limit1 or HomeIN depending on HomeSEL settings (see [Home Source Selection](#home-source-selection)). |
| 1   | StatusBit6  | Diagnostic bit routed from Limit2, Input10, or Input11 depending on HomeSEL settings. |
| 2   | StatusBit3  | Power_on diagnostic bit (matches status byte bit 3). |
| 3   | HomeIN      | Home input (CN9.6 / CN5.14). High = 1 |
| 4   | Limit1      | Reverse limit input (CN9.4 / CN5.7). High = l, normally closed. |
| 5   | Limit2      | Forward limit input (CN9.2 / CN5.15). High = 1, normally closed |
| 6   | BridgeSTA   | 1 when limit switches are bridged. |
| 7   | AmpEnable   | Drive amplifier enable input (CN8 pin 12). High = enable (mode-dependent). Not used for LDCN single mode |
| 8   | SafetyLINK  | Hardware enable/stop input (Safety bus CN3.4 & CN4.4) low = hardware stop. |
| 9   | Input9      | Digital input (CN3.6 / CN4.6). High = 1. |
| 10  | Input10     | Digital input (CN2.2). High = 1. |
| 11  | Input11     | LDCN mode: Digital input (CN9.7); Amplifier modes: hardware bridge input. High = Limit bridge active. |
| 12  | DE          | LDCN mode: PIC_AE drive enable; Amplifier modes: amplifier enable |
| 13  | Reserved    | Not used |
| 14  | Dir         | LDCN modes: master encoder A input (CN8.7); Other modes per documentation|
| 15  | FAULT       | SafetyLINK fault (CN3.3) 0 = closed, 1 = open. |

---

### Digital Outputs

| Bit | Name        | Description |
|-----|-------------|-------------|
| 0   | BrakeMODE   | 0 = Output1 follows servo diagnostic state. 1 = Output1 follows `Output1` |
| 1   | Output0     | CN1.4; Brake or digital output; Depends on BrakeMODE |
| 2   | Output2     | CN2.3; 0 = off, 1 = high. |
| 3   | Reserved    | (keep cleared) |
| 4   | HomeSEL1    | See [Home Source Selection](#home-source-selection) |
| 5   | Bridge      | Part of the limit relay logic. See [Limit Relay Control](#limit-relay-control) |
| 6   | UserREL     | User relay output. See [Limit Relay Control](#limit-relay-control) |
| 7   | SmartSTOP   | 1 enables the SmartSTOP timer + logic. |
| 8   | HomeSEL2    | See [Home Source Selection](#home-source-selection) |
| 9–11 | Reserved   | (keep cleared) |
| 12  | MODEbitA    | Mode selection bit (see mode tables in hardware manual) |
| 13  | MODEbitB    | Mode selection bit |
| 14  | MODEbitC    | Mode selection bit |
| 15  | MODEbitD    | Mode selection bit |
| 16–19 | Reserved  | (keep cleared) |

---

## I/O Modifiers

### HomeSEL
**Home Source Selection** (INbit0, INbit1)

HomeSEL 1 & 2 determine which sources drive status bits per the table below.

| HomeSEL2<br>(OUTbit8) | HomeSEL1<br>(OUTbit4)> | Status Bit5 | Status Bit6 | Motor enc latch | Master enc latch |
| -------- | ---------| ----------- | ----------- | --------------- | ---------------- |
| 0 | 0 | Limit1 | Limit2 | Motor Z | Master Z |
| 0 | 1 | HomeIN | Input10 | Motor Z | Master Z |
| 1 | 0 | HomeIN | Input10 | Input10* | Input10* |  
| 1 | 1 | HomeIN | Input11 | Input11* | Input11* |  

**Notes**:
- \* Any edge is considered a trigger
- Limit1 input is CN9.4 or CN5.7
- Limit2 input is CN9.2 or CN5.15

## Limit Relay Control

`Bridge` (OUTbit5), `UserREL` (OUTbit6), and Input11 (INbit11) jointly determine the state of the limit relay and user relay. BridgeSTA (INbit6) mirrors whether the limit relay is forced closed.

| Bridge<br>(bit 5) | UserREL<br>(bit 6) | Input11 | LDCN mode behavior | 
|----------------|-----------------|---------|-----------------------------------|
| 0              | 0               | 0       | Limit relay depends on Limit1/Limit2 | 
| 0              | 1               | 0       | Limit relay forced closed (BridgeSTA=1) | 
| 0              | 0               | 1       | Limit relay depends on Limit1/Limit2 | 
| 1              | 0               | X       | User relay open (general purpose) |
| 1              | 1               | X       | User relay closed |

> **Note:** Input11 doubles as the bridge command in amplifier modes. See the diagnostics tables for how Limit1/Limit2 states drive the relay when not forced.

---

## Diagnostic and SmartSTOP Notes

- `SmartSTOP` (OUTbit7) enables the hardware timer used for the Smart STOP feature. SmartSTOP is undocumented. It appears to use servo control to hold the motor until the brake applies.

For complete diagnostic behavior (LED patterns, brake states, safety bus wiring), refer to [servo_diagnostics.md](servo_diagnostics.md).
