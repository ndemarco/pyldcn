# LS-231SE Digital I/O Mapping

**Source:** Logosol LS-231SE Advanced Multifunctional Servo Drive Datasheet (Doc # 712231004 / Rev. A, 05/05/2011), p. 32  
**Purpose:** Quick reference for digital inputs/outputs exposed by the LS‑231SE in LDCN mode.  
**Connectors:** See CN5/CN8/CN9 pinouts in the hardware manual for wiring details.

---

## Digital Inputs

| Bit | Name        | Description |
|-----|-------------|-------------|
| 0   | StatusBit5  | Diagnostic bit routed from Limit1 or HomeIN depending on HomeSEL settings (see [Home Source Selection](#home-source-selection)). |
| 1   | StatusBit6  | Diagnostic bit routed from Limit2, Input10, or Input11 depending on HomeSEL settings. |
| 2   | StatusBit3  | Power_on diagnostic bit (matches status byte bit 3). |
| 3   | HomeIN      | Home input (CN9 pin 6 / CN5 pin 14). High = 1. |
| 4   | Limit1      | Reverse limit input (CN9 pin 4 / CN5 pin 7). High = 1. |
| 5   | Limit2      | Forward limit input (CN9 pin 2 / CN5 pin 15). High = 1. |
| 6   | BridgeSTA   | 1 when limit switches are bridged. |
| 7   | AEN         | Amplifier enable input (CN8 pin 12). High = enable (mode-dependent). |
| 8   | Enable / Stop | Hardware enable/stop input (CN3 pin 4 & CN4 pin 4). High = enable, Low = hardware stop. |
| 9   | Input9      | General-purpose input (CN3 pin 6 / CN4 pin 6). High = 1. |
| 10  | Input10     | General-purpose input (CN2 pin 2). High = 1. |
| 11  | Input11     | GP input in LDCN mode (CN9 pin 7). In amplifier modes it becomes the hardware bridge input. High = Limit bridge active. |
| 12  | DE          | Drive enable status bit (PIC_AE in LDCN mode, amplifier enable otherwise). |
| 13  | Reserved    | Not used. |
| 14  | Dir         | Multifunction input (CN8 pin 7). High = 1 (mode-dependent). |
| 15  | FAULT       | 0 = Fault relay contact closed, 1 = open. |

---

## Digital Outputs

| Bit | Name        | Description |
|-----|-------------|-------------|
| 0   | BrakeMODE   | 0 = Brake/Output1 follows servo diagnostic state. 1 = Brake/Output1 follows `Output1`. |
| 1   | Output1     | When `BrakeMODE=1`: 0 turns Brake/Output1 (CN1 pin 4) off, 1 drives it high. |
| 2   | Output2     | CN2 pin 3. 0 = off, 1 = high. |
| 3   | Reserved    | Keep cleared. |
| 4   | HomeSEL1    | Part of the home/limit source selector. See [Home Source Selection](#home-source-selection). |
| 5   | Bridge      | Part of the limit relay logic. See [Limit Relay Control](#limit-relay-control). |
| 6   | UserREL     | User relay output. See [Limit Relay Control](#limit-relay-control). |
| 7   | SmartSTOP   | 1 enables the SmartSTOP timer + logic. |
| 8   | HomeSEL2    | Second home/limit selector bit. See [Home Source Selection](#home-source-selection). |
| 9–11 | Reserved   | Keep cleared. |
| 12  | MODEbitA    | Mode selection bit (see mode tables in hardware manual). |
| 13  | MODEbitB    | Mode selection bit. |
| 14  | MODEbitC    | Mode selection bit. |
| 15  | MODEbitD    | Mode selection bit. |
| 16–19 | Reserved  | Keep cleared. |

---

## Home Source Selection

`HomeSEL1` (OUTbit4) and `HomeSEL2` (OUTbit8) determine how the LS‑231SE routes limit/home signals into StatusBit5/StatusBit6 and which latch source is used for the motor/master encoders.

| HomeSEL2 (bit 8) | HomeSEL1 (bit 4) | StatusBit5 source | StatusBit6 source | Motor encoder latch | Master encoder latch |
|------------------|------------------|-------------------|-------------------|----------------------|----------------------|
| 0                | 0                | Limit1            | Limit2            | Motor encoder phase Z | Master encoder phase Z |
| 0                | 1                | HomeIN            | Input10           | Motor encoder phase Z | Master encoder phase Z |
| 1                | 0                | HomeIN            | Input10           | Input10 (change latch) | Input10 (change latch) |
| 1                | 1                | HomeIN            | Input11           | Input11 (change latch) | Input11 (change latch) |

---

## Limit Relay Control

`Bridge` (OUTbit5), `UserREL` (OUTbit6), and Input11 (INbit11) jointly determine the state of the limit relay and user relay. BridgeSTA (INbit6) mirrors whether the limit relay is forced closed.

| Bridge (bit 5) | UserREL (bit 6) | Input11 | LDCN mode behavior                | Amplifier mode behavior          |
|----------------|-----------------|---------|-----------------------------------|----------------------------------|
| 0              | 0               | 0       | Limit relay depends on Limit1/Limit2 | Same as LDCN mode               |
| 0              | 1               | 0       | Limit relay forced closed (BridgeSTA=1) | Same as LDCN mode            |
| 0              | 0               | 1       | Limit relay depends on Limit1/Limit2 | Closed (BridgeSTA=1)            |
| 1              | 0               | X       | User relay open (general purpose)  | User relay open                  |
| 1              | 1               | X       | User relay closed                  | User relay closed                |

> **Note:** Input11 doubles as the bridge command in amplifier modes. See the diagnostics tables for how Limit1/Limit2 states drive the relay when not forced.

---

## Fault and SmartSTOP Notes

- `SmartSTOP` (OUTbit7) enables the hardware timer used for the Smart STOP feature. Wiring details are in the diagnostics section of the hardware manual.
- `FAULT` (INbit15) is the relay feedback: 0 = contact closed (fault present), 1 = contact open.

For complete diagnostic behavior (LED patterns, brake states, safety bus wiring), refer to [servo_diagnostics.md](servo_diagnostics.md).
