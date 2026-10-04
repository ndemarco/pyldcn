# LDCN lineage from JR Kerr NMC, and the status gaps it explains

Logosol's Distributed Control Network (LDCN) is a rebadged derivative of JR Kerr
Automation Engineering's Networked Motion Control (NMC) protocol. The LS-773
Network I/O Node and, by inference, the CNC-SK-2310g2 Supervisor I/O Controller
implement the JR Kerr PIC-I/O command set. This document records the evidence,
the verbatim JR Kerr definitions that Logosol's datasheets abbreviate, and the
hypotheses that fall out of the comparison. Pace work package #1725 (JR Kerr
PIC-IO lineage doc) tracks the work.

## 1. Lineage evidence

Neither vendor publishes a licensing statement. The structural evidence is
conclusive on its own:

- **Identical library export surface.** JR Kerr's `NMCLIB04.dll` (source in
  `nmclib04.zip`, archived at
  <http://web.archive.org/web/2016id_/http://www.jrkerr.com/nmclib04.zip>;
  `jrkerr.com` itself now redirects to an unrelated domain) and Logosol's
  `Ldcnlib.dll` SDK (<https://logosolinc.com/wp-content/downloads/ldcnlib.exe>,
  a zip containing `Ldcnlib.dll`, `Ldcnlib.lib`, `LdcnLib.PDF` Doc# 712173002
  Rev 0.5, 10/25/2000) export the same functions: `Sio*`, `FixSioError`,
  `IoGetStat`, `IoInBitVal`, `IoInBitSVal`, `IoOutBitVal`, `IoGetADCVal`,
  `IoGetPWMVal`, `IoGetTimerVal`, `IoGetTimerSVal`, `IoSetOutBit`,
  `IoClrOutBit`, `IoSetPWMVal`, `IoSetTimerMode`, `IoSetSynchOutput`, `Servo*`,
  `Step*`. Logosol renamed only the network layer: `NmcInit` → `LdcnInit`,
  `NmcSendCmd` → `LdcnSendCmd`, `NmcDefineStatus` → `LdcnDefineStatus`, and so on.
- **Identical initialisation sequence.** `LdcnLib.PDF` describes `LdcnInit` as:
  Hard Reset (0x0F) to group 0xFF; `SetAddress` (command byte 0x21) to address
  0x00 with `(addr, 0xFF)`; `ReadStatus` (0x13) with parameter 0x20 to fetch
  device ID and version; increment and repeat. That is JR Kerr's `NmcInit` in
  `nmccom.cpp` line for line.
- **Identical module type codes.** `nmccom.h`: `SERVOMODTYPE 0`, `ADCMODTYPE 1`,
  `IOMODTYPE 2`, `STEPMODTYPE 3`. `LdcnLib.PDF`: "SERVOMODTYPE (0) or IOMODTYPE
  (2) or STEPMODTYPE (3)". The LS-773 datasheet states "LS-773 device ID = 2".
- **JR Kerr comments survive in Logosol's own source.** Logosol's Dynamic C
  library `LDCN.LIB version 1.52N` (mirrored at
  <https://github.com/cversek/linux-ldcn/blob/master/doc/logosol/LOGOSOL_RABBIT_DYNAMIC_C_Ldcn.Lib>)
  contains `// max num points to store in the PIC-SERVO buffer` (line 5725)
  and `// punt when PIC-SERVO buffer is full` (line 6513).
- **Identical wire format.** Header 0xAA; command byte = `(n << 4) | cmd`;
  checksum = 8-bit sum of address, command and data bytes (header excluded);
  status packet = status byte, optional data, checksum (8-bit sum of the
  preceding bytes). The LS-773 datasheet (Doc# 712773001 Rev B,
  <https://logosolinc.com/wp-content/uploads/2020/08/LS-773-Network-IO-Node.pdf>)
  is a near-verbatim rewrite of the PIC-I/O datasheet, down to the same
  "if bits 0 and 3 are set" worked example in the Define Status description.

JR Kerr sources: PIC-I/O datasheet `piodata.pdf` and the older board manual
`picio.pdf`, both archived under
<http://web.archive.org/web/2016id_/http://www.jrkerr.com/>. Logosol's own
pages: <https://logosolinc.com/software/>, <https://logosolinc.com/pdf-docs/>.

## 2. JR Kerr PIC-I/O definitions

### 2.1 Command set

Identical to the LS-773 list with one addition: **0x0 Set Direction (2 data
bytes)**, which the LS-773 marks "Not used" because its outputs are fixed
open-collector drivers.

| Cmd | Name | Data bytes | Description (JR Kerr) |
|-----|------|-----------:|-----------------------|
| 0x0 | set direction | 2 | Sets I/O bit directions as inputs or outputs |
| 0x1 | set address | 2 | Sets the individual and group addresses |
| 0x2 | define status | 1 | Defines which data should be sent in every status packet |
| 0x3 | read status | 1 | Causes particular status data to be returned just once |
| 0x4 | set pwm | 2 | Immediately set the PWM output values |
| 0x5 | synch output | 0 | Outputs previously stored output bit and PWM output values |
| 0x6 | set outputs | 2 | Immediately set the states of the output bits |
| 0x7 | set synch output | 4 | Store output bit values and PWM values to be synchronously set with "synch output" |
| 0x8 | set timer mode | 1 | Sets mode of the counter timer and its prescalar |
| 0x9 | --- | 0 | Not used |
| 0xA | set baud rate | 1 | Sets the baud rate (group command only) |
| 0xB | --- | 0 | Not used |
| 0xC | synch input | 0 | Synchronously stores the input bit values and the counter/timer value |
| 0xD | --- | 0 | Not used |
| 0xE | nop | 0 | Simply causes the defined status data to be returned |
| 0xF | hard reset | 0 | Resets the controller to its powerup state |

Set Direction data (PIC-I/O only): byte 1 bits 0-7 = direction of I/O bits 1-8
(0 = output, 1 = input); byte 2 bits 0-3 = I/O bits 9-12, bits 4-7 don't care.
All bits are inputs at power-up.

Set Address: byte 1 individual address 0-0xFF (initial 0x00); byte 2 group
address (initial 0xFF). Group addresses are always interpreted as 0x80-0xFF;
clearing bit 7 in byte 2 makes the module a group leader. The first Set Address
after reset also lowers the module's ADDR_OUT pin, enabling the next module in
the chain.

### 2.2 Define Status / Read Status items (verbatim, `piodata.pdf`)

```
1. Status items: (default: 0x00)
   Bit 0: send input bit values (2 bytes)
          (the first byte will contain the input values for I/O bits 1-8, the second byte
          the input values for I/O bits 9 - 12 in the lower nibble)
      1: send A/D 1 value (1 byte)
      2: send A/D 2 value (1 byte)
      3: send A/D 3 value (1 byte)
      4: send counter/timer value (4 bytes, least significant first)
      5: send device ID, version number (2 bytes)
          (PIC-IO device ID = 0)
      6: send input bit values captured with the "synch input" command (2 bytes)
      7: send counter/timer value captured with the "synch input" command (4 bytes)
```

`picio.h` constants:

```c
#define SEND_INPUTS   0x01  //2 bytes data
#define SEND_AD1      0x02  //1 byte
#define SEND_AD2      0x04  //1 byte
#define SEND_AD3      0x08  //1 byte
#define SEND_TIMER    0x10  //4 bytes
#define SEND_ID       0x20  //2 bytes
#define SEND_SYNC_IN  0x40  //2 bytes
#define SEND_SYNC_TMR 0x80  //4 bytes
```

Returned bytes always follow bit order. `IoGetStat` in `picio.cpp` computes the
expected length as 2 + the sum of the sizes above and parses in that order.

### 2.3 Status byte (verbatim, `piodata.pdf` section 3.4)

> The first byte of every status packet is the status byte, whose bits contain
> basic operating information about the module. The PIC-I/O has only one of the
> eight bits defined: if Bit 1 of the status byte is set, the PIC-I/O detected a
> checksum error in the most recently sent command packet. Bits 0, 2, 3, 4, 5, 6
> and 7 are all undefined and can be ignored.

`nmccom.h`: `#define CKSUM_ERROR 0x02 //Checksum error bit in status byte`.

JR Kerr defines nothing beyond what Logosol does. The "undefined" bits in the
LS-773 datasheet are inherited, not redacted. For comparison, the servo module
status byte in `LDCN.LIB` is fully defined: `MOVE_DONE 0x01, CKSUM_ERROR 0x02,
OVERCURRENT 0x04, POWER_ON 0x08, POS_ERR 0x10, LIMIT1 0x20, LIMIT2 0x40,
HOME_IN_PROG 0x80`.

### 2.4 Output, timer and baud details

- Set Outputs (0x6, command byte 0x26): byte 1 = output bits 1-8, byte 2 = bits
  9-12 (bits 4-7 ignored). Bits configured as inputs ignore their data bit.
- Set Synch Output (0x7, 0x47): out byte 1, out byte 2, PWM1, PWM2.
- Set PWM (0x4, 0x24): PWM1 0-255, PWM2 0-255; 0 = driver off, 255 = 100 % duty.
- Set Timer Mode (0x8, 0x18): bit 0 enable; bit 1 = 1 counter, 0 timer; bits
  5:4 prescaler 00/01/10/11 = 1:1 / 2:1 / 4:1 / 8:1; bits 2, 3, 6, 7 don't care.
  Counter counts rising edges on I/O bit 10; timer counts the 5 MHz PIC clock.
- Baud divisors (`nmccom.h`, written to the PIC SPBRG register with BRGH = 1):
  `PB19200 64`, `PB57600 21`, `PB115200 10`, `PB230400 5`.

## 3. LS-773 differences from PIC-I/O

Source: Doc# 712773001 Rev B, 10/16/2013.

- Command 0x0 "Not used". Addresses narrowed: individual 0x00-0x7F, group
  0x80-0xFF.
- Define Status bit 5 returns "device ID = 2, version number = 50".
- Analog channels named ANALOG IN 0-2 instead of A/D 1-3.
- Output layout: Byte 0 bits 0-6 = OUTPUT 0-6 (bit 7 X); Byte 1 all X. Set Synch
  Output byte 2: "Set this byte to 0x00". OUTPUT 0 is the POWER output, OUTPUT 1
  and 2 double as PWM.
- Input layout: Byte 0 bits 0-7 = INPUT 0-7. Byte 1 bit 0 = INPUT 8, bit 1 =
  INPUT 9/COUNT, bits 2-6 X, **bit 7 = OUT_SH**: "OUT_SH = 1 when one or more
  outputs are shorted to POWER(+). OUT_SH is 0 during the normal operation."
- Counter counts high-to-low transitions of DIGITAL IN 9/COUNT (PIC-I/O: rising
  edge on I/O bit 10).
- Status byte text identical: bit 1 checksum error, "Bits 0, 2, 3, 4, 5, 6 and 7
  are undefined and can be ignored."

## 4. Ldcnlib.dll exports

### 4.1 2002 build (ldcnlib.exe, Ldcnlib.dll dated 2002-02-06)

Network layer: `LdcnInit`, `LdcnFullInit`, `LdcnSendCmd`, `LdcnShutdown`,
`LdcnSetGroupAddr`, `LdcnDefineStatus`, `LdcnReadStatus`, `LdcnChangeBaud`,
`LdcnSynchInput`, `LdcnNoOp`, `LdcnHardReset`, `LdcnResetDevice`,
`LdcnRestoreNetwork`, `LdcnGetStat`, `LdcnGetStatItems`, `LdcnGetModType`,
`LdcnGetModVer`, `LdcnGetGroupAddr`, `LdcnGroupLeader`, `FixSioError`.

Serial layer: `SioOpen`, `SioClose`, `SioChangeBaud`, `SioPutChars`,
`SioGetChars`, `SioTest`, `SioClrInbuf`.

I/O node: `IoGetStat`, `IoInBitVal`, `IoInBitSVal`, `IoOutBitVal`,
`IoGetOutputs`, `IoSetOutputs`, `IoSetOutBit`, `IoClrOutBit`, `IoGetBitDir`,
`IoBitDirIn`, `IoBitDirOut`, `IoGetADCVal`, `IoGetPWMVal`, `IoSetPWMVal`,
`IoGetTimerVal`, `IoGetTimerSVal`, `IoGetTimerMode`, `IoSetTimerMode`,
`IoSetSynchOutput`.

`IoGetOutputs` / `IoSetOutputs` are Logosol additions over JR Kerr's `picio.h`,
but both operate on the same host-side `outbits` field (see section 5).
`IoBitDirIn` / `IoBitDirOut` / `IoGetBitDir` are retained from JR Kerr even
though the LS-773 ignores command 0x0.

Servo and stepper exports (`Servo*`, `Step*`) are the JR Kerr set plus Logosol
path-mode additions (`ServoAddPathPoints`, `ServoSetFastPath`,
`ServoGetSDMode`, `ServoEEPROMCtrl`, `ServoGetAD10`, `ServoLoadTraj10`).

### 4.2 2020 build (LdcnUtility.zip, Ldcnlib.dll dated 2020-07-06)

New exports relative to 2002, all at the network layer:

```
LdcnCheckFixedAddr
LdcnDefineStatus2
LdcnGetFastSerial
LdcnGetModSubVer
LdcnGetStatItems2
LdcnHardResetBdr
LdcnOpen
LdcnReadStatus2
LdcnRestoreInitNetwork
LdcnRestoreInitNetworkAmp
LdcnSendCmdBuff
LdcnSetFastSerial
LdcnSetPacketCommMode
```

No new `Io*` exports. No exports were removed. `LdcnDefineStatus2` /
`LdcnReadStatus2` / `LdcnGetStatItems2` imply a second status-items byte, which
is how the LS-231 family's extended status (`SEND_INPORTS` and friends) is
requested; whether the SK-2310g2 honours it is untested (section 8).

## 5. Output readback is host-cached, not read from the bus

The I/O command set has no "read outputs" command and the status item list has
no output-state item. Every "get output" API returns a value the host wrote:

- JR Kerr `picio.cpp` line 197, `IoOutBitVal`:
  `return ((p->outbits >> bitnum) & 1);`
  `IoSetOutBit` (line 214-216) and `IoClrOutBit` (line 232-234) update
  `p->outbits` then send `SET_OUTPUT` with that cached value.
- Logosol `LDCN.LIB` 1.52N line 4302:
  `bool IoOutBitVal(byte addr, int bitnum) { return ((mod[addr].kind.io.outbits >> bitnum) & 1); }`
  and line 4349, `IoGetOutputs`: `return ((unsigned int)mod[addr].kind.io.outbits);`
- `LdcnLib.PDF`: "IoOutBitVal ... Returns the most recently set state of an
  output bit", "IoGetOutputs ... Returns the most recently set state of an
  output byte of an I/O node", "IoGetPWMVal ... Returns the most recently set
  PWM value".

The only hardware-sourced output diagnostic on an I/O node is the LS-773
OUT_SH input bit. The servo module, by contrast, has a `FETCH_OUTPUTS 0x20`
flag (in the Set Outputs control byte) to read its output ports back; the I/O
module has nothing equivalent.

## 6. CNC-SK-2310g2 I/O map

Source: Doc# 710231005 Rev D, 03/05/2020,
<https://logosolinc.com/wp-content/uploads/2020/08/LS-2310g2-Supervisor-IO-Controller.pdf>.
The datasheet documents **no command set, status byte, or Define Status list**;
only this application-level map. The LDCN command set is inferred from the
LS-773 lineage.

Digital inputs:

| Byte | Bit | Input | Function | Connector | Application |
|------|-----|-------|----------|-----------|-------------|
| 0 | 0 | Input 0 | Non Dedicated | CN14 | Program run |
| 0 | 1 | Input 1 | Non Dedicated | CN14 | Program stop |
| 0 | 2 | Input 2 | Spindle OFF (Note 1) | CN6 | Spindle OFF |
| 0 | 3 | Input 3 | Non Dedicated | CN6 | Spindle fault |
| 0 | 4 | Input 4 | Non Dedicated | CN6 | Spindle at speed |
| 0 | 5 | Input 5 | Non Dedicated | CN7 | Air pressure |
| 0 | 6 | Input 6 | Non Dedicated | CN7 | Measure switch |
| 0 | 7 | Input 7 | Non Dedicated | CN7 | Tool changer closed |
| 1 | 0 | Input 8 | At Home (Note 2) | N.A. | At Home |
| 1 | 1 | Input 9 | Test Mode | N.A. | Test Mode |
| 1 | 2 | Input 10 | Servo Fault | CN3 Safety BUS | Servo Fault |
| 1 | 3 | Input 11 | Status 0 | N.A. | LED1 |
| 1 | 4 | Input 12 | Status 1 | N.A. | LED2 |
| 1 | 5 | Input 13 | Status 2 | N.A. | LED3 |
| 1 | 6 | Input 14 | Status 3 | N.A. | LED4 |
| 1 | 7 | Input 15 | Status 4 | N.A. | LED5 |

Digital outputs:

| Byte | Bit | Output | Function | Connector | Application |
|------|-----|--------|----------|-----------|-------------|
| 0 | 0 | Output 0 | Non Dedicated | CN14 | Program running lamp |
| 0 | 1 | Output 1 | Non Dedicated | CN14 | Program stopped lamp |
| 0 | 2 | Output 2 | Spindle ON (Notes 3, 4) | CN6 | Spindle ON |
| 0 | 3 | Output 3 | Non Dedicated | CN6 | Spindle direction |
| 0 | 4 | Output 4 | Non Dedicated | CN6 | Spindle DC-braking |
| 0 | 5 | Output 5 | Non Dedicated | CN7 | Tool clamp |
| 0 | 6 | Output 6 | Non Dedicated | CN7 | Spindle motor cooling |
| 0 | 7 | Output 7 | Non Dedicated | CN7 | Tool cooling |
| 1 | 0 | Output 8 | Non Dedicated | CN7 | Tool changer unlock |
| 1 | 1 | Output 9 | Cover Lock | CN9, CN10 | Cover 1, 2 lock |
| 1 | 2 | Output 10 | See Automation modes | N.A. | Home Enable (alt.) |
| 1 | 3 | Output 11 | Test Mode Inhibit | N.A. | Test Mode Inhibit |
| 1 | 4 | Output 12 | Safety Link Bridge | CN3 Safety Bus | Safety Link Bridge |
| 1 | 5 | Output 13 | Non Dedicated, Inverted | CN7 | N.A. |
| 1 | 6 | Output 14 | Reserved. Set to 0 | N.A. | Covers Lock/Unlock (alt., Note 5) |
| 1 | 7 | Output 15 | System Lock | N.A. | Power ON/OFF (alt., Note 6) |

Notes (verbatim): 1. Spindle OFF = 1 when Spindle ON (Outputs/Byte0/Bit2) = 0
and Spindle Stopped (CN6 pin 2) = HIGH. 2. At Home is set 0 when Test Mode with
Acknowledge is active. 3. Spindle ON and Safety Link Bridge cannot be used
simultaneously. 5. J10-2 and J19 must be installed. 6. J21 must be installed.

The SK-2310g2 uses both output bytes (16 outputs) where the LS-773 uses seven
bits of one. Inputs 11-15 mirror the five front-panel LEDs, so the diagnostic
state the LEDs encode is readable through the ordinary `SEND_INPUTS` item.

## 7. Things Logosol leaves out that JR Kerr defines

1. Command 0x0 Set Direction and its two-byte direction layout.
2. Set Address byte 1 range 0-0xFF (Logosol: 0x00-0x7F) and the ADDR_OUT
   daisy-chain enable on the first Set Address after reset.
3. Why "synch output" is 0x5: it matches PIC-SERVO's start-motion command so
   one group command synchronises motion and I/O.
4. The `IOMOD` host structure, making explicit that output, PWM and timer-mode
   "readback" is cached host state.
5. The baud divisor table and that it is the raw PIC SPBRG value.
6. The complete library source (`nmclib04`), including the exact `IoGetStat`
   parse order and the `MAXSIOERROR 2` give-up threshold.

Neither vendor defines status-byte bits 0 or 2-7 for the I/O module type.

## 8. Hypotheses to test on the bus

Each item is a specific packet or observation. Capture with the RS-485 tap
while the Logosol utility (`Ldcn.exe`, 2020) talks to the SK-2310g2, then
replay from pyldcn.

1. **Standard enumeration.** The utility sends `AA 00 21 01 FF <ck>` (Set
   Address) then `AA 01 13 20 <ck>` (Read Status, SEND_ID). Expected reply:
   status, type, version, checksum. Record the SK-2310g2 type byte: 2 means it
   identifies as a plain I/O module; any other value means a distinct module
   type the DLL parses differently.
2. **Two-byte status mask.** `LdcnReadStatus2` / `LdcnDefineStatus2` imply the
   utility may send `AA <addr> 23 <mask0> <mask1> <ck>` (Read Status with two
   data bytes). Test whether the SK-2310g2 accepts a 2-byte 0x3 and returns
   extra bytes, or sets status bit 1 (checksum/format error) or returns only
   status + checksum. The LS-773 should reject it.
3. **Extra status items.** With a 2-byte mask, walk `mask1` bits 0-7
   individually and count returned bytes. Any non-zero count is an undocumented
   status item (candidates: output readback, safety-bus state, relay state,
   fault code).
4. **Status-byte bits 0, 2-7.** Log the status byte across every fault in
   `sk-2310g2_fault_induction_protocol.md`. If any bit other than bit 1 ever
   changes, the "undefined" bits are live on this firmware.
5. **OUT_SH analogue.** LS-773 Byte 1 bit 7 is OUT_SH. On the SK-2310g2 that
   bit is documented as Input 15 / Status 4 / LED5. Short an output to POWER(+)
   and check whether LED5 and Input Byte 1 bit 7 assert, which would mean the
   short-protection flag is multiplexed onto Status 4.
6. **Device version.** Read Status with 0x20 returns version 50 on an LS-773.
   Record the SK-2310g2 version; `LdcnGetModSubVer` suggests a sub-version
   byte, possibly returned as a third ID byte or via a different item bit.
7. **Set Outputs width.** Send `AA <addr> 26 <b0> <b1> <ck>` with bit patterns
   in byte 1. Confirm all 16 outputs respond (LS-773 ignores byte 1) and
   confirm "Reserved. Set to 0" bit 6 has no visible effect.
8. **Output readback absence.** After Set Outputs, issue Read Status with every
   defined item (0xFF) and confirm none of the returned bytes reflect the output
   state. Confirms host-cached readback for the SK-2310g2 as for the LS-773.
9. **Packet comm mode.** `LdcnSetPacketCommMode` and `LdcnSetFastSerial` exist
   in the 2020 DLL. Watch for any command code 0x9, 0xB or 0xD (unused in
   PIC-I/O) in utility traffic; those are the free slots a vendor extension
   would occupy.
10. **Timer mode.** Set Timer Mode (0x18) with bit 1 = 1 and count edges on
    Input 9 (Test Mode). If the counter advances, the SK-2310g2 kept the LS-773
    counter wiring on bit 9.

## 9. Vendor files to preserve

Store these under `logosol-docs/vendor-software/` (outside git; the folder is
empty until fetched). Fetch with `curl -L`:

- Logosol LDCN SDK: <https://logosolinc.com/wp-content/downloads/ldcnlib.exe>
  (zip: `Ldcnlib.dll` 2002, `Ldcnlib.lib`, `LdcnLib.PDF`).
- Logosol LDCN Utility: <https://logosolinc.com/wp-content/downloads/LdcnUtility.zip>
  (`Ldcn.exe` 2020, `Ldcnlib.dll` 2020, `LDCN.HLP`, `LDCN.GID`, `*.ini`).
- JR Kerr NMC library with source:
  <http://web.archive.org/web/2016id_/http://www.jrkerr.com/nmclib04.zip>
  (`picio.cpp/.h`, `nmccom.cpp/.h`, `picservo.cpp/.h`, `picstep.cpp/.h`,
  `sio_util.cpp/.h`, `NMCLIB04.pdf`, `NMCLIB04.dll`).
- JR Kerr PIC-I/O datasheets:
  <http://web.archive.org/web/2016id_/http://www.jrkerr.com/piodata.pdf>,
  <http://web.archive.org/web/2016id_/http://www.jrkerr.com/picio.pdf>,
  <http://web.archive.org/web/2016id_/http://www.jrkerr.com/pioboard.pdf>.
- Logosol Dynamic C library `LDCN.LIB` 1.52N, mirrored at
  <https://raw.githubusercontent.com/cversek/linux-ldcn/master/doc/logosol/LOGOSOL_RABBIT_DYNAMIC_C_Ldcn.Lib>.

Recommended next step: Ghidra analysis of `Ldcn.exe` and the 2020
`Ldcnlib.dll`. Targets: the `IoGetStat` byte-count table (whether a module type
other than 2 gets a different parser), the implementation of
`LdcnReadStatus2` / `LdcnDefineStatus2` (mask width and which command byte they
emit), and any string table naming SK-2310g2 status fields. The 2002 DLL and
JR Kerr's source give known-good reference symbols for diffing.
