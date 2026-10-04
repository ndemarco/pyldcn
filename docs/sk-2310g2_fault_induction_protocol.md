# SK-2310g2 fault induction and capture protocol

Purpose: induce every supervisor state on the bench while the vendor LDCN Utility
polls the CNC-SK-2310g2, capture the serial traffic with USBPcap, and diff the
status replies against a baseline to find status fields the datasheet does not
document. Pins and jumpers are from Doc 710231005 Rev. D (supervisor) and Doc
712231004 Rev. A (LS-231SE).

## 1. Bench prerequisites

- Supervisor: CN1 pin 2 = +24V, pin 1 = GND from a bench supply. CN1 pin 3
  (UM, motor power) from a second adjustable supply. J2-4 OFF (UM monitored).
- LS-832RL on CN5 (LDCN HOST) to the Windows PC through the USB serial adapter
  USBPcap will watch. Note the adapter chip (FTDI or Prolific); it sets the
  payload offset in section 5.
- One LS-231SE on CN4 (LDCN SLAVE). Drive Enable/Stop (drive CN3 pin 4 and CN4
  pin 4) from supervisor CN3 pin 3. Safety loop: supervisor CN3 pin 1 (Safety
  Link OUT) through the drive FAULT relay back to CN3 pin 2 (Safety Link IN).
  CN3 pin 4 (ServoFAULT) open.
- Harness: every contact input is a removable jumper wire so one contact can be
  opened alone. CN11 1-2 (EMG B), CN11 3-4 (EMG A); CN13 1-2/3-4 and CN15
  1-2/3-4 jumpered closed and left alone; CN8 1-2 (Home A), 3-4 (Home B); CN9
  1-2 (Cover1 A), 5-6 (Cover1 B); CN10 1-2 (Cover2 A), 5-6 (Cover2 B); CN13
  9-10 (ACKN A), 11-12 (ACKN B); CN15 5-6 (POW, momentary); CN16 5-6 (Monitor
  Loop); CN6 pin 2 to CN6 pin 8 (+24V) for Spindle Stopped HIGH; CN15 pin 11
  (Test Mode A) and pin 13 (Test Mode B) on a changeover switch to pin 12
  (+24V) so A rises as B opens within 100 ms.
- Baseline for 0x1F: EMG A and B closed, Home A closed, Home B open, all four
  cover contacts closed, Monitor Loop closed, Spindle Stopped HIGH, Test Mode B
  HIGH and A open, safety loop intact.

Record (photograph) before the first capture:

| Jumper | Sets | Record |
|---|---|---|
| J1 | LDCN mode, watchdog OFF (pos 5) or ON (pos 6); 3-4 open | position |
| J2 | Power OFF delay 1/2/4 s (pos 1-3); pos 4 ON = UM not monitored | all four |
| J10 | 1,4 zero-speed options; 2 lock/unlock buttons on CN22; 3 spindle in Test Mode | each pin |
| J14, J15 | Cover 2 lock polarity (1-2 powered unlocked, 2-3 powered locked) | both |
| J16 | 1-2 Spindle ON in Test Mode + Ack; 2-3 off when covers open | position |
| J17 | CN13 pin 7: 1-2 GND, 2-3 Unlock Enable (-) | position |
| J18 | must be open | confirm |
| J19 | open = unlock button CN13 5-6; short = Unlock Enable / Output 14 | state |
| J20 | open = Test Mode unlock needs spindle stopped; short = any time | state |
| J21 | short = Power ON by Output 15 (Byte1/Bit7) 1-to-0 edge | state |

Baseline capture: start USBPcap on the adapter's root hub, start the utility,
connect, open the status page, poll idle 30 s at 0x1F. Save as
`1F_baseline_<yyyymmdd>.pcapng`. From it note the poll request the utility
sends: command byte (expect Read Status, low nibble 0x3, with the LS-773 style
`FF FF` define-status mask or whatever is actually sent), period, reply length.
Every diff is against replies to that request. Take a second baseline at 0x17
(power off, covers closed); half the states below need power off.

## 2. Diagnostic codes 0x00-0x1F

"Overloaded" = several causes share one code, differentiated only by LED blink
or utility text. "No read path" = no documented byte/bit for the detail.

| Code | Datasheet meaning | Overloaded / no read path | Induce | Caveat | Observe |
|---|---|---|---|---|---|
| 0x00 | Power OFF delay in progress | transient | From 0x1F open CN11 1-2 and 3-4 together; capture the J2 delay | Enable drops first, UM holds for the delay | Timer field? |
| 0x01 | Initializing | no | Cycle CN1 24V with capture running | Utility loses link; start capture first | Init-only bytes |
| 0x02 | Control voltage shorted | which source: no read path | Short CN7 pin 2 (+24V, short protected) to CN7 pin 8 via 1 A fuse | Only protected sources; never CN21 24F | Per-source bit? |
| 0x03 | Output shorted | which output: no read path | Set Output 5 = 1, short CN7 pin 7 to pin 8 via fuse; repeat Output 8 (CN7 pin 13) | Brief contact | Output index? |
| 0x04 | Control voltage LOW (<18 V) | no | Bench supply to 17 V, then 16 V | Link may drop below ~15 V | Analog field |
| 0x05 v1 | Home switch malfunction (both ON) | overloaded, 2 causes | Close CN8 1-2 and 3-4 together | Power "prior": do from 0x1F and 0x17 | Text: Home vs Test Mode |
| 0x05 v2 | Test Mode switch malfunction | overloaded | CN15 pin 11 HIGH and pin 13 HIGH together | Power goes OFF | Same |
| 0x06 | Power UP Home error | no | Power up with CN8 1-2 open, 3-4 closed; also both open | Needs power cycle | vs 0x05 |
| 0x07 | Power UP Test Mode error | no | Power up with CN15 pin 11 HIGH, pin 13 open | Same | Same |
| 0x08 | System LOCKED | no | Set Output 15 (Byte1/Bit7) = 1 in utility, then clear | J21 short: the 1-to-0 edge powers on | Lock bit |
| 0x09 | Watchdog Stop | no | J1 pos 6, stop polling (disconnect) past timeout, reconnect | Restore J1 after | Watchdog flag |
| 0x0A | Safety Link Error | which drive: no read path | Lift CN3 pin 2. Variant: keep loop, remove LS-231SE motor power so its FAULT relay opens ("No Motor Power", relay open) | - | Link IN separate from ServoFAULT (Byte1/Bit2)? |
| 0x0B | Cover open, spindle not stopped | overloaded: contacts OK vs malfunction (blink only) | CN6 pin 2 low, open Cover1 A and B together. Variant: CN9 1-2 only | No real spindle; lamp only | "Contacts OK" vs "malfunction" bit? |
| 0x0C | Cover open, not at home | overloaded, same | Open CN8 1-2, power on, open both Cover2 contacts. Variant: CN10 1-2 only | - | Same |
| 0x0D | Cover open, Test Mode, no acknowledge | overloaded, same | Enter Test Mode (B open, A HIGH within 100 ms, Output 11 = 0), ACKN at rest, open Cover1 A+B. Variant: A only | Test Mode enables drives; limit UM current | Same |
| 0x0E | Cover contact fault | which cover/contact: no read path | At 0x17 open CN9 1-2 only, wait >100 ms. Repeat CN9 5-6, CN10 1-2, CN10 5-6 | Power "prior" | Four distinct values? |
| 0x0F | Limit Switch Stop | which drive/limit: no read path | On LS-231SE pull Limit1 (drive CN9 pin 4) low; then Limit2 (CN9 pin 2) | Disabled in zero-speed mode (J10-1/4) | Limit identity |
| 0x10 | Emergency Stop | no | Open CN11 1-2 and 3-4 together | - | Reference for 0x11 |
| 0x11 v1 | E-stop contact malfunction (one line open) | overloaded, 2 causes | Open CN11 3-4 only (EMG A); second capture CN11 1-2 only (EMG B) | - | Does text name A or B |
| 0x11 v2 | Monitor Loop open after E-stop | overloaded | Full E-stop, then open CN16 5-6 | - | Loop bit |
| 0x12 v1 | Busy (<6 s) | overloaded, 3 causes | Press POW (CN15 5-6); capture the 6 s | - | Busy timer |
| 0x12 v2 | Power ON button short (>6 s) | overloaded | Hold CN15 5-6 closed 10 s | - | Button bit |
| 0x12 v3 | Monitor Loop open (relay malfunction) | overloaded | Open CN16 5-6 at 0x17, press POW | - | Loop bit |
| 0x13 | Motor power under-voltage | no | At 0x1F lower UM until it trips (J2-4 OFF) | Drive sees PowerDROP, FAULT follows | UM analog/flag |
| 0x14-0x17 | Cover 1/2 open-closed matrix, ready to power | individual A/B contacts: no read path | Power off; open both contacts of cover 1, cover 2, both, neither | - | Confirm cover bits vs 0x0E |
| 0x18-0x1B | Same matrix, Test Mode | - | Enter Test Mode, press ACKN, repeat matrix | Drives enabled | Byte1/Bit1 |
| 0x1C-0x1E | Same matrix, at home, spindle stopped, power on | - | At 0x1F open covers per matrix | - | Byte1/Bit0 |
| 0x1F | All closed, power on | - | Baseline | - | - |

## 3. States with no documented read path

| State | Point | Toggle | Look for in diff |
|---|---|---|---|
| Output 0-15 bits | Set Outputs command | At 0x1F set one output at a time, hold 20 s, clear. Never Output 2 and 12 together (note 3); Output 15 is 0x08. Meter Output 5 on CN7 pin 7 | A reply byte following the mask. None = utility echoes its own command |
| Safety Link IN | CN3 pin 2 | Lift at 0x17; if 0x0A still trips, compare with the 0x0A capture | Bit following the wire, apart from the code |
| Enable/Stop | CN3 pin 3, CN16 pin 8 | 0x1F vs 0x17; meter confirms | Bit tracking power, not UM |
| Power relay A/B | CN16 pins 2, 4; CN20 pins 2/3 | 0x1F vs 0x17; also 0x13 (relays ON, UM low) | Relay state vs UM present |
| Monitor Loop | CN16 5-6 | Open at 0x17 without POW | Bit before any code change |
| Covers CLOSED output | CN19 pin 1 | Open one contact (0x0E case) | AND of four contacts |
| Cover contacts A/B | CN9 1-2, 5-6; CN10 1-2, 5-6 | The four single-contact 0x0E captures | Four bit positions |
| E-stop A vs B | CN11 3-4 (A), 1-2 (B) | The two 0x11 v1 captures | Two bits |
| Acknowledge switch | CN13 9-10 (A), 11-12 (B) | In Test Mode hold ACKN 20 s; also A only | Ack bit; A/B separate? |
| Unlock input | CN13 pin 6 (button to pin 5) | Press at 0x17 and at 0x1C; J19 open | Input bit; Unlock Enable |
| Cover LOCK/UNLOCK | CN22 pin 2, pin 4 | J10-2 and J19 short; pull each to +24V | Two input bits |
| Test Mode request | CN15 pins 11/13 | Enter Test Mode without Ack | Byte1/Bit1 plus pending bit |

## 4. Capture hygiene

- One pcapng per state, 20 s, status page visible throughout. Name
  `<code>_<variant>_<yyyymmdd>.pcapng`: `11_emgA_20261004.pcapng`,
  `0E_cover1A_20261004.pcapng`, `OUT05_set_20261004.pcapng`,
  `1F_baseline_20261004.pcapng`.
- Beside each, `<same name>.txt`: utility text verbatim, LED pattern, jumper
  record, meter readings.
- Restore and confirm 0x1F (or 0x17) between states; re-baseline every five.
- Storage: `captures/` in github.com/ndemarco/picoshark, next to the dissector
  that reads them, with a `captures/README.md` index (file, code, variant,
  utility text, result). Files are tens of kB; plain git, no LFS. pyldcn
  references them for test fixtures.

## 5. Diff procedure

1. Extract payload. FTDI prefixes each IN transfer with two modem-status bytes;
   drop them. Prolific has no prefix.
   `tshark -r 11_emgA_20261004.pcapng -Y 'usb.transfer_type == 0x03 && usb.endpoint_address.direction == 1' -T fields -e frame.time_relative -e usb.capdata > 11_emgA.in.tsv`
   and `direction == 0` for host-to-device.
2. Reassemble each direction, split host frames on 0xAA, verify the checksum
   (address + command + data mod 256), pair each poll request with the reply
   that follows. Keep only replies to the baseline poll request. A short
   Python script in the captures repo.
3. Per capture, collect the set of values at each reply byte position over the
   20 s. Positions that vary in baseline are counters or analog; mask them.
4. XOR the modal reply against the modal baseline of the same power state.
   Each set bit is a candidate: record (byte, bit, capture, cause) in
   `bitmap.csv`.
5. A candidate must be set in every capture sharing the cause (both EMG-A
   captures, all cover-1-A captures) and clear elsewhere. Bits that flip only
   with the code are Byte1 bits 7:3; discard. Confirm survivors by re-inducing
   with pyldcn sending the same request and parsing the bit.
6. If the output tests show no reply change, the output state is only on the
   wire in the Set Outputs commands, not in status; record that.
