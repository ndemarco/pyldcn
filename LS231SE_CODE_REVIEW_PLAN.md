# pyldcn LS-231SE Code Review Plan

Prepared after reading the LDCN and LS-231SE docs (ignore obsolete design docs). Focus: verify the driver matches the protocol and hardware behavior of the 231SE servo drive.

## Context Snapshot (docs shorthand)
- Packets: `0xAA` header, addr 1–127 (groups 128–255), cmd/len nibble, checksum sum(addr..data)&0xFF; group commands usually silent except leader.
- Core commands: gains (0x06), trajectory load/start (0x04/0x05), stop (0x07), IO control (0x08), homing (0x09), clear sticky bits (0x0B), path points (0x0D int8.frac8), extended 0x0E (limit-stop, hall init, HW sync, watchdog, motor error limit).
- Status: status/aux bits; status items bitmap selects appended data (pos, vel, aux, pos_err, path_count, device_id, home, watchdog, motor_pos, etc.); sticky bits require Clear Bits.
- IO/Signals: HomeSEL (OUT4/OUT8) remaps bits 5/6 among Limit1/Limit2/HomeIN/Input10/11; inputs include SafetyLINK, DE; outputs include brake mode, relays, SmartSTOP, mode bits.
- Timing (per docs): 2 s after hard reset, 300 ms after addressing, 100 ms around baud changes; default 19200 baud, recommend 125000 baud for routine comms.

## Review Goals
- Confirm code encodes/decodes packets and status exactly per docs, including LSB order, path_count, watchdog, sticky bit clearing, checksum, and group command expectations.
- Ensure motion, homing, path, IO, and safety behaviors match the manual (trajectory flags, homing control bits, path buffer timing, hardware sync, watchdog, limit stop).
- Validate state/diagnostics mapping (HomeSEL-dependent signals, status → diagnostic conditions, safety bus interactions).
- Identify timing/robustness gaps (command spacing, retries, baud changes, reset flows) and testing coverage needed.

## Detailed Review Steps
1) **Protocol Layer**  
   - Inspect `pyldcn/protocol.py`, `network.py`, `device.py` for packet assembly, checksum, cmd/len nibble handling, group-address rules, baud change handling, inter-command spacing, and serial error handling.
   - Findings:
     - Group commands only treat `0xFF` as “no response expected.” Group addresses 128–254 without a leader will raise timeouts even though silence is valid, breaking group-triggered multi-axis starts.
     - Baud change flow doesn’t enforce a pre-send close. Could miss the baud switch or leave the port at the old baud.
     - No clamp on data length (spec 0–16), and received status length is not validated beyond checksum, so malformed packets could slip through or be sent.
     - Auto-detect uses limited addresses (1/2/3/6); detection may falsely fail.

2) **Servo Architecture Glue**  
   - Read `devices/servo.py` constructor/init flow vs the 7-step init in docs; check public API surface and how subsystems share `ServoState`.
3) **Status Parsing & Diagnostics**  
   - Deep dive `devices/status/servo_status.py`, `servo_mappings.py`, `servo_diagnostics.py`: bit→field mapping accuracy (status + aux), HomeSEL resolution, path_count/order, watchdog/motor_pos parsing, sticky bit tracking/clearing, diagnostic pattern tables, and error-class reporting.
4) **Motion & Trajectory**  
   - Audit `devices/servo_motion.py`: trajectory control byte flags (servo vs PWM, profile/velocity mode, direction, start-now), load/start sequencing, scale conversions, accel/velocity units, stop behaviors (0x07 modes), reset position/home save, and adherence to command spacing. Check handling of current/pos error sticky bits.
5) **Path Mode**  
   - Review add/clear/start path functions and path point encoding (int8.frac8, buffer depth 256, timer from IO control, aux path_mode bit, path_count status). Verify host refill strategy and multi-axis/group start support.
6) **Homing**  
   - Examine homing helpers in motion/servo: encoding of 0x09 bits (limit1/2, index, stop type), two-stage home flows, status polling for `home_in_progress`, and position reset/save. Confirm limit switch semantics match IO mapping.
7) **IO & HomeSEL**  
   - Inspect `devices/servo_io.py`: brake output, generic outputs, limit/home input reads, HomeSEL outputs, limit relay control, SafetyLINK/DE usage, and how inputs map into state fields.
8) **Safety & Extended Commands**  
   - Check `devices/servo_safety.py`: extended 0x0E subcommands (limit-stop, hall init, repeat answer, HW sync enable/disable, watchdog setup/reset, motor error limit), emergency stop patterns, and interaction with safety bus expectations.
9) **Cross-Device Interactions**  
   - Skim SK-2310g2/LS-773 modules for safety bus and power control integration points (e.g., servo fault relay expectations, power on/off flows) to ensure servo-side signals align.
10) **Examples & Tests**  
   - Run through `examples/` scripts for coverage of init, status, homing, coordinated motion; note missing scenarios (HW sync, watchdog, limit-stop) and plan test additions or mocks.

## Known Gotchas from Documentation Review

- ### 1. Little-Endian Data
  - All multibyte fields are little-endian; path points use int8.frac8 (LSB fractional).  
  - [ ] Verify packing/unpacking uses `<` formats and path points encode frac-first.

- ### 2. Status Bit Context
  - Bits 5/6 depend on HomeSEL (OUT4/OUT8) routing; aux bit0 can be diagnostic.  
  - [ ] Resolver must use current HomeSEL to map Limit/Home/Input10/11; update `ServoState` when HomeSEL changes.

- ### 3. Sticky Bits
  - Sticky: status bits 1/2/4 and aux bits 1/5; clear with 0x0B.  
  - [ ] Ensure faults call Clear Bits and tests assert persistence/clear.

- ### 4. SK-2310g2 Init Dependencies
  - Init SK-2310g2 first; require diag 0x14-0x17 before power-on; pulse Out15 100ms if J21; interlock Out2 vs Out12; expect ≤5s to diag 0x18-0x1F.

- ### 5. LS-231SE 7-Step Init
  - Define Status → Load Gains → Load Trajectory (pos0) → Stop/enable → Reset Position → Clear Bits → Read Status.  
  - Timing: ≥10ms between cmds, 2s after hard reset, 300ms after addressing, 500ms around baud change.  
  - Verify mode bits = 0000 (LDCN single loop); send gains (don’t rely on NVRAM).

- ### 6. Fault Recovery Matrix
  - Sticky bits → Clear Bits (0x0B).  
  - Encoder/EEPROM diags → Hard Reset (0x0F) + re-init.  
  - Safety Link low → clear at source, clear sticky bits, ensure chain high.  
  - SK-2310g2 diag faults → address cause/jumpers, then clear.

## Deliverables
- Findings/issues list per module against the docs above.
- Recommendations for fixes/tests (e.g., command packing, timing guards, missing fields, HomeSEL handling, safety/limit behaviors).

## HAL Bridge Suitability (pyldcn ↔ LinuxCNC)
**Goal:** Evaluate pyldcn as a Logosol→LinuxCNC HAL bridge and outline what must be exposed and monitored.

- **Pins to expose (LinuxCNC HAL signals)**: position/velocity/pos_error, move_done, servo_on, power_on, current_limit*, pos_error*, pos_wrap*, servo_overrun*, home_in_progress, path_mode, home/limit switches (resolved via HomeSEL), SafetyLINK/servo_fault indication, path_count, watchdog status, brake control, outputs (brake/user relays), diagnostic code (SK-2310g2), power_ready/power_enabled (from SK-2310g2), emergency-stop chain status.
- **Path point mode in HAL**: ensure I/O Control path timer config flows through; buffer management loop in user space to keep 231SE 256-point buffer fed; group start for multi-axis sync; convert LinuxCNC trajectory planner deltas to int8.frac8 increments; HAL component needs rate guardrails and back-pressure when path_count low; handle start/stop/clear semantics aligned with 0x0D/0x05.
- **Fault/status surfacing**: map status+aux+diagnostic to HAL pins and LinuxCNC estop/fault chain; assert LinuxCNC E-stop if SafetyLINK/drop, SK-2310g2 diagnostic faults, LS-231SE sticky faults (pos_error/current_limit/servo_overrun/pos_wrap), limit triggers; expose textual status for UI; ensure checksum errors trigger retries and a visible warning.
- **Initialization flow**: hard reset; addressing; baud change; SK-2310g2 ready/power control; 7-step 231SE init (define_status, gains, zero traj, enable amp, reset pos, clear faults, verify); power on via Out15 pulse or operator prompt; confirm mode bits (LDCN single loop); set HomeSEL outputs; configure watchdog/hw sync; enforce timing gaps.
- **Troubleshooting approach**: isolate pyldcn first (serial captures, checksum/error bits, raw NOP/read_status, baud/addresses, HomeSEL mapping, watchdog/hw sync). Then HAL layer (observe pins, E-stop chain mapping, path_count cadence, planner deltas vs on-wire, inject limit/faults). Provide a minimal diagnostic HAL config to replay canned command/status sequences.
