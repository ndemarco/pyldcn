# LDCN Module Class Design

**Date:** 2025-10-29
**Status:** ✅ Implementation Complete - Awaiting Hardware Verification
**Last Updated:** 2025-11-10

---

## Purpose

This document describes the **design principles and architecture** for the pyldcn module. For implementation details, see the source code.

⚠️ **This is a design document, not a code reference.** The actual implementation is the Single Source of Truth (SSOT). Refer to source code for current API details.

---

## Class Hierarchy

```
LDCNNetwork                    # pyldcn/network.py
    │
    ├── devices: list[LDCNDevice]
    │
    └── LDCNDevice (ABC)       # pyldcn/network.py
            │
            ├── LS231SE        # pyldcn/devices/servo.py
            ├── SK2310g2       # pyldcn/devices/io.py
            └── LS773          # pyldcn/devices/io.py
```

---

## Design Principles

1. **Single Responsibility**: Each class handles one aspect (network, device, servo, I/O)
2. **Generic at Base**: LDCN protocol commands at network/base level
3. **Specific in Subclasses**: Device-specific commands in subclasses
4. **Single send_command()**: One implementation in LDCNNetwork, all else delegates
5. **Type Safety**: Full type hints for IDE support and validation
6. **Verification Tracking**: All methods marked UNVERIFIED until hardware tested
7. **Single Source of Truth**: Code is the authoritative reference, not documentation

---

## 1. LDCNNetwork Class

**Implementation:** [pyldcn/network.py](../../pyldcn/network.py)

**Purpose**: Manages serial communication and network-level LDCN protocol.

### Design Rationale

- **Single `send_command()` implementation**: All LDCN communication goes through one method to ensure consistent packet building, checksum calculation, and error handling.
- **Adaptive initialization**: Multiple initialization modes (VALIDATE, SOFT, READDRESS, FULL, AUTO) allow reconnection without disrupting device state or losing servo positions.
- **Baud rate flexibility**: Auto-detection and dynamic baud rate changes support both initial setup and performance optimization.

See [pyldcn/network.py](../../pyldcn/network.py) for complete API documentation.

---

## 2. LDCNDevice Base Class

**Implementation:** [pyldcn/network.py](../../pyldcn/network.py)

**Purpose**: Abstract base class defining the interface all LDCN devices must implement.

### Design Rationale

- **Delegation pattern**: Devices delegate `send_command()` to parent network, ensuring single point of protocol handling.
- **Abstract `read_status()`**: Forces device-specific subclasses to implement their own status parsing logic.
- **Generic operations**: Base class provides common LDCN commands (NOP, define status) used by all device types.

See [pyldcn/network.py](../../pyldcn/network.py) for complete API documentation.

---

## 3. LS231SE Class (Servo Drive)

**Implementation:** [pyldcn/devices/servo.py](../../pyldcn/devices/servo.py)

**Purpose**: Implements LS-231SE servo drive operations with motion control and fault monitoring.

### Design Rationale

- **7-step initialization sequence**: Required by hardware to properly configure PID gains, trajectory buffer, and amplifier state before motion.
- **Fault detection**: Status byte parsing identifies checksum errors, current limit, and position error conditions critical for safe operation.
- **Trajectory mode**: Uses hardware trajectory generator for smooth motion with velocity and acceleration control.

See [pyldcn/devices/servo.py](../../pyldcn/devices/servo.py) for complete API documentation.

---

## 4. SK2310g2 Class (I/O Controller / Supervisor)

**Implementation:** [pyldcn/devices/io.py](../../pyldcn/devices/io.py)

**Purpose**: Implements SK-2310g2 I/O controller, configured as supervisory controller with safety and spindle control.

### Design Rationale

- **Application-specific configuration**: Generic I/O device configured for specific machine roles (safety monitoring, power control, spindle).
- **Diagnostic decoding**: Translates hardware diagnostic codes into human-readable fault descriptions.
- **Safety integration**: Monitors e-stop, guard doors, and safe zones as part of machine safety system.

See [pyldcn/devices/io.py](../../pyldcn/devices/io.py) and [supervisory_controller.md](../supervisory_controller.md) for complete documentation.

---

## 5. LS773 Class (Network I/O Node)

**Implementation:** [pyldcn/devices/io.py](../../pyldcn/devices/io.py)

**Purpose**: Implements LS-773 Network I/O Node for general-purpose I/O, PWM, and timing operations.

### Design Rationale

- **Versatile I/O**: 10 inputs + 7 outputs support diverse machine interfacing needs.
- **PWM generation**: Hardware PWM at 20 KHz enables motor speed control and other applications.
- **Timing capabilities**: 32-bit counter/timer with prescaler for precise timing and frequency measurement.

See [pyldcn/devices/io.py](../../pyldcn/devices/io.py) for complete API documentation.

---

## 6. Communications Management Layer (Future)

**Purpose**: Stealthy debug/monitoring with minimal impact.

### Features (Phase 3)

- Async status monitoring thread
- Configurable verbosity levels (ERROR, WARN, INFO, DEBUG, TRACE)
- Pluggable logging backends (console, file, network)
- Performance metrics (timing, errors, retries)
- Packet capture for wire-level debugging
- Intelligent scheduling to avoid disrupting 1kHz control loop

**Implementation**: Separate `LDCNDebugger` class or mixin.

---

## Future Work & Testing

1. **Hardware Verification Testing**
   - Test basic protocol methods (send_command, status reading)
   - Verify baud detection and network initialization
   - Test servo initialization and motion control
   - Verify SK2310g2 diagnostic reading and safety monitoring
   - Mark verified methods as 🟢 VERIFIED

2. **Create Test Suite**
   - Unit tests for protocol methods
   - Integration tests with hardware
   - Regression tests against original utilities

3. **Documentation**
   - Add hardware test results and notes
   - Document any deviations from original behavior
   - Create user guide with examples
