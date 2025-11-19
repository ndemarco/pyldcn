"""
LS-231SE IO/Status mapping descriptors.

Defines re-usable data structures for servo IO wiring and status decoding so
we can swap mappings when supporting additional servo models in the future.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple, Callable, Any, Optional

# ---------------------------------------------------------------------------
# IO Mapping
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class HomeSourceEntry:
    homesel2: int
    homesel1: int
    status_bit5: str
    status_bit6: str
    motor_latch: str
    master_latch: str


LS231SE_HOME_SOURCE_TABLE = (
    HomeSourceEntry(0, 0, "Limit1", "Limit2", "motor_phase_z", "master_phase_z"),
    HomeSourceEntry(0, 1, "HomeIN", "Input10", "motor_phase_z", "master_phase_z"),
    HomeSourceEntry(1, 0, "HomeIN", "Input10", "Input10_change", "Input10_change"),
    HomeSourceEntry(1, 1, "HomeIN", "Input11", "Input11_change", "Input11_change"),
)


@dataclass(frozen=True)
class LimitRelayEntry:
    bridge: int
    user_rel: int
    input11: Optional[int]  # None = don't care
    lcdn_state: str
    amplifier_state: str


LS231SE_LIMIT_RELAY_TABLE = (
    LimitRelayEntry(0, 0, 0, "depends_on_limits", "depends_on_limits"),
    LimitRelayEntry(0, 1, 0, "closed_bridge", "depends_on_limits"),
    LimitRelayEntry(0, 0, 1, "depends_on_limits", "closed_bridge"),
    LimitRelayEntry(1, 0, None, "user_rel_open", "user_rel_open"),
    LimitRelayEntry(1, 1, None, "user_rel_closed", "user_rel_closed"),
)


@dataclass(frozen=True)
class IOMapping:
    """Topology description for servo IO."""
    brake_mode_bit: int
    output_bits: Dict[int, str]
    home_source_table: Tuple[HomeSourceEntry, ...]
    limit_relay_table: Tuple[LimitRelayEntry, ...]


LS231SE_IO_MAPPING = IOMapping(
    brake_mode_bit=0,
    output_bits={
        0: "BrakeMODE",
        1: "Output1",
        2: "Output2",
        4: "HomeSEL1",
        5: "Bridge",
        6: "UserREL",
        7: "SmartSTOP",
        8: "HomeSEL2",
    },
    home_source_table=LS231SE_HOME_SOURCE_TABLE,
    limit_relay_table=LS231SE_LIMIT_RELAY_TABLE,
)

# ---------------------------------------------------------------------------
# Status Mapping / Profile
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ModeDescriptor:
    """Describes how control pins behave in a specific operating mode."""
    name: str
    description: str
    mode_bits: Tuple[int, int, int]
    de_behavior: str
    aen_behavior: str
    dir_behavior: str
    step_behavior: str
    fault_behavior: str


@dataclass(frozen=True)
class StatusProfile:
    """Describes how to decode status bits for a specific servo."""
    status_flags: Dict[str, int]
    aux_flags: Dict[str, int]
    status_items: Tuple[Tuple[int, str, int, str], ...]
    resolver: Callable[[int, int, Dict[str, Any]], Dict[str, Any]]
    modes: Tuple[ModeDescriptor, ...]
    default_mode: str
    mode_lookup: Dict[str, ModeDescriptor]


def ls231se_status_resolver(
    status_byte: int,
    aux_byte: int,
    context: Dict[str, Any],
) -> Dict[str, Any]:
    """
    Resolve LS-231SE status flags, considering HomeSEL overrides.

    Args:
        status_byte: Raw status byte from device.
        aux_byte: Raw auxiliary status byte.
        context: Extra data, e.g., {"homesel": (homesel2, homesel1)}.

    Returns:
        Dictionary with resolved flag values.
    """
    homesel = context.get("homesel", (0, 0))
    mode_key = context.get("mode")
    mode_descriptor = LS231SE_STATUS_PROFILE.mode_lookup.get(
        mode_key or LS231SE_STATUS_PROFILE.default_mode
    )
    table_lookup = {
        (entry.homesel2, entry.homesel1): entry
        for entry in LS231SE_HOME_SOURCE_TABLE
    }

    entry = table_lookup.get(homesel)
    resolved = {}

    # Base flags
    resolved["move_done"] = bool(status_byte & 0x01)
    resolved["cksum_error"] = bool(status_byte & 0x02)
    resolved["current_limit"] = bool(status_byte & 0x04)
    resolved["power"] = bool(status_byte & 0x08)
    resolved["pos_error"] = bool(status_byte & 0x10)
    resolved["home_source"] = bool(status_byte & 0x20)
    resolved["limit2"] = bool(status_byte & 0x40)
    resolved["home_in_progress"] = bool(status_byte & 0x80)

    resolved["index"] = bool(aux_byte & 0x01)
    resolved["pos_wrap"] = bool(aux_byte & 0x02)
    resolved["servo_on"] = bool(aux_byte & 0x04)
    resolved["accel_done"] = bool(aux_byte & 0x08)
    resolved["slew_done"] = bool(aux_byte & 0x10)
    resolved["servo_overrun"] = bool(aux_byte & 0x20)
    resolved["path_mode"] = bool(aux_byte & 0x40)

    # Dynamic mapping of StatusBit5/6 to physical signals (if known)
    if entry:
        resolved["home_source_signal"] = entry.status_bit5
        resolved["limit2_signal"] = entry.status_bit6
    else:
        resolved["home_source_signal"] = "Unknown"
        resolved["limit2_signal"] = "Unknown"

    resolved["mode"] = mode_descriptor.name if mode_descriptor else "UNKNOWN"
    resolved["mode_description"] = (
        mode_descriptor.description if mode_descriptor else "Unknown mode"
    )

    return resolved


_LS231SE_MODE_LIST = (
    ModeDescriptor(
        name="LDCN_SINGLE_LOOP",
        description="LDCN single-loop mode",
        mode_bits=(0, 0, 0),
        de_behavior="Returns PIC_AE (1 when power driver enabled)",
        aen_behavior="Routed to master encoder latch strobe (CAP6)",
        dir_behavior="Master encoder counter phase A (CAP4)",
        step_behavior="Master encoder counter phase B (CAP5)",
        fault_behavior="Depends on SW4; default LOW = enabled or fault present",
    ),
    ModeDescriptor(
        name="LDCN_DUAL_LOOP",
        description="LDCN dual-loop mode",
        mode_bits=(0, 0, 1),
        de_behavior="1 when PIC_AE set; LOW also indicates master encoder error",
        aen_behavior="Latch strobe with index pulse at 0.4µs",
        dir_behavior="Master encoder counter phase A",
        step_behavior="Master encoder counter phase B",
        fault_behavior="Same as single loop (SW4 dependent)",
    ),
    ModeDescriptor(
        name="ANALOG_SINGLE_DUAL",
        description="Analog input single/dual loop",
        mode_bits=(0, 1, 0),
        de_behavior="Mirror of AEN input",
        aen_behavior="0 disables amplifier; 1 enables",
        dir_behavior="Phase A input",
        step_behavior="Phase B input",
        fault_behavior="SW4 dependent; LOW = fault",
    ),
    ModeDescriptor(
        name="ANALOG_DIR_INVERT",
        description="Analog input mode with direction invert",
        mode_bits=(0, 1, 1),
        de_behavior="Table: AEN/Dir combos map to enable/invert states",
        aen_behavior="Same as above (with invert control)",
        dir_behavior="Determines inversion when combined with AEN",
        step_behavior="CAP6",
        fault_behavior="SW4 dependent",
    ),
    ModeDescriptor(
        name="ANALOG_ENABLE_POS_NEG",
        description="Enable Positive/Enable Negative analog mode",
        mode_bits=(1, 0, 0),
        de_behavior="Table: AEN/Dir combos set enable/inversion",
        aen_behavior="Inputs used for enable/invert latch",
        dir_behavior="Works with AEN to determine polarity",
        step_behavior="CAP6",
        fault_behavior="SW4 dependent",
    ),
    ModeDescriptor(
        name="QUADRATURE_ENCODER",
        description="Quadrature encoder mode",
        mode_bits=(1, 0, 1),
        de_behavior="Mirror of AEN input",
        aen_behavior="0 disables, 1 enables",
        dir_behavior="Master encoder counter phase A (CAP4)",
        step_behavior="Master encoder counter phase B (CAP5)",
        fault_behavior="SW4 dependent",
    ),
    ModeDescriptor(
        name="STEP_DIR",
        description="Step & Direction mode",
        mode_bits=(1, 1, 0),
        de_behavior="Depends on AEN/Dir table (0 disables)",
        aen_behavior="Enable control",
        dir_behavior="Direction input (positive/negative)",
        step_behavior="Step input (0->1 edges)",
        fault_behavior="SW4 dependent",
    ),
    ModeDescriptor(
        name="STEP_POS_NEG",
        description="Step Positive / Step Negative mode",
        mode_bits=(1, 1, 1),
        de_behavior="Mirror of AEN input",
        aen_behavior="0 disables, 1 enables",
        dir_behavior="Selects which step direction is active",
        step_behavior="0->1 transitions trigger positive/negative steps",
        fault_behavior="SW4 dependent",
    ),
)

_LS231SE_MODE_LOOKUP = {descriptor.name: descriptor for descriptor in _LS231SE_MODE_LIST}

LS231SE_STATUS_PROFILE = StatusProfile(
    status_flags={
        "MOVE_DONE": 0x01,
        "CKSUM_ERROR": 0x02,
        "CURRENT_LIMIT": 0x04,
        "POWER": 0x08,
        "POS_ERROR": 0x10,
        "HOME_SOURCE": 0x20,
        "LIMIT2": 0x40,
        "HOME_IN_PROGRESS": 0x80,
    },
    aux_flags={
        "INDEX": 0x01,
        "POS_WRAP": 0x02,
        "SERVO_ON": 0x04,
        "ACCEL_DONE": 0x08,
        "SLEW_DONE": 0x10,
        "SERVO_OVERRUN": 0x20,
        "PATH_MODE": 0x40,
    },
    status_items=(
        (0x0001, "position", 4, "Current position in encoder counts"),
        (0x0002, "ad_value", 1, "Analog-to-digital converter value (0-255)"),
        (0x0004, "velocity", 2, "Current velocity in counts per servo tick"),
        (0x0008, "aux", 1, "Auxiliary status byte"),
        (0x0010, "home", 4, "Captured home position in encoder counts"),
        (0x0020, "device_id", 2, "Device ID and firmware version"),
        (0x0040, "pos_error", 2, "Position following error in encoder counts"),
        (0x0080, "path_count", 1, "Path buffer count"),
        (0x1000, "watchdog", 2, "Watchdog timer status"),
        (0x2000, "motor_pos", 6, "Motor position and error (6 bytes)"),
    ),
    resolver=ls231se_status_resolver,
    modes=_LS231SE_MODE_LIST,
    default_mode="LDCN_SINGLE_LOOP",
    mode_lookup=_LS231SE_MODE_LOOKUP,
)
