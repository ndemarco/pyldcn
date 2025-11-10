# LDCN Module Class Design

**Date:** 2025-10-29
**Status:** Phase 2 - Design

---

## Class Hierarchy

```
LDCNNetwork                    # Network manager (serial port, protocol)
    │
    ├── devices: list[LDCNDevice]
    │
    └── LDCNDevice (ABC)       # Base device class
            │
            ├── LS231SE        # Servo drive
            │
            └── SK2310g2       # I/O controller / supervisor
```

---

## Design Principles

1. **Single Responsibility**: Each class handles one aspect (network, device, servo, I/O)
2. **Generic at Base**: LDCN protocol commands at network/base level
3. **Specific in Subclasses**: Device-specific commands in subclasses
4. **Single send_command()**: One implementation in LDCNNetwork, all else delegates
5. **Type Safety**: Full type hints for IDE support and validation
6. **Verification Tracking**: All methods marked UNVERIFIED until hardware tested

---

## 1. LDCNNetwork Class

**Purpose**: Manages serial communication and network-level LDCN protocol.

### Properties

```python
port: str                          # Serial port path (e.g., '/dev/ttyUSB0')
baud_rate: int                     # Current baud rate (9600-1250000)
serial: Optional[serial.Serial]    # PySerial port object
devices: list[LDCNDevice]          # Discovered devices on network
timeout: float                     # Serial read timeout (default 0.1s)
```

### Methods

```python
# 🔴 UNVERIFIED - Initialization
def __init__(self, port: str) -> None:
    """
    Initialize LDCN network manager.

    The network will be opened at 19200 baud (default LDCN reset state).
    Use set_baud_rate() after initialization to upgrade to higher speeds.

    Args:
        port: Serial port path (e.g., '/dev/ttyUSB0')
    """

# 🔴 UNVERIFIED - Connection management
def open(self) -> None:
    """
    Open serial port at 19200 baud (LDCN default).

    All LDCN networks start at 19200 baud after reset.
    Use set_baud_rate() after initialization to upgrade speed.
    """

def close(self) -> None:
    """Close serial port and cleanup resources."""

def _open_port(self, baud: int) -> None:
    """
    Internal: Open serial port at specific baud rate.

    Args:
        baud: Baud rate (must be in BAUD_RATES dict)
    """

# 🔴 UNVERIFIED - Core protocol
def send_command(self, address: int, command: int, data: list[int] = []) -> bytes:
    """
    Send LDCN command packet and return response.

    This is the SINGLE source of truth for LDCN communication.
    All other send_command() methods delegate to this.

    Args:
        address: Device address (1-127) or group (128-255)
        command: LDCN command (0x00-0x0F)
        data: Data bytes (0-16 bytes)

    Returns:
        Response bytes from device (status + data + checksum)

    Raises:
        LDCNChecksumError: Response checksum mismatch
        LDCNTimeoutError: No response received
    """

# 🔴 UNVERIFIED - Baud rate management
def _try_baud(self, baud: int) -> bool:
    """
    Test if devices respond at specific baud rate.

    Returns:
        True if any device responds
    """

def auto_detect_baud(self, baud_list: Optional[list[int]] = None) -> int:
    """
    Auto-detect current network baud rate.

    Args:
        baud_list: List of baud rates to try (default: common rates)

    Returns:
        Detected baud rate

    Raises:
        LDCNDetectionError: No response at any baud rate
    """

def set_baud_rate(self, baud: int) -> None:
    """
    Upgrade network baud rate for all devices.

    This is called AFTER initialize() to upgrade from 19200 to a higher speed.

    Steps:
    1. Send SET_BAUD command to group address 0xFF
    2. Close serial port
    3. Wait 500ms
    4. Reopen serial port at new baud rate

    Args:
        baud: Target baud rate (must be in BAUD_RATES dict)

    Raises:
        ValueError: If baud rate not supported
        LDCNError: If upgrade fails

    Example:
        network.initialize()  # At 19200 baud
        network.set_baud_rate(125000)  # Upgrade to 125kbps
    """

# 🔴 UNVERIFIED - Network initialization
def reset(self) -> None:
    """
    Send hard reset to all devices.

    Devices return to address 0x00 and 19200 baud.
    Waits 2 seconds after reset.
    """

def address_devices(self, max_devices: int = 127) -> int:
    """
    Sequentially address devices on network.

    Sends SET_ADDRESS to address 0x00 repeatedly until no response.
    Each successful command enables the next device in the daisy chain
    and assigns it the next sequential address (1, 2, 3, ...).

    Args:
        max_devices: Maximum address to try (default 127, safety limit)

    Returns:
        Number of devices successfully addressed

    Example:
        num_found = network.address_devices()
        # Devices are now at addresses 1, 2, 3, ..., num_found
    """

def discover_devices(self, start_address: int = 1, end_address: Optional[int] = None) -> list[dict]:
    """
    Discover device types and versions on the network.

    Queries each address using READ_STATUS (0x3) with status bit 5 (device ID).
    Returns device information without creating device objects.

    Args:
        start_address: First address to query (default: 1)
        end_address: Last address to query (default: last addressed device)

    Returns:
        List of device info dictionaries:
        [
            {
                'address': int,
                'device_id': int,      # Device type ID from hardware
                'version': int,        # Firmware version
                'responding': bool     # True if device responded
            },
            ...
        ]

    Example:
        devices = network.discover_devices()
        # [{'address': 1, 'device_id': 0x17, 'version': 0x23, 'responding': True},
        #  {'address': 2, 'device_id': 0x17, 'version': 0x23, 'responding': True},
        #  ...]
    """

def verify_devices(self, device_list: list[dict]) -> list[int]:
    """
    Verify devices are still responding.

    Sends NOP command to each device in the list to confirm communication.

    Args:
        device_list: List of device info dicts from discover_devices()

    Returns:
        List of addresses that responded successfully

    Example:
        devices = network.discover_devices()
        responding = network.verify_devices(devices)
        print(f"Responding addresses: {responding}")
    """

def create_device_objects(self, device_list: list[dict]) -> list[LDCNDevice]:
    """
    Create device objects from device info list.

    Maps device IDs to appropriate classes (LS231SE, SK2310g2, etc.)
    and populates self.devices list.

    Args:
        device_list: List of device info dicts from discover_devices()

    Returns:
        List of LDCNDevice objects

    Example:
        device_info = network.discover_devices()
        network.create_device_objects(device_info)
        # Now network.devices is populated
    """

# 🔴 UNVERIFIED - Complete initialization
def initialize(self, create_objects: bool = True) -> tuple[int, list[dict]]:
    """
    Complete network initialization sequence at 19200 baud.

    Steps:
    1. Hard reset all devices (at 19200 baud)
    2. Wait 2 seconds
    3. Address devices sequentially (assigns addresses 1, 2, 3...)
    4. Discover device types and versions (queries each device)
    5. Verify all devices are responding
    6. Optionally create device objects and populate self.devices

    This initializes the network at 19200 baud. Use set_baud_rate()
    afterwards to upgrade to a higher speed if desired.

    Args:
        create_objects: If True, create device objects and populate self.devices

    Returns:
        Tuple of (num_devices, device_info_list)
        - num_devices: Number of devices addressed
        - device_info_list: List of device info dicts from discover_devices()

    Raises:
        LDCNInitializationError: If initialization fails

    Example:
        network = LDCNNetwork('/dev/ttyUSB0')
        network.open()
        num_devices, device_info = network.initialize()  # At 19200 baud
        print(f"Found {num_devices} devices:")
        for dev in device_info:
            print(f"  Address {dev['address']}: ID=0x{dev['device_id']:02X}, Version=0x{dev['version']:02X}")
        network.set_baud_rate(125000)  # Upgrade to 125kbps
    """

# 🔴 UNVERIFIED - Context manager support
def __enter__(self) -> 'LDCNNetwork':
    """Enable 'with' statement usage."""
    self.open()
    return self

def __exit__(self, exc_type, exc_val, exc_tb) -> None:
    """Cleanup on 'with' statement exit."""
    self.close()
```

### Constants

```python
# Protocol
HEADER = 0xAA
ADDRESS_UNADDRESSED = 0x00
ADDRESS_GROUP = 0xFF

# Commands (generic LDCN)
CMD_RESET_POS = 0x00
CMD_SET_ADDRESS = 0x01
CMD_DEFINE_STATUS = 0x02
CMD_READ_STATUS = 0x03
CMD_SET_BAUD = 0x0A
CMD_NOP = 0x0E
CMD_HARD_RESET = 0x0F

# Servo commands (for reference, used in LS231SE)
CMD_LOAD_TRAJECTORY = 0x04
CMD_START_MOTION = 0x05
CMD_LOAD_GAINS = 0x06
CMD_STOP_MOTOR = 0x07
CMD_CLEAR_BITS = 0x0B

# Device IDs (from READ_STATUS with status bit 5)
# These are hardware-reported device type identifiers
DEVICE_ID_LS231SE = 0x17      # LS-231SE Servo Drive (TBD - verify from hardware)
DEVICE_ID_SK2310G2 = 0x23     # SK-2310g2 I/O Controller (TBD - verify from hardware)
# ... add other device IDs as discovered

# Device ID to class mapping
DEVICE_CLASS_MAP = {
    DEVICE_ID_LS231SE: LS231SE,
    DEVICE_ID_SK2310G2: SK2310g2,
    # Default to generic LDCNDevice for unknown IDs
}

# Status bits for device discovery
STATUS_BIT_DEVICE_ID = 0x0020  # Bit 5: Device ID and version (2 bytes)

# Baud rates
BAUD_RATES = {
    9600: 0x81,
    19200: 0x3F,
    57600: 0x14,
    115200: 0x0A,
    125000: 0x27,
    312500: 0x0F,
    625000: 0x07,
    1250000: 0x03
}

# Timing (seconds)
DELAY_AFTER_COMMAND = 0.02
DELAY_AFTER_RESET = 2.0
DELAY_AFTER_ADDRESS = 0.3
DELAY_AFTER_BAUD_CHANGE = 0.5
```

### Usage Example

```python
# Manual step-by-step usage
network = LDCNNetwork('/dev/ttyUSB0')
network.open()  # Opens at 19200 baud

# Initialize network (at 19200 baud)
num_devices, device_info = network.initialize()
print(f"Found {num_devices} devices:")
for dev in device_info:
    print(f"  Address {dev['address']}: ID=0x{dev['device_id']:02X}, Version=0x{dev['version']:02X}")

# Optionally upgrade to higher speed
network.set_baud_rate(125000)  # Upgrade to 125kbps

# Access devices (objects created by initialize())
servo1 = network.devices[0]  # LS231SE at address 1
servo2 = network.devices[1]  # LS231SE at address 2
io_controller = network.devices[5]  # SK2310g2 at address 6

network.close()

# Context manager usage (recommended)
with LDCNNetwork('/dev/ttyUSB0') as network:
    # Initialize at 19200 baud
    num_devices, device_info = network.initialize()

    # Upgrade speed
    network.set_baud_rate(125000)

    # Work with devices
    servo = network.devices[0]
    servo.initialize()  # Initialize servo with default gains
    servo.move_to(position=10.0, velocity=100.0, accel=50.0, scale=2000.0)

# Granular control (manual steps)
with LDCNNetwork('/dev/ttyUSB0') as network:
    # Step by step initialization
    network.reset()  # Hard reset at 19200 baud
    num_found = network.address_devices()  # Assign addresses 1, 2, 3...
    device_info = network.discover_devices()  # Query device types
    responding = network.verify_devices(device_info)  # Verify communication
    network.create_device_objects(device_info)  # Create device objects

    # Upgrade speed
    network.set_baud_rate(125000)
```

---

## 2. LDCNDevice Base Class

**Purpose**: Abstract base class for all LDCN devices.

### Properties

```python
network: LDCNNetwork      # Reference to parent network
address: int              # Device address (1-127)
device_type: str          # Device type string (e.g., "LS-231SE", "SK-2310g2")
model_id: Optional[int]   # Device model ID from Define Status bit 5
```

### Methods

```python
# 🔴 UNVERIFIED - Initialization
def __init__(self, network: LDCNNetwork, address: int) -> None:
    """
    Initialize base device.

    Args:
        network: Parent LDCNNetwork object
        address: Device address (1-127)
    """

# 🔴 UNVERIFIED - Communication
def send_command(self, command: int, data: list[int] = []) -> bytes:
    """
    Send command to this device.

    Delegates to network.send_command() with this device's address.

    Args:
        command: LDCN command code
        data: Data bytes

    Returns:
        Response bytes from device
    """

# 🔴 UNVERIFIED - Generic LDCN commands
def nop(self) -> bytes:
    """Send NOP command, return status."""

def define_status(self, status_bits: int) -> None:
    """
    Configure status reporting (permanent).

    Args:
        status_bits: 16-bit status configuration
    """

def read_status(self) -> dict:
    """
    Read device status (abstract - implemented by subclasses).

    Returns:
        Device-specific status dictionary
    """

def reset_position(self) -> None:
    """Reset position counter to zero (if supported)."""

# 🔴 UNVERIFIED - String representation
def __repr__(self) -> str:
    """Return string representation."""
```

---

## 3. LS231SE Class (Servo Drive)

**Purpose**: Servo drive specific operations.

### Additional Properties

```python
position: Optional[int]        # Last known position (counts)
velocity: Optional[int]        # Last known velocity
status_byte: Optional[int]     # Last status byte
aux_status: Optional[int]      # Last auxiliary status
pos_error: Optional[int]       # Last position error

# Gains (after set_gains() call)
kp: Optional[int]              # Proportional gain
kd: Optional[int]              # Derivative gain
ki: Optional[int]              # Integral gain
```

### Methods

```python
# 🔴 UNVERIFIED - Initialization
def initialize(self,
               kp: int = 2, kd: int = 50, ki: int = 0,
               il: int = 40, ol: int = 255, cl: int = 0,
               el: int = 2000, sr: int = 20, db: int = 0) -> bool:
    """
    Complete 7-step servo initialization.

    Steps:
    1. Define status reporting (pos, vel, aux, pos_err)
    2. Set PID gains
    3. Load initial trajectory (position 0)
    4. Enable amplifier
    5. Reset position counter
    6. Clear sticky status bits
    7. Read and verify status

    Args:
        kp, kd, ki: PID gains
        il: Integration limit
        ol: Output limit
        cl: Current limit (0 = disabled)
        el: Position error limit (counts)
        sr: Servo rate divisor
        db: Deadband

    Returns:
        True if initialization successful
    """

# 🔴 UNVERIFIED - Status reading
def read_status(self) -> dict:
    """
    Read complete servo status.

    Returns:
        {
            'status': status_byte,
            'position': position_counts,
            'velocity': velocity,
            'aux_status': aux_byte,
            'pos_error': following_error,
            'flags': {
                'move_done': bool,
                'cksum_error': bool,
                'current_limit': bool,
                'power_on': bool,
                'pos_error': bool,
                # ... all status bits
            }
        }
    """

def read_position(self) -> dict:
    """
    Read position only (fast status read).

    Returns:
        {'position': position_counts, 'status': status_byte}
    """

def decode_status_flags(self, status_byte: int) -> dict[str, bool]:
    """
    Decode status byte into flag dictionary.

    Args:
        status_byte: Status byte from response

    Returns:
        Dictionary of flag names to boolean values
    """

def check_faults(self, status_byte: int) -> list[str]:
    """
    Check status byte for fault conditions.

    Args:
        status_byte: Status byte to check

    Returns:
        List of active fault names (e.g., ['cksum_error', 'pos_error'])
    """

# 🔴 UNVERIFIED - Configuration
def set_gains(self,
              kp: int, kd: int, ki: int,
              il: int, ol: int, cl: int,
              el: int, sr: int, db: int) -> None:
    """Set PID gains (LOAD_GAINS command)."""

def configure_status(self, status_bits: int) -> None:
    """Configure status reporting (wrapper for define_status)."""

# 🔴 UNVERIFIED - Motion control
def move_to(self, position: float, velocity: float, accel: float, scale: float = 1.0) -> None:
    """
    Command motion to absolute position.

    Args:
        position: Target position (physical units, e.g., mm)
        velocity: Velocity (physical units/sec)
        accel: Acceleration (physical units/sec²)
        scale: Counts per physical unit (e.g., 2000 counts/mm)
    """

def move_to_counts(self, position: int, velocity: int, accel: int) -> None:
    """
    Command motion to absolute position (raw counts).

    Args:
        position: Target position (encoder counts)
        velocity: Velocity (counts per servo tick)
        accel: Acceleration (counts per tick²)
    """

# 🔴 UNVERIFIED - Amplifier control
def enable(self) -> None:
    """Enable amplifier (STOP_MOTOR with AMP_ENABLE flag)."""

def disable(self) -> None:
    """Disable amplifier and position servo."""

# 🔴 UNVERIFIED - Fault management
def reset_position(self) -> None:
    """Reset position counter to zero."""

def clear_faults(self) -> None:
    """Clear sticky status bits (CLEAR_BITS command)."""
```

### Constants

```python
# Status bit masks
STATUS_MOVE_DONE = 0x01
STATUS_CKSUM_ERROR = 0x02
STATUS_CURRENT_LIMIT = 0x04
STATUS_POWER_ON = 0x08
STATUS_POS_ERROR = 0x10
STATUS_HOME_SOURCE = 0x20
STATUS_LIMIT2 = 0x40
STATUS_HOME_IN_PROG = 0x80

# Stop motor flags
STOP_ABRUPT = 0x01
STOP_SMOOTH = 0x02
MOTOR_OFF = 0x04
AMP_ENABLE = 0x10

# Status reporting bits
STATUS_BIT_POSITION = 0x0001
STATUS_BIT_AD_VALUE = 0x0002
STATUS_BIT_VELOCITY = 0x0004
STATUS_BIT_AUX = 0x0008
STATUS_BIT_HOME = 0x0010
STATUS_BIT_DEVICE_ID = 0x0020
STATUS_BIT_POS_ERROR = 0x0040
STATUS_BIT_PATH_COUNT = 0x0080
```

### Usage Example

```python
with LDCNNetwork('/dev/ttyUSB0') as network:
    network.initialize()

    servo = network.devices[0]  # LS231SE at address 1

    # Initialize servo with default gains
    servo.initialize()

    # Enable amplifier
    servo.enable()

    # Move to position
    servo.move_to(position=10.0, velocity=100.0, accel=50.0, scale=2000.0)

    # Monitor status
    while True:
        status = servo.read_status()
        if status['flags']['move_done']:
            break
        print(f"Position: {status['position']}")
        time.sleep(0.1)
```

---

## 4. SK2310g2 Class (I/O Controller)

**Purpose**: I/O controller operations. The SK-2310g2 is a **generic LDCN I/O controller device** used in this application as a **supervisory controller** with safety and spindle control functions.

**Device Type**: I/O Controller
**Model**: SK-2310g2
**Application Role**: Supervisory Safety and Spindle Controller

**Hardware Capabilities**:
- Dual mechanical relay power control
- Spindle control with spindle enable mechanical relay
- Dual line emergency stop monitoring
- Dual work zone "covers" contacts (guarded area monitoring)
- Dual safe zone sensor interface
- 3 analog inputs
- 1 analog output (CN6.11 is 0-10V spindle speed control)
- Digital I/O (16 inputs, 16 outputs)

**Note**: The specific mapping of digital I/O channels to physical functions (E-stop, covers, relays, etc.) is **application-specific** and depends on how the I/O controller is wired in the machine. The class provides generic I/O access methods, and application-specific constants should be defined for channel mappings.

### Additional Properties

```python
# Status and diagnostics
diagnostic_code: Optional[int]     # Last diagnostic code (LED display)
status_byte: Optional[int]         # Last status byte

# Power and safety
power_state: Optional[bool]        # Power button state (mechanical relay)
estop_state: Optional[bool]        # Emergency stop state (dual line)
cover_open: Optional[bool]         # Work zone cover state (guarded areas)
safe_zone: Optional[bool]          # Safe zone sensor state

# I/O states
digital_inputs: Optional[int]      # 16-bit digital input state
digital_outputs: Optional[int]     # 16-bit digital output state
analog_inputs: Optional[dict]      # Analog input values {channel: value}

# Spindle control
spindle_speed: Optional[float]     # Spindle speed setpoint (0-10V → RPM)
spindle_enable: Optional[bool]     # Spindle enable relay state
```

### Methods

```python
# 🔴 UNVERIFIED - Configuration
def configure(self) -> None:
    """
    Configure supervisor for full status reporting.

    Sends DEFINE_STATUS with 0xFFFF (all status data).
    """

# 🔴 UNVERIFIED - Status reading
def read_status(self) -> dict:
    """
    Read complete I/O controller status.

    Returns:
        {
            'status': status_byte,
            'diagnostic': diagnostic_code,

            # Safety systems
            'power_state': bool,
            'estop_state': bool,
            'cover_state': bool,
            'safe_zone_state': bool,

            # I/O states
            'digital_inputs': int (16-bit),
            'digital_outputs': int (16-bit),
            'analog_inputs': {channel: value},

            # Spindle
            'spindle_enable': bool,
            'spindle_speed': float (percent),
        }
    """

def read_diagnostic(self) -> int:
    """
    Read diagnostic code (LED display value).

    Returns:
        Diagnostic code (0x00-0xFF)
    """

def read_power_state(self) -> bool:
    """
    Read power button state from status bit 3.

    Returns:
        True if power ON, False if power OFF
    """

# 🔴 UNVERIFIED - Power and safety monitoring
def wait_for_power_button(self, timeout: Optional[float] = None,
                           poll_rate: float = 0.1) -> bool:
    """
    Wait for power button press detection.

    Continuously monitors power state until transition from OFF to ON.

    Args:
        timeout: Maximum wait time (None = infinite)
        poll_rate: Status polling rate (seconds)

    Returns:
        True if power button pressed, False if timeout
    """

def read_estop_state(self) -> bool:
    """
    Read emergency stop state (dual line monitoring).

    Returns:
        True if E-stop is OK, False if E-stop is active
    """

def read_cover_state(self) -> bool:
    """
    Read work zone cover state (guarded area contacts).

    Returns:
        True if covers closed (safe), False if any cover open
    """

def read_safe_zone_state(self) -> bool:
    """
    Read safe zone sensor state (dual sensor interface).

    Returns:
        True if safe zone clear, False if zone occupied
    """

# 🔴 UNVERIFIED - Digital I/O
def read_digital_inputs(self) -> int:
    """
    Read all digital input states.

    Returns:
        16-bit digital input value
    """

def set_digital_outputs(self, outputs: int) -> None:
    """
    Set all digital output states.

    Args:
        outputs: 16-bit digital output value
    """

def set_digital_output(self, channel: int, state: bool) -> None:
    """
    Set individual digital output.

    Args:
        channel: Output channel (0-15)
        state: True = ON, False = OFF
    """

def read_digital_input(self, channel: int) -> bool:
    """
    Read individual digital input.

    Args:
        channel: Input channel (0-15)

    Returns:
        True if input high, False if low
    """

# 🔴 UNVERIFIED - Analog I/O
def read_analog_inputs(self) -> dict[int, int]:
    """
    Read all analog input values (3 channels).

    Returns:
        Dictionary of {channel: value} pairs
    """

def read_analog_input(self, channel: int) -> int:
    """
    Read single analog input value.

    Args:
        channel: Analog input channel (0-2)

    Returns:
        Analog value (ADC counts or voltage, device-specific)
    """

def set_analog_output(self, voltage: float) -> None:
    """
    Set analog output voltage (CN6.11 spindle speed control).

    Args:
        voltage: Output voltage (0.0 - 10.0V)
    """

# 🔴 UNVERIFIED - Spindle control
def set_spindle_speed(self, speed_percent: float) -> None:
    """
    Set spindle speed via analog output.

    Args:
        speed_percent: Speed as percentage (0.0 - 100.0)
    """

def enable_spindle(self) -> None:
    """Enable spindle via mechanical relay."""

def disable_spindle(self) -> None:
    """Disable spindle via mechanical relay."""

def set_spindle_state(self, enable: bool, speed_percent: float = 0.0) -> None:
    """
    Set spindle enable and speed atomically.

    Args:
        enable: True to enable spindle, False to disable
        speed_percent: Speed as percentage (0.0 - 100.0)
    """

# 🔴 UNVERIFIED - Power control
def set_power_relay(self, relay: int, state: bool) -> None:
    """
    Control power relay (dual mechanical relays).

    Args:
        relay: Relay number (0 or 1)
        state: True = ON, False = OFF
    """
```

### Constants

```python
# Diagnostic codes (from SK-2310g2 manual)
DIAG_POWER_OFF = 0x04
DIAG_POWER_ON = 0x0C
DIAG_ESTOP = 0x0E
# ... other diagnostic codes from SK-2310g2 manual

# Analog channels
ANALOG_INPUT_0 = 0
ANALOG_INPUT_1 = 1
ANALOG_INPUT_2 = 2
ANALOG_OUTPUT_SPINDLE = 0  # CN6.11 (0-10V spindle speed)

# Power relays
POWER_RELAY_0 = 0
POWER_RELAY_1 = 1

# Digital I/O channel mappings (application-specific)
# These would be defined based on actual wiring in the machine
# Example:
# DI_ESTOP_LOOP_1 = 0
# DI_ESTOP_LOOP_2 = 1
# DI_COVER_1 = 2
# DI_COVER_2 = 3
# DI_SAFE_ZONE_1 = 4
# DI_SAFE_ZONE_2 = 5
# DO_SPINDLE_ENABLE = 0
# DO_POWER_RELAY = 1

# Spindle control
SPINDLE_VOLTAGE_MIN = 0.0   # Volts
SPINDLE_VOLTAGE_MAX = 10.0  # Volts
SPINDLE_SPEED_MIN = 0.0     # Percent
SPINDLE_SPEED_MAX = 100.0   # Percent
```

### Usage Example

```python
with LDCNNetwork('/dev/ttyUSB0') as network:
    network.initialize()

    io_controller = network.devices[5]  # SK2310g2 at address 6

    # Configure for full status
    io_controller.configure()

    # Wait for power button press
    print("Waiting for power button...")
    if io_controller.wait_for_power_button(timeout=30.0):
        print("Power button pressed!")
    else:
        print("Timeout waiting for power button")

    # Check safety systems
    if not io_controller.read_estop_state():
        print("ERROR: E-stop is active!")
        sys.exit(1)

    if not io_controller.read_cover_state():
        print("WARNING: Work zone cover open!")

    if not io_controller.read_safe_zone_state():
        print("WARNING: Safe zone occupied!")

    # Read diagnostic code
    diag = io_controller.read_diagnostic()
    print(f"Diagnostic: 0x{diag:02X}")

    # Read digital inputs
    inputs = io_controller.read_digital_inputs()
    print(f"Digital inputs: 0b{inputs:016b}")

    # Control spindle
    io_controller.set_spindle_state(enable=True, speed_percent=50.0)
    time.sleep(5)  # Run at 50% for 5 seconds
    io_controller.set_spindle_state(enable=False, speed_percent=0.0)

    # Read analog inputs
    analog_values = io_controller.read_analog_inputs()
    for channel, value in analog_values.items():
        print(f"Analog input {channel}: {value}")
```

---

## 5. LS773 Class (Network I/O Node)

**Purpose**: Generic I/O controller operations for the LS-773 Network I/O Node.

**Device Type**: Network I/O Controller
**Model**: LS-773
**Application Role**: General Purpose I/O Control

**Hardware Capabilities**:
- 10 general purpose digital inputs (with configurable pull-up/pull-down)
- 6 open collector outputs (1A max each)
- 1 solid-state relay output (0.5A max, OUTPUT 0/POWER)
- 3 analog inputs (8-bit, 0-5V/0-10V/0-20V/0-30V selectable)
- 32-bit counter/timer with prescaler (5.0 MHz clock)
- 20 KHz PWM mode for OUTPUT 1 and OUTPUT 2
- Device ID: 2, Version: 50

**Note**: The LS-773 is a **generic I/O controller** suitable for a wide range of applications. Unlike the SK-2310g2, it does not have application-specific safety features built-in. The class provides generic I/O access methods, and application-specific logic should be implemented by the user.

### Additional Properties

```python
# Status
status_byte: Optional[int]         # Last status byte
out_sh: Optional[bool]             # Output short circuit flag

# Digital I/O states
digital_inputs: Optional[int]      # 10-bit digital input state (0-9)
digital_outputs: Optional[int]     # 7-bit digital output state (0-6)

# Analog inputs
analog_inputs: Optional[dict]      # Analog input values {channel: value}
analog_range: dict[int, str]       # Voltage range per channel {0:'0-5V', 1:'0-10V', 2:'0-20V'}

# PWM outputs
pwm_values: Optional[dict]         # PWM duty cycles {1: value, 2: value}

# Counter/Timer
counter_enabled: Optional[bool]    # Counter/timer enabled state
counter_mode: Optional[str]        # 'timer' or 'counter'
counter_prescaler: Optional[int]   # Prescaler value (1, 2, 4, or 8)
counter_value: Optional[int]       # Current counter/timer value (32-bit)

# Synchronized capture
synch_inputs: Optional[int]        # Captured input states (Synch Input)
synch_counter: Optional[int]       # Captured counter value (Synch Input)
```

### Methods

```python
# 🔴 UNVERIFIED - Configuration
def configure(self, status_bits: int = 0x1F) -> None:
    """
    Configure I/O node for status reporting.

    Args:
        status_bits: Status reporting configuration (default: inputs + all analog + counter)
                     Bit 0: Input bits (2 bytes)
                     Bit 1: ANALOG IN 0 (1 byte)
                     Bit 2: ANALOG IN 1 (1 byte)
                     Bit 3: ANALOG IN 2 (1 byte)
                     Bit 4: Counter/timer value (4 bytes)
                     Bit 5: Device ID, version (2 bytes)
                     Bit 6: Synch input bits (2 bytes)
                     Bit 7: Synch counter value (4 bytes)
    """

# 🔴 UNVERIFIED - Status reading
def read_status(self) -> dict:
    """
    Read complete I/O controller status.

    Returns:
        {
            'status': status_byte,
            'out_sh': bool,                    # Output short circuit flag

            # Digital I/O
            'digital_inputs': int (10-bit),    # DIGITAL IN 0-9
            'digital_outputs': int (7-bit),    # OUTPUT 0-6

            # Analog inputs
            'analog_inputs': {channel: value}, # 0-255 per channel

            # Counter/Timer
            'counter_value': int (32-bit),

            # Synchronized capture (if Synch Input was called)
            'synch_inputs': int (10-bit),
            'synch_counter': int (32-bit),
        }
    """

def check_output_short(self) -> bool:
    """
    Check if any output is shorted to POWER(+).

    Returns:
        True if short detected, False if normal operation
    """

# 🔴 UNVERIFIED - Digital I/O
def read_digital_inputs(self) -> int:
    """
    Read all digital input states.

    Returns:
        10-bit digital input value (DIGITAL IN 0-9)
    """

def read_digital_input(self, channel: int) -> bool:
    """
    Read individual digital input.

    Args:
        channel: Input channel (0-9)

    Returns:
        True if input high, False if low
    """

def set_digital_outputs(self, outputs: int) -> None:
    """
    Set all digital output states immediately.

    Args:
        outputs: 7-bit digital output value (OUTPUT 0-6)
    """

def set_digital_output(self, channel: int, state: bool) -> None:
    """
    Set individual digital output.

    Args:
        channel: Output channel (0-6)
        state: True = ON, False = OFF
    """

def get_digital_outputs(self) -> int:
    """
    Get current output states (cached value).

    Returns:
        7-bit digital output value
    """

# 🔴 UNVERIFIED - Analog I/O
def read_analog_inputs(self) -> dict[int, int]:
    """
    Read all analog input values (3 channels).

    Returns:
        Dictionary of {channel: value} pairs (0-255 per channel)
    """

def read_analog_input(self, channel: int) -> int:
    """
    Read single analog input value.

    Args:
        channel: Analog input channel (0-2)

    Returns:
        Analog value (0-255)
    """

def analog_to_voltage(self, value: int, channel: int) -> float:
    """
    Convert analog reading to voltage based on configured range.

    Args:
        value: Raw ADC value (0-255)
        channel: Analog input channel (0-2)

    Returns:
        Voltage value based on configured range
    """

def voltage_to_analog(self, voltage: float, channel: int) -> int:
    """
    Convert voltage to analog value based on configured range.

    Args:
        voltage: Voltage value
        channel: Analog input channel (0-2)

    Returns:
        Raw ADC value (0-255)
    """

def set_analog_range(self, channel: int, range_str: str) -> None:
    """
    Set the expected voltage range for an analog input (for conversion).
    Note: Physical range is set via DIP switches on the hardware.

    Args:
        channel: Analog input channel (0-2)
        range_str: Range string ('0-5V', '0-10V', '0-20V', '0-30V')
    """

# 🔴 UNVERIFIED - PWM Control
def set_pwm(self, pwm1: int, pwm2: int) -> None:
    """
    Set PWM duty cycle for OUTPUT 1 and OUTPUT 2.

    Args:
        pwm1: PWM 1 value (255=OFF, 128=50%, 0=FULLY ON)
        pwm2: PWM 2 value (255=OFF, 128=50%, 0=FULLY ON)
    """

def set_pwm_percent(self, pwm1_percent: float, pwm2_percent: float) -> None:
    """
    Set PWM duty cycle as percentage.

    Args:
        pwm1_percent: PWM 1 percentage (0.0-100.0)
        pwm2_percent: PWM 2 percentage (0.0-100.0)
    """

def enable_pwm(self) -> None:
    """
    Enable PWM mode for OUTPUT 1 and OUTPUT 2.

    Sets output bits 1 and 2 to 1 to activate PWM mode.
    """

def disable_pwm(self) -> None:
    """
    Disable PWM mode for OUTPUT 1 and OUTPUT 2.

    Sets output bits 1 and 2 to 0 to deactivate PWM mode.
    """

# 🔴 UNVERIFIED - Counter/Timer
def configure_counter(self, mode: str = 'counter', prescaler: int = 1, enabled: bool = True) -> None:
    """
    Configure the counter/timer.

    Args:
        mode: 'timer' (5.0 MHz internal clock) or 'counter' (external input)
        prescaler: Prescaler value (1, 2, 4, or 8)
        enabled: True to enable, False to disable
    """

def read_counter(self) -> int:
    """
    Read current counter/timer value.

    Returns:
        32-bit counter/timer value
    """

def reset_counter(self) -> None:
    """
    Reset counter/timer to zero.

    Note: LS-773 counter cannot be directly reset. To reset, disable and re-enable.
    """

# 🔴 UNVERIFIED - Synchronized I/O
def synch_input(self) -> None:
    """
    Capture current input states and counter value atomically.

    Captured values are stored internally and can be read via read_status()
    with status bits 6 and 7 enabled.
    """

def read_synch_inputs(self) -> dict:
    """
    Read previously captured synchronized inputs.

    Returns:
        {
            'inputs': int (10-bit),
            'counter': int (32-bit)
        }
    """

def set_synch_outputs(self, outputs: int, pwm1: int, pwm2: int) -> None:
    """
    Stage output values for synchronized application.

    Args:
        outputs: 7-bit digital output value
        pwm1: PWM 1 value (255-0)
        pwm2: PWM 2 value (255-0)
    """

def apply_synch_outputs(self) -> None:
    """
    Apply previously staged output values atomically.
    """

# 🔴 UNVERIFIED - Initialization
def initialize(self) -> bool:
    """
    Complete I/O node initialization.

    Steps:
    1. Configure status reporting (inputs, analog, counter)
    2. Initialize outputs to known state (all OFF)
    3. Disable counter/timer
    4. Set PWM to OFF
    5. Read and verify status

    Returns:
        True if initialization successful
    """
```

### Constants

```python
# Device identification
DEVICE_ID = 2        # LS-773 device ID
VERSION = 50         # Typical firmware version

# Output channels
OUTPUT_0_POWER = 0   # Solid-state relay, 0.5A
OUTPUT_1_PWM = 1     # Open collector, 1A, PWM capable
OUTPUT_2_PWM = 2     # Open collector, 1A, PWM capable
OUTPUT_3 = 3         # Open collector, 1A
OUTPUT_4 = 4         # Open collector, 1A
OUTPUT_5 = 5         # Open collector, 1A
OUTPUT_6 = 6         # Open collector, 1A

# Input channels
DIGITAL_IN_0 = 0
DIGITAL_IN_1 = 1
DIGITAL_IN_2 = 2
DIGITAL_IN_3 = 3
DIGITAL_IN_4 = 4
DIGITAL_IN_5 = 5
DIGITAL_IN_6 = 6
DIGITAL_IN_7 = 7
DIGITAL_IN_8 = 8
DIGITAL_IN_9_COUNT = 9  # Also counter input

# Analog channels
ANALOG_IN_0 = 0
ANALOG_IN_1 = 1
ANALOG_IN_2 = 2

# Analog voltage ranges
ANALOG_RANGE_0_5V = '0-5V'
ANALOG_RANGE_0_10V = '0-10V'
ANALOG_RANGE_0_20V = '0-20V'
ANALOG_RANGE_0_30V = '0-30V'

# Status reporting bits (for Define Status / Read Status)
STATUS_BIT_INPUTS = 0x01       # Digital input bytes (2 bytes)
STATUS_BIT_ANALOG_0 = 0x02     # Analog input 0 (1 byte)
STATUS_BIT_ANALOG_1 = 0x04     # Analog input 1 (1 byte)
STATUS_BIT_ANALOG_2 = 0x08     # Analog input 2 (1 byte)
STATUS_BIT_COUNTER = 0x10      # Counter/timer value (4 bytes)
STATUS_BIT_DEVICE_ID = 0x20    # Device ID and version (2 bytes)
STATUS_BIT_SYNCH_INPUTS = 0x40 # Synch input bits (2 bytes)
STATUS_BIT_SYNCH_COUNTER = 0x80 # Synch counter value (4 bytes)

# Counter/Timer modes
COUNTER_MODE_TIMER = 'timer'     # 5.0 MHz internal clock
COUNTER_MODE_COUNTER = 'counter' # External input on DIGITAL IN 9

# Counter/Timer prescaler values
PRESCALER_1 = 1
PRESCALER_2 = 2
PRESCALER_4 = 4
PRESCALER_8 = 8

# Timer specifications
TIMER_CLOCK_HZ = 5_000_000      # 5.0 MHz
TIMER_RESOLUTION_NS = 200       # 200 ns per count
TIMER_MAX_COUNT = 0xFFFFFFFF    # 32-bit counter

# PWM specifications
PWM_FREQUENCY_HZ = 20_000       # 20 KHz
PWM_OFF = 255                    # 0% duty cycle
PWM_50_PERCENT = 128             # 50% duty cycle
PWM_ON = 0                       # 100% duty cycle

# Status byte flags
STATUS_CKSUM_ERROR = 0x02        # Bit 1: Checksum error

# Input status byte 1 flags
INPUT_STATUS_OUT_SH = 0x02       # Bit 1: Output short circuit
```

### Usage Example

```python
with LDCNNetwork('/dev/ttyUSB0') as network:
    network.initialize()

    io_node = network.devices[0]  # LS773 at address 1

    # Initialize I/O node
    io_node.initialize()

    # Configure analog input ranges (for conversion)
    io_node.set_analog_range(0, '0-10V')
    io_node.set_analog_range(1, '0-5V')
    io_node.set_analog_range(2, '0-20V')

    # Read digital inputs
    inputs = io_node.read_digital_inputs()
    print(f"Digital inputs: 0b{inputs:010b}")

    # Set digital outputs
    io_node.set_digital_outputs(0b00001010)  # OUTPUT 1 and OUTPUT 3 ON

    # Read analog inputs with voltage conversion
    analog_values = io_node.read_analog_inputs()
    for channel, raw_value in analog_values.items():
        voltage = io_node.analog_to_voltage(raw_value, channel)
        print(f"Analog input {channel}: {raw_value} (raw) = {voltage:.2f}V")

    # Configure and use PWM
    io_node.enable_pwm()
    io_node.set_pwm_percent(75.0, 50.0)  # OUTPUT 1 at 75%, OUTPUT 2 at 50%

    # Configure counter mode
    io_node.configure_counter(mode='counter', prescaler=1, enabled=True)

    # Read counter value
    count = io_node.read_counter()
    print(f"Counter value: {count}")

    # Synchronized capture
    io_node.synch_input()  # Capture inputs and counter atomically
    synch_data = io_node.read_synch_inputs()
    print(f"Synch inputs: {synch_data['inputs']}, counter: {synch_data['counter']}")

    # Synchronized output (for multi-node coordination)
    io_node.set_synch_outputs(outputs=0b00000101, pwm1=128, pwm2=200)
    # ... set synch outputs on other nodes ...
    io_node.apply_synch_outputs()  # All nodes change simultaneously
```

---

## 6. Exception Classes

```python
class LDCNError(Exception):
    """Base exception for LDCN errors."""

class LDCNTimeoutError(LDCNError):
    """No response from device."""

class LDCNChecksumError(LDCNError):
    """Response checksum mismatch."""

class LDCNDetectionError(LDCNError):
    """Auto-detection failed."""

class LDCNInitializationError(LDCNError):
    """Device initialization failed."""
```

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

## Implementation Status

### Completed (🟡 IMPLEMENTED - Awaiting Hardware Verification)

1. ✅ **Class Design Documentation** - Complete and detailed
2. ✅ **Module Structure** - Created modular architecture:
   - `pyldcn/network.py` (1252 lines) - LDCNNetwork and LDCNDevice base classes
   - `pyldcn/devices/servo.py` (595 lines) - LS231SE servo drive class
   - `pyldcn/devices/io.py` (1328 lines) - SK2310g2 and LS773 I/O controller classes
3. ✅ **Base Protocol Methods** - All implemented:
   - `send_command()` - Core LDCN protocol communication
   - `auto_detect_baud()` - Baud rate detection
   - `set_baud_rate()` - Network baud rate changes
   - `reset()` - Hard reset at all baud rates
   - `address_devices()` - Sequential device addressing
   - `discover_devices()` - Device type discovery
   - `verify_devices()` - Communication verification
4. ✅ **Adaptive Initialization** - Enhanced beyond original design:
   - `validate_devices()` - Fast validation without state changes
   - `soft_initialize()` - State-preserving discovery
   - `initialize()` - Multi-mode adaptive initialization (VALIDATE, SOFT, READDRESS, FULL, AUTO)
5. ✅ **Device-Specific Subclasses** - Fully implemented:
   - **LS231SE** - Servo drive with 7-step initialization, motion control, status reading
   - **SK2310g2** - Supervisory controller with safety monitoring, diagnostic decoding, I/O control
   - **LS773** - Generic I/O node with PWM, counter/timer, analog I/O
6. ✅ **High-Level Control** - Command layer implemented:
   - `pyldcn/command/axis.py` - Axis configuration and motion control
   - `pyldcn/command/machine.py` - Multi-axis machine coordination

### In Progress

7. 🔴 **Hardware Testing** - Ready for testing:
   - All methods marked 🔴 UNVERIFIED
   - Awaiting hardware access for verification
   - Need to compare behavior with original utilities byte-for-byte

### Next Steps

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

### Implementation Notes

**Current Status:** All designed functionality has been implemented in code (~3175 lines). The module is feature-complete and ready for hardware testing. The implementation includes enhancements beyond the original design, particularly the adaptive initialization system which provides state preservation and faster reconnection capabilities.

