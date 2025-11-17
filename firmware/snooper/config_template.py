"""
Configuration template for LDCN Snooper

Copy this file to config.py and customize for your specific hardware setup.
The main code.py will import settings from config.py if it exists, otherwise
it will use the default values in code.py.
"""

# ============================================================================
# UART PIN CONFIGURATION
# ============================================================================

# Channel 1: Controller → Device (TX direction)
# This channel monitors commands from the controller to devices
UART_TX_CHANNEL_RX_PIN = "GP1"  # Use board.GP1 in code

# Channel 2: Device → Controller (RX direction)
# This channel monitors responses from devices to controller
UART_RX_CHANNEL_RX_PIN = "GP5"  # Use board.GP5 in code

# Alternative pin configurations for different HATs:
# - Waveshare 2-CH RS485: GP1, GP5 (default)
# - Custom wiring: Adjust to your actual connections
# - Available UART pairs on Pico: (GP0,GP1), (GP4,GP5), (GP8,GP9), (GP12,GP13), (GP16,GP17)

# ============================================================================
# UART PARAMETERS
# ============================================================================

# Baud rate must match your LDCN bus speed
UART_BAUDRATE = 9600  # Standard LDCN

# Alternative baud rates (uncomment if needed):
# UART_BAUDRATE = 19200  # Fast LDCN
# UART_BAUDRATE = 38400  # Custom
# UART_BAUDRATE = 115200  # High speed

# UART buffer size - increase if you see buffer warnings
UART_BUFFER_SIZE = 512  # bytes

# Buffer usage warning threshold (bytes)
# Warnings are printed when buffer exceeds this level
BUFFER_WARNING_THRESHOLD = 384  # 75% of buffer size

# ============================================================================
# LDCN PROTOCOL CONFIGURATION
# ============================================================================

# Frame start byte - first byte of every LDCN frame
FRAME_START_BYTE = 0xAA  # Standard LDCN

# Frame length constraints
MIN_FRAME_LENGTH = 5   # Start + Addr + Cmd + Len + Checksum (minimum)
MAX_FRAME_LENGTH = 64  # Maximum reasonable frame size

# Set MAX_FRAME_LENGTH based on your LDCN variant:
# - Standard LDCN: 32-64 bytes typical
# - Extended LDCN: Up to 255 bytes
# - Custom: Adjust based on protocol documentation

# ============================================================================
# TIMING CONFIGURATION
# ============================================================================

# Inter-byte timeout: Maximum time between consecutive bytes in a frame
# At 9600 baud, byte time ≈ 1.04ms, so 10ms allows for significant jitter
INTER_BYTE_TIMEOUT_MS = 10  # milliseconds

# Frame timeout: Maximum total time to receive a complete frame
# Should be longer than (MAX_FRAME_LENGTH * byte_time) + some margin
FRAME_TIMEOUT_MS = 50  # milliseconds

# Adjust for different baud rates:
# - 9600 baud: 10ms inter-byte, 50ms frame (default)
# - 19200 baud: 5ms inter-byte, 25ms frame
# - 38400 baud: 3ms inter-byte, 15ms frame
# - 115200 baud: 1ms inter-byte, 5ms frame

# ============================================================================
# STATISTICS AND REPORTING
# ============================================================================

# How often to print statistics (milliseconds)
STATS_INTERVAL_MS = 60000  # 60 seconds

# Set to 0 to disable periodic statistics
# STATS_INTERVAL_MS = 0

# Alternative intervals:
# STATS_INTERVAL_MS = 30000   # 30 seconds
# STATS_INTERVAL_MS = 300000  # 5 minutes

# ============================================================================
# CHECKSUM CONFIGURATION
# ============================================================================

# Checksum algorithm selection
# Different LDCN implementations may use different checksums
CHECKSUM_TYPE = "sum8"  # Default: 8-bit sum

# Supported checksum types:
# - "sum8": Simple 8-bit sum (most common)
# - "xor8": 8-bit XOR
# - "crc8": CRC-8 (specify polynomial)
# - "none": No checksum validation (not recommended)

# CRC-8 configuration (only used if CHECKSUM_TYPE = "crc8")
CRC8_POLYNOMIAL = 0x07  # CRC-8-CCITT
CRC8_INIT = 0x00
CRC8_XOROUT = 0x00

# ============================================================================
# DEBUG OPTIONS
# ============================================================================

# Enable verbose debugging output
DEBUG = False  # Set to True for detailed diagnostics

# Print every frame to console (in addition to USB serial)
ECHO_FRAMES = False  # Useful for debugging, but slows down capture

# Print raw bytes for frames that fail validation
PRINT_INVALID_FRAMES = False  # Helpful for debugging protocol issues

# ============================================================================
# USB SERIAL CONFIGURATION
# ============================================================================

# USB serial output format
# Options: "csv", "json", "hex"
OUTPUT_FORMAT = "csv"  # Default: timestamp,direction,hex_payload

# CSV format: timestamp_ms,direction,hex_payload
# JSON format: {"timestamp":1234,"direction":"tx","payload":"aa01..."}
# HEX format: timestamp_ms [TX] aa 01 05 03 ...

# ============================================================================
# HARDWARE-SPECIFIC SETTINGS
# ============================================================================

# Waveshare 2-Channel RS-485 HAT
# No additional configuration needed - uses default pins

# Custom RS-485 transceivers
# Enable receiver (if using /RE pin):
# ENABLE_RECEIVER_PIN = "GP2"  # Set LOW to enable receiver
# Set to None if /RE is hardwired to GND

# Disable driver (if using DE pin):
# DISABLE_DRIVER_PIN = "GP3"  # Set LOW to disable driver
# Set to None if DE is hardwired to GND

# ============================================================================
# EXAMPLE CONFIGURATIONS
# ============================================================================

# Example 1: Standard Waveshare HAT, 9600 baud LDCN
"""
UART_TX_CHANNEL_RX_PIN = "GP1"
UART_RX_CHANNEL_RX_PIN = "GP5"
UART_BAUDRATE = 9600
FRAME_START_BYTE = 0xAA
"""

# Example 2: High-speed LDCN, custom pins
"""
UART_TX_CHANNEL_RX_PIN = "GP9"
UART_RX_CHANNEL_RX_PIN = "GP13"
UART_BAUDRATE = 38400
INTER_BYTE_TIMEOUT_MS = 3
FRAME_TIMEOUT_MS = 15
"""

# Example 3: Extended frames with CRC
"""
UART_BAUDRATE = 19200
MAX_FRAME_LENGTH = 128
CHECKSUM_TYPE = "crc8"
CRC8_POLYNOMIAL = 0x07
"""

# ============================================================================
# NOTES
# ============================================================================

# 1. Pin numbering uses GP numbers (e.g., "GP1" not "GPIO1")
# 2. Baud rate must exactly match bus speed (no auto-detection)
# 3. Frame start byte must match protocol (0xAA is LDCN standard)
# 4. Timeouts should be tuned based on bus characteristics and baud rate
# 5. Buffer size should be increased for high-traffic busses
# 6. Statistics interval affects CPU usage (more frequent = higher CPU)
