"""
LDCN Snooper - CircuitPython firmware for Raspberry Pi Pico
Passively monitors full-duplex RS-485 bus and forwards frames via USB CDC serial.

Hardware:
- Raspberry Pi Pico
- Waveshare 2-Channel RS-485 HAT (or compatible)
- Channel 1 (GP0/GP1): Controller → Device traffic
- Channel 2 (GP4/GP5): Device → Controller traffic

Output format: timestamp_ms,direction,hex_payload
- timestamp_ms: milliseconds since boot
- direction: "tx" (controller to device) or "rx" (device to controller)
- hex_payload: raw frame bytes in hex
"""

import board
import busio
import supervisor
import time
import usb_cdc

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================

# RS-485 UART configuration
UART_BAUDRATE = 9600  # Standard LDCN bus speed

# Channel 1: Controller → Device (TX direction)
UART_TX_CHANNEL_RX_PIN = board.GP1  # Receive controller transmissions

# Channel 2: Device → Controller (RX direction)
UART_RX_CHANNEL_RX_PIN = board.GP5  # Receive device responses

# LDCN Protocol Constants
FRAME_START_BYTE = 0xAA
MIN_FRAME_LENGTH = 5  # Start + Addr + Cmd + Len + Checksum (minimum)
MAX_FRAME_LENGTH = 64  # Reasonable maximum for LDCN frames
STATUS_RESPONSE_FLAG = 0x80  # Bit 7 set in status byte indicates response

# Frame timing
INTER_BYTE_TIMEOUT_MS = 10  # Max time between bytes in a frame
FRAME_TIMEOUT_MS = 50  # Max time for complete frame

# Buffer configuration
UART_BUFFER_SIZE = 512
BUFFER_WARNING_THRESHOLD = 384  # Warn at 75% full

# ============================================================================
# GLOBAL STATE
# ============================================================================

class Statistics:
    """Track monitoring statistics."""
    def __init__(self):
        self.tx_frames = 0
        self.rx_frames = 0
        self.invalid_frames = 0
        self.buffer_warnings = 0
        self.usb_disconnects = 0

    def reset(self):
        """Reset all statistics."""
        self.__init__()

stats = Statistics()

# ============================================================================
# INITIALIZATION
# ============================================================================

# Initialize USB CDC serial for host communication
usb_serial = usb_cdc.console

def init_uart(rx_pin, label):
    """
    Initialize a UART for passive monitoring (RX only).

    Args:
        rx_pin: CircuitPython pin for RX
        label: Descriptive label for error messages

    Returns:
        Initialized UART object or None on failure
    """
    try:
        uart = busio.UART(
            tx=None,  # No TX - passive monitoring only
            rx=rx_pin,
            baudrate=UART_BAUDRATE,
            bits=8,
            parity=None,
            stop=1,
            timeout=0,  # Non-blocking
            receiver_buffer_size=UART_BUFFER_SIZE
        )
        print(f"✓ {label} initialized on {rx_pin}")
        return uart
    except Exception as e:
        print(f"✗ Failed to initialize {label}: {e}")
        return None

# Initialize both UART channels
uart_tx_channel = init_uart(UART_TX_CHANNEL_RX_PIN, "TX Channel (Controller→Device)")
uart_rx_channel = init_uart(UART_RX_CHANNEL_RX_PIN, "RX Channel (Device→Controller)")

if not uart_tx_channel or not uart_rx_channel:
    print("ERROR: Failed to initialize UARTs. Check pin configuration.")
    raise RuntimeError("UART initialization failed")

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def get_timestamp_ms():
    """Get milliseconds since boot."""
    return int(supervisor.ticks_ms())

def calculate_checksum(frame):
    """
    Calculate LDCN checksum for frame validation.

    LDCN typically uses simple sum checksum.
    Adjust this based on actual protocol specification.

    Args:
        frame: bytes object containing frame data

    Returns:
        Calculated checksum byte
    """
    # Simple 8-bit sum checksum (adjust if protocol differs)
    checksum = sum(frame[:-1]) & 0xFF
    return checksum

def validate_frame(frame):
    """
    Validate frame structure and checksum.

    Args:
        frame: bytes object to validate

    Returns:
        True if valid, False otherwise
    """
    # Check minimum length
    if len(frame) < MIN_FRAME_LENGTH:
        return False

    # Check maximum length
    if len(frame) > MAX_FRAME_LENGTH:
        return False

    # Verify start byte
    if frame[0] != FRAME_START_BYTE:
        return False

    # Verify length field matches actual length
    if len(frame) >= 4:
        declared_length = frame[3]
        # Total frame = Start + Addr + Cmd + Len + Payload + Checksum
        expected_length = 4 + declared_length + 1
        if len(frame) != expected_length:
            return False

    # Verify checksum
    expected_checksum = frame[-1]
    calculated_checksum = calculate_checksum(frame)
    if expected_checksum != calculated_checksum:
        return False

    return True

def check_buffer_usage(uart, channel_name):
    """
    Monitor UART buffer usage and warn if getting full.

    Args:
        uart: UART object to check
        channel_name: Name for warning messages
    """
    if uart.in_waiting > BUFFER_WARNING_THRESHOLD:
        stats.buffer_warnings += 1
        # Only warn periodically to avoid spam
        if stats.buffer_warnings % 10 == 1:
            print(f"⚠ {channel_name} buffer high: {uart.in_waiting}/{UART_BUFFER_SIZE}")

# ============================================================================
# FRAME READING
# ============================================================================

def read_frame(uart, channel_name):
    """
    Read a complete, validated frame from the UART.

    Implements proper frame synchronization:
    1. Search for start byte
    2. Read header to determine frame length
    3. Read complete frame based on length field
    4. Validate checksum

    Args:
        uart: UART object to read from
        channel_name: Channel identifier for debugging

    Returns:
        bytes object containing valid frame, or None if no complete frame available
    """
    if uart.in_waiting == 0:
        return None

    # Check buffer usage
    check_buffer_usage(uart, channel_name)

    frame = bytearray()
    last_byte_time = get_timestamp_ms()
    frame_start_time = last_byte_time

    # State machine for frame reading
    STATE_SEARCH_START = 0
    STATE_READ_HEADER = 1
    STATE_READ_PAYLOAD = 2

    state = STATE_SEARCH_START
    expected_length = 0

    while True:
        current_time = get_timestamp_ms()

        # Check for inter-byte timeout
        if len(frame) > 0 and (current_time - last_byte_time) > INTER_BYTE_TIMEOUT_MS:
            # Incomplete frame - discard and return None
            if len(frame) > 0:
                stats.invalid_frames += 1
            return None

        # Check for overall frame timeout
        if (current_time - frame_start_time) > FRAME_TIMEOUT_MS:
            if len(frame) > 0:
                stats.invalid_frames += 1
            return None

        # Check if data available
        if uart.in_waiting == 0:
            # No more data available
            if state == STATE_READ_PAYLOAD and len(frame) == expected_length:
                # Frame complete - validate and return
                if validate_frame(bytes(frame)):
                    return bytes(frame)
                else:
                    stats.invalid_frames += 1
                    return None
            # Waiting for more data
            continue

        # Read next byte
        byte_data = uart.read(1)
        if not byte_data:
            continue

        byte_val = byte_data[0]
        last_byte_time = current_time

        if state == STATE_SEARCH_START:
            # Looking for start byte
            if byte_val == FRAME_START_BYTE:
                frame = bytearray([byte_val])
                state = STATE_READ_HEADER
                frame_start_time = current_time

        elif state == STATE_READ_HEADER:
            # Reading header: Addr + Cmd + Len
            frame.append(byte_val)

            if len(frame) == 4:  # Start + Addr + Cmd + Len
                length_field = frame[3]
                # Expected total length = 4 (header) + length_field (payload) + 1 (checksum)
                expected_length = 4 + length_field + 1

                # Sanity check
                if expected_length < MIN_FRAME_LENGTH or expected_length > MAX_FRAME_LENGTH:
                    # Invalid length - restart search
                    state = STATE_SEARCH_START
                    frame = bytearray()
                    stats.invalid_frames += 1
                else:
                    state = STATE_READ_PAYLOAD

        elif state == STATE_READ_PAYLOAD:
            # Reading payload + checksum
            frame.append(byte_val)

            if len(frame) == expected_length:
                # Frame complete - validate and return
                if validate_frame(bytes(frame)):
                    return bytes(frame)
                else:
                    stats.invalid_frames += 1
                    # Start searching for next frame
                    state = STATE_SEARCH_START
                    frame = bytearray()

        # Safety check - prevent infinite loop
        if len(frame) > MAX_FRAME_LENGTH:
            stats.invalid_frames += 1
            return None

# ============================================================================
# OUTPUT
# ============================================================================

def send_to_host(timestamp_ms, direction, payload):
    """
    Send formatted record to host via USB CDC.

    Format: timestamp_ms,direction,hex_payload

    Args:
        timestamp_ms: Milliseconds since boot
        direction: "tx" or "rx"
        payload: bytes object containing frame data
    """
    hex_payload = payload.hex()
    line = f"{timestamp_ms},{direction},{hex_payload}\n"

    if usb_serial and usb_serial.connected:
        try:
            usb_serial.write(line.encode('ascii'))
        except OSError as e:
            stats.usb_disconnects += 1
            # Don't spam console with disconnect messages
            if stats.usb_disconnects % 100 == 1:
                print(f"⚠ USB write failed: {e}")

def print_statistics():
    """Print monitoring statistics."""
    print(f"\n--- Statistics ---")
    print(f"TX frames: {stats.tx_frames}")
    print(f"RX frames: {stats.rx_frames}")
    print(f"Invalid frames: {stats.invalid_frames}")
    print(f"Buffer warnings: {stats.buffer_warnings}")
    print(f"USB disconnects: {stats.usb_disconnects}")
    print(f"------------------\n")

# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    """Main monitoring loop with dual-channel support."""
    print("\n" + "="*50)
    print("LDCN Snooper - Dual Channel RS-485 Monitor")
    print("="*50)
    print(f"UART: {UART_BAUDRATE} baud")
    print(f"TX Channel: GP{UART_TX_CHANNEL_RX_PIN} (Controller→Device)")
    print(f"RX Channel: GP{UART_RX_CHANNEL_RX_PIN} (Device→Controller)")
    print(f"Frame format: {FRAME_START_BYTE:#04x} START, min {MIN_FRAME_LENGTH}B")
    print("="*50)
    print("Monitoring started. Waiting for frames...\n")

    last_stats_time = get_timestamp_ms()
    STATS_INTERVAL_MS = 60000  # Print stats every 60 seconds

    while True:
        current_time = get_timestamp_ms()

        # Monitor TX channel (Controller → Device)
        tx_frame = read_frame(uart_tx_channel, "TX")
        if tx_frame:
            timestamp = get_timestamp_ms()
            send_to_host(timestamp, "tx", tx_frame)
            stats.tx_frames += 1

        # Monitor RX channel (Device → Controller)
        rx_frame = read_frame(uart_rx_channel, "RX")
        if rx_frame:
            timestamp = get_timestamp_ms()
            send_to_host(timestamp, "rx", rx_frame)
            stats.rx_frames += 1

        # Periodically print statistics
        if (current_time - last_stats_time) > STATS_INTERVAL_MS:
            print_statistics()
            last_stats_time = current_time

        # Minimal delay - only yield to other tasks
        # No sleep in frame reading loop!
        time.sleep(0.0001)  # 100μs - just yield CPU

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user")
        print_statistics()
    except Exception as e:
        print(f"\n\nFATAL ERROR: {e}")
        import traceback
        traceback.print_exception(type(e), e, e.__traceback__)
        raise
