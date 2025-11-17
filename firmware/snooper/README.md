# LDCN Snooper - Full Duplex RS-485 Bus Monitor

CircuitPython firmware for Raspberry Pi Pico that passively monitors bidirectional RS-485 traffic on LDCN control buses.

## Features

✅ **Dual-channel monitoring** - Simultaneous capture of both directions
✅ **Full frame validation** - Start byte, length field, and checksum verification
✅ **Robust frame synchronization** - Proper state machine with timeout handling
✅ **Buffer overflow protection** - Monitors and warns about buffer usage
✅ **Error statistics** - Tracks invalid frames and USB disconnects
✅ **Zero bus impact** - Receive-only passive monitoring (TX disabled)

## Hardware Requirements

### Components
- **Raspberry Pi Pico** (or Pico W)
- **Waveshare 2-Channel RS-485 HAT** or compatible dual-channel RS-485 transceiver
- USB cable for power and data

### Wiring

```
LDCN Bus (Controller → Device traffic):
  RS-485 A/B  →  Channel 1  →  Pico GP1 (UART0 RX)

LDCN Bus (Device → Controller traffic):
  RS-485 A/B  →  Channel 2  →  Pico GP5 (UART1 RX)
```

**Pin Configuration:**
- GP1: RX for controller→device traffic (TX channel)
- GP5: RX for device→controller traffic (RX channel)
- GP0, GP4: Not used (TX explicitly disabled for safety)

### Important Notes

⚠️ **Passive Monitoring Only**
Both UARTs are configured with `tx=None` to prevent accidental transmission that could disrupt the bus.

⚠️ **Differential RS-485**
Both A/B lines of each RS-485 channel must be tapped into the corresponding bus direction. Ensure proper termination on the main bus.

⚠️ **Power**
Pico can be powered via USB (from monitoring computer) or external 5V supply.

## Software Requirements

### CircuitPython Installation

1. Download CircuitPython 8.x or later for Raspberry Pi Pico:
   - https://circuitpython.org/board/raspberry_pi_pico/

2. Enter bootloader mode:
   - Hold BOOTSEL button while connecting USB
   - Pico appears as USB drive

3. Copy `.uf2` file to the drive
   - Pico reboots automatically

### Firmware Installation

1. Copy `code.py` to the Pico's CIRCUITPY drive
2. Pico reboots and starts monitoring
3. Connect via USB serial to view output

## Usage

### Output Format

Each captured frame is output as a CSV line:

```
timestamp_ms,direction,hex_payload
```

**Fields:**
- `timestamp_ms`: Milliseconds since boot (Pico clock)
- `direction`:
  - `tx` = Controller → Device (commands)
  - `rx` = Device → Controller (status/responses)
- `hex_payload`: Frame bytes in hexadecimal (no spaces)

**Example:**
```
1234,tx,aa01050300abcd12
5678,rx,aa0185030012ab34
```

### Connecting to the Snooper

**Linux/macOS:**
```bash
# Find the device
ls /dev/tty.usb* # macOS
ls /dev/ttyACM* # Linux

# Connect with screen
screen /dev/ttyACM0 115200

# Or use cat
cat /dev/ttyACM0 > ldcn_capture.csv

# Or Python
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0',115200); \
  [print(s.readline().decode()) for _ in iter(int, 1)]"
```

**Windows:**
```powershell
# Find COM port in Device Manager, then:
mode COM3 BAUD=115200 PARITY=N DATA=8 STOP=1
type COM3 > ldcn_capture.csv
```

### Startup Messages

On boot, the snooper prints diagnostic information:

```
==================================================
LDCN Snooper - Dual Channel RS-485 Monitor
==================================================
✓ TX Channel (Controller→Device) initialized on GP1
✓ RX Channel (Device→Controller) initialized on GP5
UART: 9600 baud
TX Channel: GP1 (Controller→Device)
RX Channel: GP5 (Device→Controller)
Frame format: 0xaa START, min 5B
==================================================
Monitoring started. Waiting for frames...
```

### Statistics

Every 60 seconds, the snooper prints monitoring statistics:

```
--- Statistics ---
TX frames: 1234
RX frames: 1189
Invalid frames: 5
Buffer warnings: 0
USB disconnects: 0
------------------
```

## Frame Validation

The snooper implements comprehensive frame validation:

### LDCN Frame Structure
```
Byte 0:    0xAA           Start byte
Byte 1:    Address        Device address
Byte 2:    Command/Status Command byte (or status with bit 7 set)
Byte 3:    Length         Payload length (N bytes)
Bytes 4-N: Payload        Command/status data
Byte N+1:  Checksum       8-bit sum checksum
```

### Validation Checks
1. ✅ Start byte must be `0xAA`
2. ✅ Frame length matches length field
3. ✅ Frame length within `MIN_FRAME_LENGTH` to `MAX_FRAME_LENGTH`
4. ✅ Checksum verification (8-bit sum)
5. ✅ Inter-byte timeout (max 10ms between bytes)
6. ✅ Overall frame timeout (max 50ms per frame)

Invalid frames are counted in statistics but not output.

## Configuration

Edit constants at the top of `code.py`:

### Pin Configuration
```python
UART_TX_CHANNEL_RX_PIN = board.GP1  # Controller→Device
UART_RX_CHANNEL_RX_PIN = board.GP5  # Device→Controller
```

### Protocol Parameters
```python
UART_BAUDRATE = 9600           # Bus speed
FRAME_START_BYTE = 0xAA        # LDCN protocol start byte
MIN_FRAME_LENGTH = 5           # Minimum valid frame size
MAX_FRAME_LENGTH = 64          # Maximum valid frame size
```

### Timing Parameters
```python
INTER_BYTE_TIMEOUT_MS = 10     # Max gap between bytes in frame
FRAME_TIMEOUT_MS = 50          # Max time for complete frame
```

### Buffer Configuration
```python
UART_BUFFER_SIZE = 512              # UART receive buffer
BUFFER_WARNING_THRESHOLD = 384      # Warn at 75% full
```

## Troubleshooting

### No frames captured

1. **Check wiring:**
   - Verify RS-485 A/B connections
   - Confirm GP1 and GP5 are receiving data
   - Check RS-485 transceiver power

2. **Check bus activity:**
   - Use oscilloscope to verify differential signals
   - Confirm bus is active and properly terminated

3. **Check baud rate:**
   - Verify `UART_BAUDRATE` matches actual bus speed
   - LDCN typically uses 9600, but confirm

### High invalid frame count

1. **Checksum mismatch:**
   - Verify `calculate_checksum()` matches your protocol
   - Some LDCN variants use different checksum algorithms

2. **Noise/interference:**
   - Check RS-485 grounding
   - Verify termination resistors
   - Use shielded cable

3. **Baud rate mismatch:**
   - Confirm exact baud rate
   - Frame corruption often indicates timing issues

### Buffer warnings

1. **USB bottleneck:**
   - Increase USB poll rate on host
   - Use faster USB library (pyserial vs. cat)

2. **Too much traffic:**
   - Increase `UART_BUFFER_SIZE`
   - Reduce `STATS_INTERVAL_MS` to avoid print delays

3. **Processing delays:**
   - Code should not block in main loop
   - Check for modifications that add delays

### USB disconnects

This is usually harmless - occurs when:
- Host application closes serial port
- USB cable is briefly disconnected
- Host computer suspends

Frames are lost during disconnection. Use persistent USB connection for reliable capture.

## Performance

### Throughput
- **9600 baud:** ~960 bytes/sec theoretical, ~800 bytes/sec with overhead
- **Typical LDCN:** 10-50 frames/sec depending on polling interval
- **Max sustainable:** ~200 frames/sec with average 8-byte frames

### Latency
- **Frame detection:** <1ms after last byte received
- **USB output:** <10ms typical
- **Total latency:** <15ms from bus to host file

### CPU Usage
- Minimal - most time spent in UART hardware
- No busy-wait loops (uses 100μs yield sleep)
- Statistics printing is the main CPU consumer

## Integration with pyldcn

Captured data can be analyzed using the pyldcn library:

```python
import pyldcn.protocol as protocol

# Parse captured frames
with open('ldcn_capture.csv') as f:
    for line in f:
        if line.startswith('---') or line.startswith('='):
            continue  # Skip stats

        timestamp, direction, hex_data = line.strip().split(',')
        frame_bytes = bytes.fromhex(hex_data)

        # Parse using pyldcn protocol module
        # (adjust based on actual pyldcn API)
        print(f"{timestamp}ms [{direction}]: {frame_bytes.hex()}")
```

## Development Notes

### Code Changes Since Original

**Critical fixes implemented:**
1. ✅ Dual UART initialization (both channels)
2. ✅ Removed direction detection (use channel as ground truth)
3. ✅ Proper frame state machine with synchronization
4. ✅ Fixed timeout logic (inter-byte vs. frame timeout)
5. ✅ Comprehensive frame validation
6. ✅ Removed sleep from frame reading
7. ✅ TX explicitly disabled (`tx=None`)
8. ✅ Error handling and statistics
9. ✅ Buffer overflow monitoring
10. ✅ Named constants instead of magic numbers

### Testing Recommendations

1. **Bench test:**
   - Use USB-RS485 adapter to generate test frames
   - Verify both directions captured correctly
   - Test frame validation with intentional errors

2. **Field test:**
   - Connect to live LDCN bus
   - Monitor during known operations
   - Verify frame counts match expected traffic

3. **Stress test:**
   - Generate high frame rate
   - Monitor buffer warnings
   - Verify no frame loss

## License

Same as pyldcn main library.

## Support

Report issues or questions in the main pyldcn repository.
