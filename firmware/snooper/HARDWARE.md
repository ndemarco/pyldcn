# Hardware Setup Guide

## Bill of Materials

| Component | Quantity | Notes |
|-----------|----------|-------|
| Raspberry Pi Pico | 1 | RP2040-based board |
| Waveshare 2-Channel RS-485 HAT | 1 | Or equivalent dual-channel transceiver |
| USB Micro-B Cable | 1 | For power and data |
| Jumper Wires | 4-6 | If not using HAT directly |
| Terminal Blocks | 2 | For RS-485 bus connection (if not on HAT) |

## Waveshare RS-485 HAT Pinout

The Waveshare 2-Channel RS-485 HAT provides two independent RS-485 transceivers.

### Default Configuration

**Channel 1 (UART0):**
- RXD: GPIO 0 (Pico GP0) - **NOT USED in passive mode**
- TXD: GPIO 1 (Pico GP1) - **Used for RX in our implementation**

**Channel 2 (UART1):**
- RXD: GPIO 4 (Pico GP4) - **NOT USED in passive mode**
- TXD: GPIO 5 (Pico GP5) - **Used for RX in our implementation**

**Note:** We use the TX pins as RX because we're only receiving. The UART is configured with `tx=None` to prevent transmission.

### Modification for Passive Monitoring

Since we're only receiving, we can use either the RX or TX pins of the UART. The code uses GP1 and GP5.

If your HAT has different pin assignments, update these constants in `code.py`:
```python
UART_TX_CHANNEL_RX_PIN = board.GP1  # Adjust as needed
UART_RX_CHANNEL_RX_PIN = board.GP5  # Adjust as needed
```

## Physical Connections

### Bus Tapping

You need to tap into the RS-485 bus at two points:

```
                    ┌─────────────────┐
                    │   Controller    │
                    └────────┬────────┘
                             │ RS-485
                             │
                    ┌────────┴────────┐
                    │   TAP POINT 1   │ ← Monitor Controller→Device
                    │   (Channel 1)   │
                    └────────┬────────┘
                             │
                    ┌────────┴────────┐
                    │     Device      │
                    └────────┬────────┘
                             │
                    ┌────────┴────────┐
                    │   TAP POINT 2   │ ← Monitor Device→Controller
                    │   (Channel 2)   │
                    └─────────────────┘
```

### Wiring Diagram

```
RS-485 Bus (Controller Side)     Waveshare HAT
  A (Data+) ──────────────────→  CH1 A
  B (Data-) ──────────────────→  CH1 B
  GND ─────────────────────────→  GND

RS-485 Bus (Device Side)         Waveshare HAT
  A (Data+) ──────────────────→  CH2 A
  B (Data-) ──────────────────→  CH2 B
  GND ─────────────────────────→  GND

Waveshare HAT                     Raspberry Pi Pico
  UART0 TX (GPIO1) ─────────────  GP1 (UART0 RX)
  UART1 TX (GPIO5) ─────────────  GP5 (UART1 RX)
  GND ──────────────────────────  GND
  5V ───────────────────────────  VSYS (if powering from HAT)
```

### Alternative: Without HAT

If you're using standalone RS-485 transceivers:

**Channel 1 (Controller→Device):**
```
RS-485 Transceiver 1
  A/B → Controller side of bus
  RO (Receiver Output) → Pico GP1
  /RE (Receiver Enable) → GND (always enabled)
  DE (Driver Enable) → GND (always disabled)
  DI (Driver Input) → Not connected
  VCC → 3.3V
  GND → GND
```

**Channel 2 (Device→Controller):**
```
RS-485 Transceiver 2
  A/B → Device side of bus
  RO (Receiver Output) → Pico GP5
  /RE (Receiver Enable) → GND (always enabled)
  DE (Driver Enable) → GND (always disabled)
  DI (Driver Input) → Not connected
  VCC → 3.3V
  GND → GND
```

## Bus Termination

⚠️ **Important:** The snooper should NOT add termination to the bus.

The LDCN bus should already have proper termination resistors (typically 120Ω) at each end. Adding the snooper should not affect termination.

**Recommended setup:**
- Use high-impedance RS-485 transceivers
- Disable termination on the HAT/transceivers
- Monitor only - do not modify bus characteristics

## Power Considerations

### Option 1: USB Powered (Recommended)
- Connect Pico to monitoring computer via USB
- Pros: Simple, provides power and data connection
- Cons: Requires USB cable to computer

### Option 2: External 5V
- Power Pico from VSYS pin (external 5V supply)
- USB still needed for data
- Pros: Can place snooper remotely
- Cons: Requires separate power supply

### Option 3: Bus Powered
- Some RS-485 networks provide power on the bus
- Check voltage levels before connecting!
- Pros: Single cable to snooper
- Cons: Depends on bus power availability

**Power consumption:**
- Pico: ~25-30mA typical
- RS-485 transceivers: ~5mA each
- **Total: ~40mA @ 5V**

## Mechanical Assembly

### With Waveshare HAT

1. Connect HAT directly to Pico GPIO headers
2. Secure with standoffs if needed
3. Connect RS-485 terminals to screw terminals on HAT
4. Use strain relief for cables

### Without HAT

1. Use breadboard or prototype board
2. Mount Pico and RS-485 transceivers
3. Wire according to schematic above
4. Use terminal blocks for RS-485 connections
5. Add strain relief and enclosure

## Recommended Enclosure

- Small plastic project box (approx. 80x60x30mm)
- Cutouts for:
  - USB cable
  - RS-485 cable entry
- Standoffs to mount Pico
- Ventilation not critical (low power)

## Testing Hardware Setup

### Step 1: Visual Inspection
- Check all connections
- Verify no shorts between pins
- Confirm polarity of A/B lines

### Step 2: Power On Test
```
1. Connect USB to computer
2. Pico LED should light up
3. Check Device Manager/lsusb for USB serial device
4. Open serial terminal at 115200 baud
5. Should see startup messages
```

### Step 3: Signal Test
```
1. Use multimeter to check voltages:
   - 3.3V on Pico 3V3(OUT) pin
   - ~2.5V (idle) on RS-485 A/B lines

2. Use oscilloscope:
   - Verify RS-485 differential signal on A/B
   - Check GP1 and GP5 for UART signals
```

### Step 4: Loopback Test
```
1. Disconnect from actual bus
2. Use USB-RS485 adapter to generate test frames
3. Send frames at 9600 baud
4. Verify snooper captures and outputs frames
```

## Troubleshooting

### No USB device detected
- Check USB cable (use data cable, not charge-only)
- Try different USB port
- Verify Pico is powered (LED on)
- Re-flash CircuitPython if needed

### No frames captured
- Verify RS-485 A/B polarity
- Check voltage on A/B lines (should be ~2.5V idle)
- Use oscilloscope to verify bus activity
- Confirm baud rate setting (9600 for LDCN)

### Incorrect/garbled frames
- Check RS-485 polarity (swap A and B)
- Verify ground connection
- Check for noise/interference
- Confirm baud rate matches bus

### High invalid frame count
- Check signal quality with oscilloscope
- Verify termination on main bus (not snooper)
- Check cable quality and length
- Reduce electrical noise sources

## Safety Notes

⚠️ **Do not hot-plug RS-485 connections** - Power down bus before connecting

⚠️ **ESD protection** - Use anti-static precautions when handling Pico

⚠️ **Voltage levels** - Verify bus voltage before connecting (should be ±12V max for RS-485)

⚠️ **Isolation** - Consider using isolated RS-485 transceivers for industrial environments

⚠️ **No transmission** - Firmware disables TX to prevent bus disruption, but verify before deployment

## Recommended RS-485 Transceivers

For DIY builds without HAT:

| Part Number | Features | Notes |
|-------------|----------|-------|
| MAX485 | Basic, non-isolated | Common, cheap |
| MAX3485 | 3.3V compatible | Better for Pico |
| ADM2682E | Isolated, 2.5kV | Industrial use |
| LTC2865 | Integrated termination | Simplified design |

For passive monitoring, any RS-485 transceiver will work. Isolation is recommended for industrial environments.
