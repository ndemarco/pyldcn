# Testing Guide - At the Machine

You're at the machine and ready to test! Here's your quick start guide.

## Goal

Get one axis to move. That's it. Simple and focused.

## What You Have

1. **SK-2310g2 I/O Controller** - Handles power, e-stop, digital I/O
2. **LS-231SE Servo Drive(s)** - Controls axis motors
3. **pyldcn library** - Python code to talk to these devices

## Test Scripts Available

### 1. **simple_motion_test.py** - START HERE
```bash
cd /home/user/pyldcn
python3 examples/simple_motion_test.py --port /dev/ttyUSB0 --servo 1
```

This script:
- ✓ Initializes the LDCN network
- ✓ Finds and configures the SK-2310g2 (if present)
- ✓ Checks power state
- ✓ Initializes a servo drive
- ✓ Moves the axis 1000 counts
- ✓ Monitors position in real-time

**Options:**
```bash
--port /dev/ttyUSB0    # Serial port
--servo 1              # Servo address (1-15)
--baud 125000          # Baud rate
--distance 1000        # Distance in encoder counts
--velocity 1000        # Speed in counts/sec
--accel 5000           # Acceleration in counts/sec^2
```

### 2. **power_on_workflow.py** - If power is off
```bash
python3 examples/power_on_workflow.py --port /dev/ttyUSB0
```

This script:
- Initializes network
- Finds SK-2310g2
- Requests you to press the power button
- Waits for power ON
- Verifies system is ready

### 3. **test_network.py** - Diagnostic testing
```bash
python3 tests/test_network.py --port /dev/ttyUSB0 --test-servo 1
```

This runs a full test suite:
- Network initialization
- Servo initialization
- I/O controller status
- Position reading

## Step-by-Step First Test

### Step 1: Find your serial port
```bash
ls /dev/ttyUSB*
# or
ls /dev/ttyACM*
```

### Step 2: Check permissions
```bash
sudo chmod 666 /dev/ttyUSB0
# or add yourself to dialout group:
sudo usermod -a -G dialout $USER
# (then log out and back in)
```

### Step 3: Run the simple test
```bash
cd /home/user/pyldcn
python3 examples/simple_motion_test.py --port /dev/ttyUSB0 --servo 1 --distance 500
```

**Start with a small distance (500 counts) for safety!**

## What to Watch For

### ✓ Good Signs
- "Found X devices" - Network initialization succeeded
- "✓ Servo initialization complete" - Servo is ready
- "MOVING" messages during movement - Axis is moving
- "Movement complete" - Success!

### ⚠ Warning Signs
- "Power is OFF" - You may need to press power button
- "Active faults detected" - Servo has a problem
- "Servo initialization completed with warnings" - May still work
- "Position error: X counts" - Small errors are OK, large ones are not

### ✗ Error Signs
- "No devices discovered" - Check serial port, connections
- "Servo at address X not found" - Wrong address or not connected
- "Timeout" errors - Communication problem
- Movement doesn't happen - Check power, e-stop, enable signals

## Troubleshooting

### No devices found
1. Check serial port: `ls /dev/ttyUSB*`
2. Check permissions: `ls -l /dev/ttyUSB0`
3. Check physical connections
4. Try different baud rates: `--baud 19200`

### Power is OFF
1. Run `power_on_workflow.py` first
2. Press the physical power button
3. Check e-stop is not engaged
4. Verify 24V power supply is on

### Servo won't initialize
1. Check servo address (DIP switches or previous addressing)
2. Try a different servo: `--servo 2`
3. Look for diagnostic codes in output
4. Check servo power (LED indicators)

### Axis doesn't move
1. Check if servo is enabled (status output)
2. Verify no faults are active
3. Check limit switches aren't engaged
4. Ensure axis is mechanically free to move
5. Try even smaller movement: `--distance 100`

## Understanding Encoder Counts

Encoder counts are raw position units. To convert to physical units:

```python
# Example: 2000 counts per revolution, 5mm pitch lead screw
counts_per_mm = 2000 / 5  # = 400 counts/mm

# To move 10mm:
distance_counts = 10 * 400  # = 4000 counts
```

**Start small!** 1000 counts might be:
- ~2.5mm on a 2000 counts/rev, 5mm pitch screw
- Less on higher resolution encoders

## What We're Learning

Each test teaches us something:

1. **Network init** - Can we talk to the devices?
2. **Power state** - Is the system ready?
3. **Servo init** - Can we configure the servo?
4. **Position reading** - Can we monitor position?
5. **Movement** - Can we command motion?

Once these work, we can build more sophisticated control (LinuxCNC integration).

## Quick Reference - Common Commands

```bash
# Test with default settings (safe small move)
python3 examples/simple_motion_test.py

# Test specific servo
python3 examples/simple_motion_test.py --servo 2

# Very small test move
python3 examples/simple_motion_test.py --distance 100

# Faster movement
python3 examples/simple_motion_test.py --distance 1000 --velocity 5000

# Different serial port
python3 examples/simple_motion_test.py --port /dev/ttyACM0

# Check network and devices only (no movement)
python3 tests/test_network.py --port /dev/ttyUSB0
```

## Ready?

You're at the machine. Let's start with:

```bash
python3 examples/simple_motion_test.py --port /dev/ttyUSB0 --distance 500
```

Report what you see, and we'll iterate from there!

## Safety Reminders

- ✓ E-stop easily accessible
- ✓ Clear path for axis movement
- ✓ Start with small movements
- ✓ Power can be cycled if needed
- ✓ You can Ctrl+C to stop at any time
