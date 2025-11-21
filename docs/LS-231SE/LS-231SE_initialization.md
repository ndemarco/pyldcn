This document presumes the LS-231SE device has been configured for *LDCN Single loop mode*. The device mode is modified using the LDCN utility, which uses undocumented commands.


## Persistent parameters
The LS-231SE stores certain parameters in non-volatile storage. The parameters, commands, and data structures are undocumented by Logosol Inc. Known NVRAM parameters can be viewed and modified by the LDCN Utility.

When configured in any LDCN device mode, these stored parameters are of no direct use, as all gains and related servo parameters are set to defaults at power on or hard_reset. The servo tuning parameters may be stored as backups to be recalled later. The storage and recall commands are undocumented

### Device Mode
Sets the LS-231SE device to one of 8 modes using the LDCN utility.

The mode bit pattern indicates the mode. Note LDCN data is LSB.

D	C	B	A	Mode description
0	0	0	0	LDCN single loop
X	0	0	1	LDCN dual looop
X	0	1	0	Analog input single/dual loop
X	0	1	1	Analog input with direction invert input
X	1	0	0	Enable positive / enable negative analog input
X	1	0	1	Quadrature encoder
X	1	1	0	Step & dir
X	1	1	1	Step positive / step negative
The selected mode defines status and auxillary bit meanings.

It is good practice to confirm the mode upon initializing the device.

#### Get current mode
Read output bits to determine the mode. Out.12 = A, Out.13 = B, Out.14 = C, Out.15 = D.

## Servo Initialization Sequence

Complete 7-step initialization sequence for servo drives:

1. Define status reporting
2. Set PID gains (KP, KD, KI, IL, OL, CL, EL, SR, DB)
3. Load initial trajectory (position 0, minimal acceleration)
4. Enable amplifier and close servo loop
5. Reset position counter
6. Clear sticky status bits
7. Read and verify status