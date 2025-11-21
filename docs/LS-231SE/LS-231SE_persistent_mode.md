LS-231SE NVRAM storage, commands, and structure are undocumented by Logosol Inc. Most NVRAM details are set by the LDCN Utility.

## Mode
Set the LS-231SE device to one of 8 modes using the LDCN utility.

The mode bit pattern indicates the mode. Note LDCN data is LSB.


| D | C | B | A | Mode description |
| - | - | - | - | -----------------|
| 0 | 0 | 0 | 0 | LDCN single loop |
| X | 0 | 0 | 1 | LDCN dual looop |
| X | 0 | 1 | 0 | Analog input single/dual loop |
| X | 0 | 1 | 1 | Analog input with direction invert input |
| X | 1 | 0 | 0 | Enable positive / enable negative analog input |
| X | 1 | 0 | 1 | Quadrature encoder |
| X | 1 | 1 | 0 | Step & dir |
| X | 1 | 1 | 1 | Step positive / step negative |

The selected mode defines status and auxillary bit meanings.

### Get current mode

Read output bits to determine the mode. Out.12 = A, Out.13 = B, Out.14 = C, Out.15 = D.

## Tuning parameters

Servo tuning parameters are stored in NVRAM.

For LDCN modes, these stored parameters are of no direct use, as all gains and related servo parameters are set to defaults at power on or `hard_reset`. The servo tuning parameters may be stored as backups to be recalled later. The storage and recall commands are undocumented.