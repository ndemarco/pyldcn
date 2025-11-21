# LS-231SE Initialization (Working Sequence)

- **Prereqs**: Supervisor (SK-2310g2) must report `read_power_state() == True`; mode bits must be `0000` (LDCN single loop).
- **Persistent status**: Define status to include pos/vel/aux/pos_err (mask `0x0001|0x0004|0x0008|0x0040` or as needed).
- **Gains**: Send `LOAD_GAINS` with desired kp/kd/ki/limits (don’t rely on NVRAM defaults). Defaults used here cache kp=2, kd=50, ki=0 in `ServoState`.
- **Clear/reset/enable** (order matters): `CLEAR_BITS` → `RESET_POSITION` → `STOP_MOTOR` with AMP_ENABLE to turn on the amplifier and hold position.
- **Verify**: Read status; expect `power=True`, `servo_on=True`, `pos_error=False`, limits True (healthy NC switches), condition=None.***
