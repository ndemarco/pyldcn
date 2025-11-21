# Homing Progress (LS-231SE @ addr 1)

- Prereqs satisfied: supervisor power-on via `power_on()` / `read_power_state() == True`; servo init succeeds (power=True, servo_on=True, pos_error=False), mode check enforced (0000 LDCN single loop).
- Homing configured: Limit1 + Index, abrupt stop (control byte 0x19), velocity mode reverse at 1 mm/s, accel 500 mm/s² (8,000 counts/mm scale), start_now=1.
- Results: Motion did not occur; sampled status shows position stuck at 0, power/servo_on True, condition None; `home_in_progress` never cleared within 10 s.
- Likely causes to investigate: command accepted but velocity not applied (direction/sign?), limit/home/homeSEL gating (both limits read True by default), need to trigger motion with explicit start or different profile, or safety interlock still inhibiting motion.
- Next steps: verify motion with a small absolute move; log status/flags while issuing a velocity command; confirm HOMESEL mapping and limit states; retry homing after confirming the axis will move under current gains/enable state.***
