# Axis Scaling (LS-231SE with 10k-line encoder, 5 mm/rev screw)

- Encoder: 10,000 lines, quadrature → 4 edges/line → **40,000 counts/rev**.
- Leadscrew: 5 mm/rev → **8,000 counts/mm** (40,000 ÷ 5).
- Velocity (counts/tick) ↔ mm/s: `mm_per_s = (counts_per_tick * 8000) / servo_rate_counts_per_s`; invert as needed for commands.
- Acceleration (counts/tick²) ↔ mm/s²: scale by 8,000 counts/mm and servo tick period squared.
- Use 8,000 counts/mm as the scale in motion helpers to work in engineering units.***
