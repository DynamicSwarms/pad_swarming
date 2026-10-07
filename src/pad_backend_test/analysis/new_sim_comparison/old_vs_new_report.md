# Old vs new simulation

Three successful wall-time runs per group, cf0–cf2, with identical 68-second sequences. Earlier incomplete/interrupted October 7 runs are excluded. Exact filenames are in `old_vs_new_manifest.json`.

| Phase | Old vz (m/s) | New vz (m/s) | Old dz (m) | New dz (m) |
|---|---:|---:|---:|---:|
| up | 0.0750 | 0.1687 | 0.3635 | 0.7401 |
| down | -0.0750 | -0.1086 | -0.3525 | -0.3441 |
| up_with_settle | 0.0750 | 0.1665 | 0.3655 | 0.7027 |
| down_with_settle | -0.0750 | -0.1136 | -0.3670 | -0.4311 |
| up_forward | 0.0530 | 0.1246 | 0.2603 | 0.5126 |

Velocity metrics use a linear fit over the last second of each phase; displacement is interpolated between actual command boundaries. Values are means over three aircraft. Plots show each run, using actual ROS time for velocity differentiation and matching phase boundaries to nominal sequence time for the full-sequence overlay. The centered 0.4-second difference smooths transitions; it cannot resolve exact latency or high-frequency oscillation. These recordings alone do not establish the exact compiled controller version.

Run `python3 compare_new_sim.py` to reproduce (NumPy and Matplotlib required). No original recordings are modified.

Observed: horizontal responses nearly overlap. New vertical response has an approximately +0.026 m/s drift during zero vertical commands, upward overshoot, and slow descent reversal. The half-speed plateau is removed, but vertical tracking is not yet correct.
