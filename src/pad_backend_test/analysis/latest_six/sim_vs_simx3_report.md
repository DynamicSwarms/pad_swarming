# New Sim vs Simx3

Only the six latest runs: three Sim wall-time runs and three Simx3 simulation-time runs, cf0–cf2, with identical 68-second sequences. Earlier incomplete/interrupted October 7 runs are excluded. Exact filenames are in `sim_vs_simx3_manifest.json`.

| Phase | Sim vz (m/s) | Simx3 vz (m/s) | Sim dz (m) | Simx3 dz (m) |
|---|---:|---:|---:|---:|
| up | 0.1500 | 0.1500 | 0.7361 | 0.7377 |
| down | -0.1500 | -0.1499 | -0.7221 | -0.7257 |
| up_with_settle | 0.1500 | 0.1499 | 0.7391 | 0.7377 |
| down_with_settle | -0.1500 | -0.1516 | -0.7421 | -0.7377 |
| up_forward | 0.1061 | 0.1062 | 0.5226 | 0.5216 |

Velocity metrics use a linear fit over the last second of each phase; displacement is interpolated between actual command boundaries. Values are means over three aircraft. Plots show each run, using actual ROS time for velocity differentiation and matching phase boundaries to nominal sequence time for the full-sequence overlay. The centered 0.4-second difference smooths transitions; it cannot resolve exact latency or high-frequency oscillation. These recordings alone do not establish the exact compiled controller version.

Run `python3 plot_six.py` to reproduce (NumPy and Matplotlib required). No original recordings are modified.
