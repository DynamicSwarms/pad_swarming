# Three simulation versions

Versions 1 and 2 each contain three successful wall-time runs (cf0–cf2); version 3 currently contains one successful run (cf2). All use identical 68-second sequences. Earlier incomplete/interrupted October 7 runs are excluded. Exact filenames are in `three_versions_manifest.json`.

| Phase | V1 vz (m/s) | V2 vz (m/s) | V3 vz (m/s) | V1 dz (m) | V2 dz (m) | V3 dz (m) |
|---|---:|---:|---:|---:|---:|---:|
| up | 0.0750 | 0.1687 | 0.1500 | 0.3635 | 0.7401 | 0.7261 |
| down | -0.0750 | -0.1086 | -0.1500 | -0.3525 | -0.3441 | -0.7021 |
| up_with_settle | 0.0750 | 0.1665 | 0.1500 | 0.3655 | 0.7027 | 0.7291 |
| down_with_settle | -0.0750 | -0.1136 | -0.1500 | -0.3670 | -0.4311 | -0.7350 |
| up_forward | 0.0530 | 0.1246 | 0.1061 | 0.2603 | 0.5126 | 0.5113 |

Velocity metrics use a linear fit over the last second of each phase; displacement is interpolated between actual command boundaries. Values are means over the available runs; V3 has no replication yet. Plots show each run, using actual ROS time for velocity differentiation and matching phase boundaries to nominal sequence time for the full-sequence overlay. The centered 0.4-second difference smooths transitions; it cannot resolve exact latency or high-frequency oscillation. These recordings alone do not establish the exact compiled controller version.

Run `python3 compare_versions.py` to reproduce (NumPy and Matplotlib required). No original recordings are modified.
