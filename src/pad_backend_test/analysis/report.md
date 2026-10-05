# Backend velocity comparison

Analyzed nine successful recordings of the identical 19-step, 68-second sequence: three aircraft (cf0, cf1, cf2) per mode. Older 6-/15-step runs and one failed connection attempt were excluded. All nine selected recordings report successful return/deactivation.

**Label caveat:** every recording says `backend: simulation`. Sim-time is explicitly identified by `use_sim_time: true`. The latest batch (2026-10-05 02:06:13, 02:07:41, 02:09:11 Berlin time) is provisionally treated as SITL based on the stated three-mode experiment and its position after the SITL repair. Confirm this mapping before using the SITL comparison as a definitive result. Original files are unchanged. `runs.csv` lists exact input names and assigned groups.

## Main results

Values below are means over the three aircraft. Steady speed is estimated by a least-squares world-position slope over the final second of each movement, projected onto the commanded direction. Vertical values are magnitudes.

| Measurement | Sim | Sim-time | SITL (provisional) |
|---|---:|---:|---:|
| Sequence wall duration | 68.15 s | 22.67 s | 68.17 s |
| Sequence ROS duration | 68.15 s | 68.00 s | 68.17 s |
| ROS/wall speed ratio | 1.00× | 3.00× | 1.00× |
| Commands per ROS second | 19.96 | 20.00 | 19.95 |
| Info samples per ROS second | 9.99 | 10.00 | 10.00 |
| Forward steady speed (target 0.15 m/s) | 0.1500 | 0.1506 | 0.1531 |
| Up steady speed (target 0.15 m/s) | 0.0750 | 0.0747 | 0.1532 |
| Immediate-down steady speed (target 0.15 m/s) | 0.0750 | 0.0752 | 0.1532 |
| Settled-down steady speed (target 0.15 m/s) | 0.0750 | 0.0749 | 0.1520 |

The accelerated simulation reproduces the main steady responses at approximately three times wall speed. It also reproduces the vertical tracking defect: vertical velocity is roughly half the requested value. This is not caused by choosing the wrong analysis time base. Horizontal speeds stay near the requested 0.15 m/s in both modes.

All selected files have valid world-pose flags throughout. The average across runs of each run's largest info interval is 0.1002 s for sim, 0.1100 s for sim-time, and 0.1004 s for the latest batch. These are reception/logging intervals, not independent estimates of source sampling latency.

## Vertical mismatch and source evidence

In `/home/winni/ds/dynamic_swarms/src/crazyflie_simulation/src/crazyflie_simulation/src/simulation/controller.cpp:77`, the velocity controller computes `vz_command = desired_vz - current_vz` with gain 1. In `simulation.cpp:65`, the result is assigned directly to the model's vertical velocity. Ignoring integration details, the recurrence is `v_next = target - v`, whose fixed point/time-average is `target / 2`. This is a strong source-level explanation consistent with the recordings, rather than evidence of a time-scaling error. The runtime binary has not been instrumented to establish causality, and no controller code was changed during analysis.

The same effect appears in up-forward: the simulator follows the horizontal component while its vertical component is approximately halved. Thus a diagonal speed scalar alone would hide an important direction error. The component plots show it directly.

## Settling and reversal

Net position change during horizontal pauses (includes braking; not total path length):

| Pause after | Sim | Sim-time | SITL (provisional) |
|---|---:|---:|---:|
| Forward | 1.69 cm | 1.68 cm | 4.69 cm |
| Right | 1.59 cm | 1.65 cm | 4.52 cm |
| Forward-right | 1.39 cm | 1.67 cm | 4.41 cm |
| Immediate backward→forward reversal | 2.09 cm | 1.68 cm | 5.49 cm |

The latest batch exhibits overshoot and slower braking than the simple simulator. Its final-second residual horizontal-pause speeds are around 0.0014–0.0020 m/s, so the 2-second pauses substantially settle horizontal motion. Its vertical pauses retain roughly 0.0050–0.0054 m/s final-second average drift. A fixed two-second pause should not be interpreted as exact rest.

For the latest batch, downward displacement during the immediate up→down segment is 0.728 m, versus 0.752 m for down after a settle (nominal 0.750 m). Immediate reversal therefore loses about 2.4 cm of downward displacement relative to the settled case, although both reach similar late-segment speed. Sim shows 0.353 m versus 0.367 m; sim-time shows 0.364 m versus 0.369 m, with the dominant half-speed defect affecting both.

The initial hover also captures a deployment transient. In the latest batch its final-second mean vertical drift magnitude is about 0.042 m/s, with another 4.37 cm net downward displacement during the subsequent forward segment. Thus the first forward segment is not a fully settled baseline for vertical performance. Later horizontal segments have much smaller altitude drift.

## Reproducibility and limits

- `analyze.py` reads the nine preserved recordings in `recordings/` and regenerates CSV metrics, JSON summary, and plots.
- `phases.csv` contains metrics for every phase and aircraft; `runs.csv` contains timing, data quality, and input filenames.
- Analysis uses ROS receipt timestamps relative to the first published sequence command. These use simulated seconds for sim-time runs. The sequence excludes deployment and return.
- Phase boundaries are the first command for each phase; position at boundaries is linearly interpolated from valid world poses. Requested displacement uses actual recorded phase duration.
- Figures estimate velocity using a centered 0.4-second position difference. This smooths noise but also smears transitions by approximately 0.2 seconds; do not infer precise latency or overshoot peaks from these plots.
- The three observations per group are different aircraft IDs, not repeated trials of the same aircraft. Means and per-run ranges describe this small dataset, not statistical equivalence.
- Logged commands precede collision avoidance and backend transport. The logs do not contain final actuator commands, so an end-to-end tracking difference cannot always be assigned solely to the backend.
- Backend identity is user-supplied metadata, and it is incorrect or ambiguous in the latest files. Record future SITL runs with `--backend sitl`.

Recommended next steps: confirm the latest-batch identity, correct the simulator vertical velocity implementation, rerun all three modes, and extend the initial hover or gate the first movement on a measured settling threshold. Preserve these recordings as the pre-fix baseline.

## Regenerating this analysis

From the package directory, run `python3 analysis/analyze.py` (requires NumPy and Matplotlib). Use `--input /path/to/recordings --output /path/to/results` to override the bundled input and output directories. Original workspace recordings remain untouched.
