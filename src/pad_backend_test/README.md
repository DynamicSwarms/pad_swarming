# Pad backend velocity experiment

Uses the existing `pad_management/vicon.launch.py` stack. Select one backend per
run; do not start multiple stacks in the same ROS domain. SITL uses the hardware
UDP-radio backend. No aircraft moves until the separate runner is invoked.

Build from `/home/winni/ds/pad_swarming`:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
colcon build --packages-select pad_backend_test
source install/setup.bash
ros2 launch pad_backend_test vicon_test.launch.py backend:=simulation
```

In another sourced terminal:

```bash
ros2 run pad_backend_test velocity_test --backend simulation --id 0
```

Repeat with launch `backend:=sitl` and runner `--backend sitl --id 0`.
For real hardware use launch `backend:=hardware` and runner
`--backend hardware --id 0xA1` (choose the actual configured aircraft ID).
The runner's backend argument labels the recording; it does not switch or verify
an already running stack. Radio/Vicon connection and CF creation belong to the
existing launch. The runner waits for the configured, inactive Padflie, activates
it, deploys, streams world-frame velocities, returns, and deactivates.

Default sequence is grouped into four parts after an initial 2-second hover:

1. Settled horizontal motion: forward, pause, right, pause, forward-right, pause.
2. Immediate reversal: backward directly into forward, then pause.
3. Immediate vertical reversal: up directly into down, then pause.
4. Settled vertical and combined motion: up, pause, down, pause, up-forward, final pause.

Every movement lasts 5 seconds; pauses command zero velocity for 2 seconds.
The sequence takes 68 seconds, excluding deploy/return. There is intentionally
no zero-velocity segment between backward and forward or within the first up/down pair, so the recording captures
both stopping/settling and an immediate velocity reversal. Pauses are fixed-duration
holds, not a measured guarantee that the aircraft has settled.
Forward is world +x, right is world -y, and up is world +z. Translation speed is
0.15 m/s; diagonal components are 0.106066 m/s to retain approximately the same
speed. Publication is 20 Hz and collision avoidance is enabled. This sequence
is not displacement-balanced; the commander returns to its selected site afterward.
The JSON keeps each step on a single line for editing.

Deploy and Return use the commander's default targets/sites. A successful action
result is required before streaming commands. The runner should have exclusive
control of the selected aircraft. An already active Padflie is not taken over.

Options: `--sequence /path/sequence.json`, `--rate 20`, `--timeout 90`,
`--info-timeout 1`, `--output /path/results`. Sequence entries contain `name`,
`duration` (seconds), and `velocity` `[vx, vy, vz, yaw_rate]` in m/s and rad/s.
Review the sequence and available flight space before running hardware.

Each uniquely named JSONL file records metadata, every received `/padflie<ID>/info`
message, every published SendTarget (including its `info` label), action results,
and final success/failure. Entries include monotonic elapsed time, ROS time, and
phase. Compare runs by phase and elapsed time; `pose_world` is suitable for deriving
measured velocity and displacement, and the command records supply the requested
velocity. This is an experiment/recording harness, not a claim that the backends
meet an unspecified numerical equivalence tolerance.

Missing telemetry, invalid world pose, critical battery, rejected/failed actions,
and timeouts fail the run. SIGINT/SIGTERM keep ROS alive for cleanup. Cleanup sends
zero velocity, requests Return, and attempts normal lifecycle deactivation even
if Return fails. Deactivation itself also invokes the commander's return path.
A network loss, killed process, or failed return cannot guarantee recovery; inspect
reported failures. Logs are flushed throughout so partial runs remain readable.
The upstream Padflie state field is currently hardcoded to 1, so this test does
not interpret it as flight completion. Existing Connect/priority fields are unused
by the current commander implementation.

Offline verification (source ROS and workspace first):

```bash
python3 -m pytest src/pad_backend_test/test
```

## Real-time and accelerated simulation

By default the runner still uses wall time. To make the same 5-second movements
follow simulation time at either 1x or faster, start the wrapper with a shared clock:

```bash
ros2 launch pad_backend_test vicon_test.launch.py backend:=simulation use_sim_time:=true speed:=1.0
ros2 run pad_backend_test velocity_test --backend simulation --id 0 --use-sim-time
```

For an accelerated run, stop the previous stack and relaunch with `speed:=4.0`;
use the same runner command. The 68-second sequence then targets approximately
17 wall seconds, excluding deploy/return, subject to compute and DDS throughput.
Only run one `/clock` publisher. The wrapper applies `use_sim_time` to the included
Vicon stack; the simulation gateway propagates it to spawned Crazyflies.
The clock advances in 10 ms simulation increments and requests a correspondingly
scaled wall timer rate. It does not synchronize each clock tick with all physics
and control callbacks, so high acceleration is not a lockstep or fidelity guarantee.

Step durations and the command publication schedule use the selected clock;
`--rate 20` means 20 commands per simulated second with `--use-sim-time`.
Recordings retain wall `elapsed` and `ros_time_ns`, and add `sequence_elapsed`
(in the selected time domain, starting after deployment) and clock mode metadata.
Service/action timeouts, telemetry freshness, and `--clock-timeout 5` use wall time
so clock loss cannot hang the test. A backwards clock jump aborts the sequence;
a pause longer than the clock timeout also aborts and attempts cleanup.
Hardware and SITL remain wall-time only in this wrapper; accelerated SITL has not
been integrated. Both launch and runner reject simulation-time mode for those backends.

## Recorded backend comparison

The [analysis report](analysis/report.md), plots, per-run and per-phase CSV metrics, JSON summary, reproduction script, and twelve input recordings are in `analysis/`. The dataset includes three hardware runs. Regenerate the outputs with `python3 analysis/analyze.py` from this package directory (NumPy and Matplotlib required). The report explains the provisional SITL labels.
