# Actor-local ORCA and collision blackboard

Padflie commanders select and load their calculation plugin at construction, using
the same `pluginlib::ClassLoader` and `InterfaceTraits` constructor-injection pattern
as pad resource managers. Set the read-only startup parameter:

```yaml
collision_avoidance_plugin: collision_avoidance::Orca
orca.time_step: 0.1
orca.time_horizon: 1.0
orca.neighbor_distance: 1.0
collision_avoidance.max_state_age: 0.2
collision_avoidance.apply: true
```

The `ICollisionAvoidance` interface is in `collision_avoidance_interfaces`. Its ORCA
plugin and shared solver live in `collision_avoidance`. The actor's
`BlackboardCollisionAvoidance` module uses node interfaces and the injected plugin;
it has no concrete solver dependency and makes no collision-avoidance service calls.
Both position and velocity commands use it. Position commands retain the previous
XY target-to-velocity conversion, with unchanged target Z. ORCA is still 2D.

Start one common blackboard (the vicon and scaling launches now do this):

```sh
ros2 run collision_avoidance collision_avoidance_blackboard
```

- `/collision_avoidance/state`: each actor publishes a timestamped `CollisionState`.
- `/collision_avoidance/states`: blackboard broadcasts a `CollisionStates` snapshot.

All vectors use `world`. The actor excludes itself, rejects future/stale/invalid
states, then calculates locally. The ORCA plugin filters neighbors by distance.
Actors publish in both low-level command modes even when avoidance is bypassed,
so other actors still see their position and commanded velocity. State velocity
retains the previous service's commanded-velocity model; it is not measured velocity.
States are published before final controller clipping. High-level takeoff/landing
commands and inactive actors do not publish new states.

Blackboard parameters `publish_period` (0.1 s) and `state_max_age` (10 s) are startup
settings. It retains the latest timestamp per id and only performs broad expiry and
input validity checks; it never solves ORCA. Enable `publish_visualization` for the
existing world-frame markers of actor-published preferred/calculated velocities.
Calculation tuning now belongs to each padflie, not to the blackboard. Actors and
blackboard must use the same ROS clock; use `use_sim_time` consistently in simulation.
Before the first snapshot, or when no fresh neighbors remain, ORCA uses an empty
neighbor list, as the previous service did on initial contact.

`collision_avoidance::Orca` is exported in `plugins.xml`. Alternative calculators
implement `ICollisionAvoidance`, specialize no additional traits, accept the declared
node-parameters interface constructor argument, and export their class with
`PLUGINLIB_EXPORT_CLASS` and a plugin description for `collision_avoidance_interfaces`.
The plugin loader is retained longer than the actor and calculation instance.

Plugin tests cover class loading with constructor injection, free velocity and Z,
speed limiting, head-on avoidance, neighbor-distance filtering, and parameter
validation. They are registered with ament/gtest; no build is performed by this change.

## Legacy service examples

The old service executables remain available for standalone example scenarios; the
actor and updated launches use the blackboard/plugin path. Their commands are below.

### Collision avoidance tests

From the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select collision_avoidance_interfaces collision_avoidance collision_avoidance_examples
source install/setup.bash
```

Start the velocity collision-avoidance service:

```bash
ros2 run collision_avoidance velocity_reciprocal_collision_avoidance
```

In another sourced terminal, run one of the test sets:

```bash
# Basic cases
ros2 run collision_avoidance_examples velocity_reciprocal_test

# Larger, offset, and random cases (8-20 agents)
ros2 run collision_avoidance_examples velocity_reciprocal_scenarios

# Smoothly changing preferred velocities
ros2 run collision_avoidance_examples velocity_reciprocal_dynamic
```

Select one scenario or change its duration:

```bash
ros2 run collision_avoidance_examples velocity_reciprocal_scenarios \
  --ros-args -p scenario:=random_12 -p scenario_duration:=20.0
```

For visualization, start `rviz2`, set the fixed frame to `world`, and add a
`MarkerArray` display for `/visualization_marker_array`.
