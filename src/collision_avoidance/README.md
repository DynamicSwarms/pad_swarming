# Collision avoidance tests

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
