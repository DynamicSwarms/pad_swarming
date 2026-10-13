# All vectors use the world frame; ORCA operates on their XY components.
uint8 id
builtin_interfaces/Time stamp
geometry_msgs/Vector3 position
geometry_msgs/Vector3 preferred_velocity
geometry_msgs/Vector3 velocity # Last commanded velocity
geometry_msgs/Vector3 calculated_velocity # ORCA result, even when bypassed
float64 radius
float64 max_speed
