# Generic fields understood by the commander.
string name
string takeoff_plugin_name
string landing_plugin_name
bool available

# Pad-specific fields. Other site providers may leave these empty/defaulted.
string pad_idle_target_service_name
string pad_right_control_action_name
string[] pad_tf_names
builtin_interfaces/Duration wait_time

uint8 charging_speed

uint8 CHARGING_SPEED_NONE = 0
uint8 CHARGING_SPEED_SLOW = 1
uint8 CHARGING_SPEED_FAST = 2
