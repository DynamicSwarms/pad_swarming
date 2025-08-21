string cf_prefix
geometry_msgs/PoseStamped pose
bool pose_valid false
geometry_msgs/Pose pose_world
bool pose_world_valid false
bool is_home
uint8 battery
uint8 padflie_state

uint8 BATTERY_STATE_CRITICAL = 0
uint8 BATTERY_STATE_LOW = 1
uint8 BATTERY_STATE_OK = 2

uint8 PADFLIE_STATE_DEACTIVATED = 0
uint8 PADFLIE_STATE_IDLE = 1
uint8 PADFLIE_STATE_TAKEOFF = 2
uint8 PADFLIE_STATE_LAND = 3
uint8 PADFLIE_STATE_TARGET = 4