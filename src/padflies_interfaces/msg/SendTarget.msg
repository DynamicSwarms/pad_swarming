geometry_msgs/PoseStamped target # Target position and orientation in specific frame (stamp)
bool use_yaw false # if the yaw should be used. Otherwise 0.0 is assumed
bool collision_avoidance true # if the collision avoidance should be used. Otherwise, the target will be sent directly to the crazyflie without checking for collisions
uint32 priority_id # Highest number gets to send target to crazyflie 
string info # Additional information