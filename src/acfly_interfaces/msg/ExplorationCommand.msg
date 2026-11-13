# header: command timestamp; frame_id is base_link.
# mission: exact header of the active /exploration/goal in camera_init.
std_msgs/Header header
std_msgs/Header mission
# Body-frame horizontal velocity (m/s) and yaw rate (rad/s).
geometry_msgs/Twist velocity
# Altitudes use the lidar SLAM frame (m); vertical speed limit is in m/s.
float64 altitude
float64 vertical_speed_limit
uint8 VELOCITY=0
uint8 control_mode
