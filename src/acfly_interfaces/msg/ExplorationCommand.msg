# Atomically carry XY/yaw, altitude and a deduplicated release request.
std_msgs/Header header
std_msgs/Header mission
geometry_msgs/Twist velocity
float64 altitude
float64 vertical_speed_limit
# Zero means no release. 1..3 select the physical package, not the visual class.
uint8 release_index
uint64 target_id
