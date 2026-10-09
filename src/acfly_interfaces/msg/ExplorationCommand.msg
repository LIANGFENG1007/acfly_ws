# Atomically carry XY/yaw, altitude and a deduplicated release request.
std_msgs/Header header
std_msgs/Header mission
geometry_msgs/Twist velocity
float64 altitude
float64 vertical_speed_limit
# Zero means no release. 1..3 select the physical package, not the visual class.
uint8 release_index
uint64 target_id
# One atomic command selects exactly one output interface. Default remains velocity.
uint8 VELOCITY=0
uint8 POSITION=1
uint8 control_mode
# POSITION: camera_init XYZ and absolute yaw (rad); velocity is ignored.
geometry_msgs/Point position
float64 yaw
