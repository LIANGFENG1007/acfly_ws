# Local main-controller feedback, NOT an acknowledgement from the flight controller.
std_msgs/Header header
std_msgs/Header mission
bool position_active
bool ready
# Fixed XY handover/watchdog anchor; Z always remains the requested corridor height.
geometry_msgs/Point hold_position
