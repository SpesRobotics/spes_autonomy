uint8 REVERSING_NO = 0
uint8 REVERSING_AUTO = 1
uint8 REVERSING_FORCE = 2

std_msgs/Header header
string odom_frame

geometry_msgs/Pose2D target

spes_msgs/MoveProperties linear_properties
spes_msgs/MoveProperties angular_properties
bool ignore_obstacles
builtin_interfaces/Duration timeout
uint8 reversing

bool rotate_towards_goal true
bool translate true
bool rotate_at_goal true
