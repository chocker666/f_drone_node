uint8 POSITION=0
uint8 VELOCITY=1
uint8 ACCELERATION=2
uint8 ATTITUDE=3
uint8 BODY_RATE=4

uint8 control_mode

builtin_interfaces/Time stamp

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
bool use_jerk
geometry_msgs/Vector3 jerk
float32 yaw       # deg, ROS ENU, only POSITION
float32 yawspeed  # deg/s, ROS ENU, only VELOCITY
