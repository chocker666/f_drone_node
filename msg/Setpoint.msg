uint8 POSITION=0
uint8 VELOCITY=1
uint8 ACCELERATION=2
uint8 ATTITUDE=3
uint8 BODY_RATE=4

builtin_interfaces/Time stamp
uint8 control_mode

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
bool use_jerk
geometry_msgs/Vector3 jerk

float32 yaw       # deg, ENU, POSITION
float32 yawspeed  # deg/s, ENU, VELOCITY and ACCELERATION

geometry_msgs/Quaternion orientation  # FLU body -> ENU world
geometry_msgs/Vector3 body_rate        # FLU, rad/s
geometry_msgs/Vector3 thrust           # FLU, normalized [-1, 1]