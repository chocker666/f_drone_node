
# ControlTrajectory.msg
# Unified trajectory command from FlightExecutor to FlightController.

# Trajectory identity.
uint64 trajectory_id

# Absolute trajectory start time (ROS time).
builtin_interfaces/Time start_time

# Reference coordinate frame. Currently only "map" is supported.
string frame_id

# Trajectory types.
uint8 FIXED=0

uint8 trajectory_type

# Trajectory-specific parameters.
FixedTrajectory fixed_params

# Controller types.
uint8 PID=0

uint8 controller_type

# Controller-specific parameters.
PIDParams pid_params
