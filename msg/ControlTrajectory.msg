# Unified trajectory command.

uint64 trajectory_id

builtin_interfaces/Time start_time

string frame_id

uint8 FIXED=0
uint8 trajectory_type

FixedTrajectory fixed_params

uint8 PID=0
uint8 controller_type

# Position-to-velocity controller.
PIDParams position_pid_params

# Yaw-to-yaw-rate controller.
PIDParams yaw_pid_params