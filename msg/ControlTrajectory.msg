# Unified trajectory command.

uint64 trajectory_id

builtin_interfaces/Time start_time

string frame_id

# Trajectory type
uint8 FIXED=0
uint8 trajectory_type

# Fixed trajectory parameters
FixedTrajectory fixed_params

# Controller type
uint8 PID=0
uint8 controller_type