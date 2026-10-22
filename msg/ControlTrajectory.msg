# Unified trajectory command in local ENU.
# Accepted trajectories activate immediately.
uint64 trajectory_id
string frame_id

uint8 POINT=0
uint8 LIST=1
uint8 PARAMETRIC=2
uint8 trajectory_type

# Only the payload selected by trajectory_type is interpreted.
FixedTrajectory point_params
ListTrajectory list_params

# PARAMETRIC is reserved.
# Its payload and sampler design have not been determined.

# Physical controller and parameter profile are independent.
string controller_id "position_yaw_pid"
string config_name "default"