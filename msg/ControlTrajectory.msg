# Unified trajectory command in local ENU.
# Accepted trajectories activate immediately.
uint64 trajectory_id
string frame_id

uint8 DISCRETE=0
uint8 PIECEWISE_POLYNOMIAL=1
uint8 trajectory_type

# Only the payload selected by trajectory_type is interpreted.
DiscreteTraj discrete_traj
PolynomialTraj polynomial_traj

# Physical controller and parameter profile are independent.
string controller_id "position_yaw_pid"
string config_name "default"