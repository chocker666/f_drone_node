
# ControlTrajectory.msg
#
# Unified piecewise quintic polynomial trajectory command.
# Only polynomial trajectories are supported.
#
# Coordinate frame: local ENU.
# Units: SI (m, s, rad).
#
# Execution policy:
# - A newly received trajectory must be either accepted and
#   activated immediately, or rejected entirely.
# - No trajectory queuing, delayed activation, or pending state.
# - Rejected trajectories must not affect the active trajectory.
# - On acceptance, the trajectory sampler, physical controller
#   instance, and controller parameter profile must be switched
#   together as one atomic operation.
# - Trajectory time starts at the instant of activation.
#
# Completion policy:
# - After the total trajectory duration has elapsed, hold the
#   terminal position and yaw.
# - Reference velocity, acceleration, and yaw rate become zero.
# - Reaching the trajectory end time does not necessarily mean
#   the vehicle has reached the target state.

# Trajectory identifier.
# Must be non-zero.
# ID 0 is reserved internally to represent no active trajectory.
# Non-zero IDs do not need to be monotonically increasing.
uint64 trajectory_id

# Coordinate frame identifier.
# Must match the configured local ENU control frame.
# No automatic coordinate transformation is performed.
string frame_id

# Number of polynomial segments.
# Must be >= 1.
uint32 segment_count

# Duration of each segment [s].
# Array length = segment_count.
# All values must be finite and > 0.
float64[] segment_duration

# Polynomial coefficients.
#
# All position and yaw polynomials are fixed to degree 5.
#
# For each segment i:
#   f_i(t) = c0 + c1*t + c2*t^2 + c3*t^3
#            + c4*t^4 + c5*t^5
#
# where:
#   t is the local segment time [s]
#   t in [0, segment_duration[i]]
#
# Coefficients are flattened in segment-major order,
# with ascending powers of local segment time.
#
# For segment i, coefficients occupy indices:
#   [6*i, 6*i + 5]
#
# Every coefficient array must have length:
#   segment_count * 6
#
# All coefficients must be finite.
#
# Segment boundary continuity requirements:
# - Position: C2 continuity (position, velocity, acceleration).
# - Yaw: C1 continuity (yaw, yaw rate).
#
# Yaw is represented as an unwrapped angle in radians.

# X position polynomial coefficients [m].
float64[] coeff_x

# Y position polynomial coefficients [m].
float64[] coeff_y

# Z position polynomial coefficients [m].
float64[] coeff_z

# Unwrapped yaw polynomial coefficients [rad].
float64[] coeff_yaw

# Physical controller identifier.
# Must reference an available controller implementation.
# An invalid controller causes the entire command to be rejected.
string controller_id "position_yaw_pid"

# Controller parameter profile name.
# Must reference a valid, preconfigured parameter set.
# An invalid profile causes the entire command to be rejected.
string config_name "default"
