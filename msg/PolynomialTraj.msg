
# PolynomialTraj.msg
# Piecewise polynomial trajectory.
#
# All trajectory times are relative to activation.
# All polynomial coefficients must be finite.
# Units: SI (m, s, rad).

# Unique trajectory identifier.
uint64 trajectory_id

# Reference coordinate frame, e.g. odom_ned.
string frame_id

# Number of polynomial segments, must be >= 1.
uint32 segment_count

# Duration of each segment [s].
# Length = segment_count.
# All values must be finite and > 0.
float64[] segment_duration

# Polynomial degrees.
# X/Y/Z share the same position_degree.
# Yaw uses an independent yaw_degree.
uint32 position_degree
uint32 yaw_degree

# Flattened polynomial coefficients.
#
# Coefficients are stored in segment-major order,
# with ascending powers of local segment time.
#
# For segment i:
# p_i(t) = c0 + c1*t + ... + cn*t^n
# where t is local time in [0, segment_duration[i]].
#
# Position array length:
#   segment_count * (position_degree + 1)
#
# Yaw array length:
#   segment_count * (yaw_degree + 1)
#
# All coefficients must be finite.

# X position coefficients.
float64[] coeff_x

# Y position coefficients.
float64[] coeff_y

# Z position coefficients.
float64[] coeff_z

# Unwrapped yaw coefficients [rad].
float64[] coeff_yaw
