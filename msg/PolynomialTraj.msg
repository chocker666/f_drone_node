
# PolynomialTraj.msg
# Time-parameterized piecewise polynomial trajectory.

# Unique trajectory identifier.
uint64 trajectory_id

# Reference coordinate frame, e.g. odom_ned.
string frame_id

# Scheduled activation time in ROS time.
# Zero means activation at the next eligible control-cycle boundary.
builtin_interfaces/Time start_time

# Number of polynomial segments, must be >= 1.
uint32 segment_count

# Duration of each segment [s].
# Length = segment_count.
# All durations must be finite and strictly positive.
float64[] segment_duration

# Polynomial degrees.
# Position axes share position_degree.
# Yaw uses yaw_degree independently.
uint32 position_degree
uint32 yaw_degree

# Flattened polynomial coefficients.
# Segment-major order, ascending powers of local time.
#
# Position coefficient array length:
# segment_count * (position_degree + 1)
#
# Yaw coefficient array length:
# segment_count * (yaw_degree + 1)
#
# For segment i, position coefficients start at:
# i * (position_degree + 1)
#
# For segment i, yaw coefficients start at:
# i * (yaw_degree + 1)
#
# All coefficients must be finite.

# X position polynomial coefficients [m, m/s, ...].
float64[] coeff_x

# Y position polynomial coefficients.
float64[] coeff_y

# Z position polynomial coefficients.
float64[] coeff_z

# Yaw polynomial coefficients [rad, rad/s, ...].
# Yaw is represented as a continuous unwrapped angle.
float64[] coeff_yaw
