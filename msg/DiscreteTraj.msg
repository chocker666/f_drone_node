
# DiscreteTraj.msg
# Time-parameterized discrete trajectory.
# All sample arrays must have the same length N >= 2.

# Unique trajectory identifier.
uint64 trajectory_id

# Reference coordinate frame, e.g. odom_ned.
string frame_id

# Scheduled activation time in ROS time.
# Zero means activation at the next eligible control-cycle boundary.
builtin_interfaces/Time start_time

# Relative sample times [s].
# Must be finite, start at 0.0, and be strictly increasing.
float64[] time_from_start

# Position reference [m].
# All components must be finite.
float64[] px
float64[] py
float64[] pz

# Velocity reference [m/s].
# NaN means unspecified.
float64[] vx
float64[] vy
float64[] vz

# Acceleration reference [m/s^2].
# NaN means unspecified.
float64[] ax
float64[] ay
float64[] az

# Yaw reference [rad].
# Must be finite; interpolation uses angle unwrapping.
float64[] yaw

# Yaw angular velocity [rad/s].
# NaN means unspecified.
float64[] yaw_speed
