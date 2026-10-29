
# DiscreteTraj.msg
# Non-uniform time-parameterized discrete trajectory.

# Unique trajectory identifier.
uint64 trajectory_id

# Reference frame, e.g. odom_ned.
string frame_id

# Relative time [s], N >= 2.
# Must start at 0, be finite and strictly increasing.
float64[] time_from_start

# Position [m], all finite.
float64[] px
float64[] py
float64[] pz

# Velocity [m/s].
# NaN: no explicit velocity constraint.
float64[] vx
float64[] vy
float64[] vz

# Acceleration [m/s^2].
# NaN: no explicit acceleration constraint.
float64[] ax
float64[] ay
float64[] az

# Yaw [rad], all finite.
# Unwrapped internally by the sampler.
float64[] yaw

# Yaw rate [rad/s].
# NaN: no explicit yaw-rate constraint.
float64[] yaw_speed
