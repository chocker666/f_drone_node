# Spatial path in the parent ControlTrajectory frame.
# No per-point timestamps.
#
# FixedTrajectory carries position (m), yaw (rad), and yaw_direction.
# For each point after the first, yaw_direction selects the rotation
# from the previous point to this point.
FixedTrajectory[] points

# Desired translation speed, m/s.
# Zero means unspecified: use max_speed.
float64 target_speed

# Positive finite translation limits.
float64 max_speed
float64 max_acceleration

# Positive finite yaw limits, rad/s and rad/s^2.
float64 max_yaw_rate
float64 max_yaw_acceleration