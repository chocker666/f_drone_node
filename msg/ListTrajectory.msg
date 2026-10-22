# Spatial path in the parent ControlTrajectory frame.
# The first point is the path origin.
# The receiver does not prepend the current vehicle position.
# No per-point timestamps.
#
# Translation and yaw use independent timelines.
# They do not wait for each other at intermediate points.
# A channel that finishes first holds its final reference.
#
# FixedTrajectory carries position (m), yaw (rad), and yaw_direction.
# For each point after the first, yaw_direction selects rotation
# from the previous unwrapped yaw to this point.
# The first point's direction selects initial controller yaw alignment.
FixedTrajectory[] points

# Desired translation speed, m/s.
# Zero means unspecified: use max_speed.
# A positive request above max_speed is capped at max_speed.
float64 target_speed

# Positive finite translation limits.
float64 max_speed
float64 max_acceleration

# Positive finite yaw limits, rad/s and rad/s^2.
float64 max_yaw_rate
float64 max_yaw_acceleration