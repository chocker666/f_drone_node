
# PIDParams.msg
# Parameters for the position-to-velocity PID controller.

float64 kp
float64 ki
float64 kd

# Maximum velocity command norm, m/s.
float64 max_speed

# Maximum velocity-command slew rate, m/s^2.
float64 max_acceleration

# Integral error norm limit, m*s.
float64 integral_limit
