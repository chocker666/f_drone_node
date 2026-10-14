
# ControlStatus.msg
# Tracking feedback from FlightController to FlightExecutor.

builtin_interfaces/Time stamp
uint64 trajectory_id

# Controller execution states.
uint8 IDLE=0
uint8 WAITING=1
uint8 TRACKING=2
uint8 HOLDING=3
uint8 FAULT=4

uint8 state

# Current reference and actual position, ENU, m.
geometry_msgs/Point reference_position
geometry_msgs/Point actual_position

# Reference and actual velocity, ENU, m/s.
geometry_msgs/Vector3 reference_velocity
geometry_msgs/Vector3 actual_velocity

# Signed tracking errors: reference - actual.
geometry_msgs/Vector3 position_error
geometry_msgs/Vector3 velocity_error

# Euclidean error norms.
float64 position_error_norm
float64 velocity_error_norm

# Relative time from trajectory start, seconds.
float64 trajectory_time

# Fault information.
uint16 FAULT_NONE=0
uint16 FAULT_STATE_TIMEOUT=1
uint16 FAULT_INVALID_STATE=2
uint16 FAULT_CONTROLLER_ERROR=3

uint16 fault_code
