# ControlStatus.msg
# Execution status and tracking errors from FlightController.

builtin_interfaces/Time stamp
uint64 trajectory_id

# Execution states.
uint8 IDLE=0
uint8 WAITING=1
uint8 TRACKING=2
uint8 HOLDING=3
uint8 FAULT=4

uint8 state

# Fault codes.
uint16 FAULT_NONE=0
uint16 FAULT_STATE_TIMEOUT=1
uint16 FAULT_INVALID_STATE=2
uint16 FAULT_CONTROLLER_ERROR=3

uint16 fault_code

# Euclidean norm of position tracking error, ENU, m.
float64 position_error

# Continuous signed yaw tracking error, rad.
# Positive: CCW; negative: CW.
float64 yaw_error