# Flight controller runtime status.

builtin_interfaces/Time stamp

# Active trajectory ID. Zero means no active trajectory.
uint64 trajectory_id

uint8 IDLE=0
uint8 TRACKING=1
uint8 HOVER=2
uint8 FAULT=3

uint8 state

uint16 FAULT_NONE=0
uint16 FAULT_STATE_TIMEOUT=1
uint16 FAULT_INVALID_STATE=2
uint16 FAULT_CONTROLLER_ERROR=3

uint16 fault_code

# NaN when tracking errors are unavailable.
float64 position_error
float64 yaw_error