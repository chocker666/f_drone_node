# Controller types

uint8 NONE=0
uint8 PID=1
uint8 LQR=2
uint8 MPC=3
uint8 ADRC=4

# Selected controller type
uint8 type

# Parameters for PID controller
# Valid when type == PID
PIDParams pid

# Parameters for other controller types
# Reserved for future versions
