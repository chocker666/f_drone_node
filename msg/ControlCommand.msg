# FlightController state-control commands.

uint8 CANCEL=0
uint8 EXIT_IDLE=1
uint8 RESET_FAULT=2

uint8 command

# Used by CANCEL to prevent cancelling a newer trajectory.
uint64 trajectory_id