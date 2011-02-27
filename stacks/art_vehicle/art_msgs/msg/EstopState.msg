# Navigator E-stop state values
# $Id$

uint16 Pause    = 0			# E-stop pause (initial state)
uint16 Run      = 1			# E-stop run enabled
uint16 Done     = 2			# mission finished (disabled)
uint16 Suspend  = 3			# suspend autonomous operation
uint16 N_states = 4

uint16 state
