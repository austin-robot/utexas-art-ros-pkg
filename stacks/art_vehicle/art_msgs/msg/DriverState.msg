# ART driver states -- similar to those in driver_base.

# $Id$

# constants
uint32 CLOSED = 0       # Not connected to the hardware
uint32 OPENED = 1       # Passively connected to the hardware
uint32 RUNNING = 2      # Sending hardware commands

uint32 state
