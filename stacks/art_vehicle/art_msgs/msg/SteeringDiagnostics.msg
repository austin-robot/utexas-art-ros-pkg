# ART steering controller diagnostics message

# This message contains data not needed for normal operation of the
# device, but useful for diagnosing problems with it.

# $Id$

Header  header

int32 encoder                   # current reported encoder value
int32 last_request              # last requested encoder value
int32 center_ticks              # initial center encoder value
uint16 status_word		# current internal status
