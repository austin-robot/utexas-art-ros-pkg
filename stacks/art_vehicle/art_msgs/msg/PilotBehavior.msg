# ART autonomous vehicle pilot node behaviors.
#
# Normally, the pilot node does Run, continually sending commands to
# the servo device actuators and monitoring their state.  With Pause,
# the pilot becomes passive, allowing a learning algorithm or human
# controller direct access to those devices.  In the Off state,
# various devices are shut down: the transmission in Park, the brake
# released, the throttle at idle.  The engine is not turned off, but
# it could be.

# $Id$

# Behavior value
uint8 value

# Behavior numbers:
uint8 Run = 0                   # normal driving
uint8 Pause = 1                 # stop issuing servo commands
uint8 Off = 2                   # turn off devices
uint8 N_behaviors = 3
