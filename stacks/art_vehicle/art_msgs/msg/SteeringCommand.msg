# ART steering controller command message

# $Id$

# The maximum allowed steering angle is ArtVehicle::max_steer_degrees
# (29 degrees) in either direction, defined in the art_msgs/ArtVehicle
# message.

# The driver enforces this limit to protect the vehicle from
# mechanical damage.  There is about a two degree safety margin
# included in that limit.

Header  header

# request types
uint32 Degrees = 0     # set absolute angle in degrees: +left, 0 center, -right
uint32 Relative = 1    # change angle relative to current setting (degrees)

uint32  request                # request type
float32 angle                  # requested angle (degrees)


