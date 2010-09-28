# ART brake controller command message

# $Id$

Header  header

# request types
uint32 Absolute = 0    # set absolute position: 0.0 is off, 1.0 is fully on
uint32 Relative = 1    # change relative to current setting, [-1.0 to 1.0]

uint32  request                # request type
float32 position               # requested position


