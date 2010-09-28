# ART shifter message
#
# Used to both request and report gear shifts.

# $Id$

# Our Arens Controls hardware mechanism requires holding the shift
# relay on for one second before resetting it.  A command node must
# request the desired gear, then wait a second before sending the
# Reset command.  To be safe, it should check that the shift actually
# occurred before continuing.

Header  header          # standard ROS message header

# gear numbers
uint8 Reset = 0         # request reset of shifter relays
uint8 Park = 1
uint8 Reverse = 2
uint8 Neutral = 3
uint8 Drive = 4

uint8 gear              # requested or reported gear number
uint8 relays            # current relay values
