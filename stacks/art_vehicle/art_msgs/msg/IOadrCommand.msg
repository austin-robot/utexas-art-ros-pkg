# NCD IOADR8x controller command message for ART vehicle
#
# Sets or clears relays.  See: IOadrState for bit definitions.

# $Id$

Header  header          # standard ROS message header

uint8 relays_on         # relays to set
uint8 relays_off        # relays to clear
