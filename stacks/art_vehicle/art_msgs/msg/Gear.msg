# ART vehicle transmission gear numbers
#
# Used by several different messages.

# $Id$

# Gear numbers. 
#
# Naught means: reset all Shifter relays; no change of CarDrive gear.
uint8 Naught = 0
uint8 Park = 1
uint8 Reverse = 2
uint8 Neutral = 3
uint8 Drive = 4
uint8 N_gears = 5

uint8 value		# requested or reported gear number
