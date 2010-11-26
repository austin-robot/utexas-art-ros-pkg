#!/usr/bin/env python
#
# Send command to ART shifter interface.
#
#   Copyright (C) 2005, 2007, 2009 Austin Robot Technology
#
#   License: Modified BSD Software License Agreement
#
# $Id$
#

# make print compatible with python 2.6 and 3.0
from __future__ import print_function

import roslib;
roslib.load_manifest('art_servo')

import sys
import string
import rospy
from art_msgs.msg import Shifter

# global variables
cur_gear = None
target_gear = None
gear_name = ''
finished = False

def shifter_state_update(state):
    "shifter/state message callback"
    global cur_gear, finished
    if cur_gear:
        if cur_gear != state.gear:
            rospy.logdebug("shifted from %u to %u", cur_gear, state.gear)
        if not finished and cur_gear == target_gear:
            rospy.loginfo("transmission now in " + gear_name)
            finished = True
    cur_gear = state.gear

def log_shifter_cmd(cmd):
    "log shifter/cmd message"
    rospy.logdebug("sending shifter command: " + str(cmd.gear))

def shift(gear):
    "shift transmission to desired gear"
    topic = rospy.Publisher('shifter/cmd', Shifter)
    rospy.Subscriber('shifter/state', Shifter, shifter_state_update)
    rospy.init_node('shift')

    rospy.loginfo('Shifting transmission into ' + gear_name)

    # initially reset shifter
    cmd = Shifter()                     # shifter message
    cmd.gear = gear
    log_shifter_cmd(cmd)
    topic.publish(cmd)

    # The Arens Controls hardware mechanism requires holding the shift
    # relay on for one second before resetting it.
    rospy.sleep(1.0)                    # wait a second

    # TODO: Make sure the shift actually occurred before continuing.
    # The command could possibly have been lost.  This version depends
    # on the user to notice if anything went wrong.
    #
    # Moreover, this script might as well not bother to send a command
    # if the transmission was already in the desired gear.

    reset_cmd = Shifter()               # reset shifter relay message
    reset_cmd.gear = Shifter.Reset
    log_shifter_cmd(reset_cmd)
    topic.publish(reset_cmd)

    rospy.sleep(1.0)                    # wait another second

    rospy.loginfo('Finished shifting transmission')

def usage():
    "print usage message"
    print("""
Usage:

    rosrun art_servo shift.py <gear>

Where <gear> is one of: park, reverse, neutral, drive.  These names
are not case-sensitive, and may be abbreviated to the first character.

""")

if __name__ == '__main__':

    if len(sys.argv) < 2:
        usage()
        exit(1)

    gears = {'park': Shifter.Park,
             'reverse': Shifter.Reverse,
             'neutral': Shifter.Neutral,
             'drive': Shifter.Drive}

    expand_abbrev = {'p': 'park',
                     'r': 'reverse',
                     'n': 'neutral',
                     'd': 'drive'}

    gear_name = string.lower(sys.argv[1])

    # expand abbreviated names
    if gear_name in expand_abbrev:
        gear_name = expand_abbrev[gear_name]

    if not gear_name in gears:
        print('unknown gear:', gear_name)
        usage()
        exit(2)

    target_gear = gears[gear_name]

    try:
        shift(target_gear)
    except rospy.ROSInterruptException: pass

    exit(0)
