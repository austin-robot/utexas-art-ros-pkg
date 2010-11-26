#!/usr/bin/env python
#
# Set relays using ART ioadr interface.
#
#   Copyright (C) 2009 Austin Robot Technology
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
from art_msgs.msg import IOadrCommand
from art_msgs.msg import IOadrState

# global variables

cur_relays = None
started = False
finished = False

def ioadr_state_update(state):
    "ioadr/state message callback"
    global cur_relays, started, finished
    if started:
        if cur_relays != state.relays:
            rospy.loginfo("relays changed from 0x%02x to 0x%02x",
                          cur_relays, state.relays)
            finished = True
    cur_relays = state.relays
    started = True

def relays(relays_on, relays_off = 0):
    "set and clear IOADR8x relays"
    rospy.init_node('relays')
    topic = rospy.Publisher('ioadr/cmd', IOadrCommand)
    rospy.Subscriber('ioadr/state', IOadrState, ioadr_state_update)

    rospy.loginfo('waiting for relays from ioadr driver')
    global started
    while not started:
        rospy.sleep(0.1)        # wait for relay state input
        if rospy.is_shutdown():
            rospy.logwarn('command interrupted, node shut down')
            return 

    # TODO check if no change required, exit if so
    global cur_relays
    old_relays = cur_relays
    if ((old_relays & ~relays_off) | relays_on) == old_relays:
        rospy.loginfo('nothing to do, relays already 0x%02x', old_relays)
        return

    cmd = IOadrCommand()        # ioadr/cmd message
    cmd.relays_on = relays_on
    cmd.relays_off = relays_off
    rospy.loginfo('sending command now')

    global finished
    while not finished and not rospy.is_shutdown():
        topic.publish(cmd)
        rospy.sleep(0.5)        # wait for relays to change

    rospy.loginfo('finished setting relays')

def usage():
    "print usage message"
    print("""
Usage:

    rosrun art_servo relays.py <set> [<clear>]

Where <set> and <clear> are hex relay values, such as 0x80 or 4.

Not implemented:

  Allow <set> and <clear> to be relay names such as: ENABLED, RUN,
  FLASHER, ALARM, LASER_FRONT, LASER_TOP, TURN_LEFT, or TURN_RIGHT.
  These names are not case-sensitive.

""")

if __name__ == '__main__':

    if len(sys.argv) < 2:
        usage()
        exit(1)

    #gears = {'park':    IOadrState.Park,
    #         'reverse': IOadrState.Reverse,
    #         'neutral': IOadrState.Neutral,
    #         'drive':   IOadrState.Drive}
    #
    #expand_abbrev = {'p': 'park',
    #                 'r': 'reverse',
    #                 'n': 'neutral',
    #                 'd': 'drive'}
    #
    #gear_name = string.lower(sys.argv[1])
    #
    ## expand abbreviated names
    #if gear_name in expand_abbrev:
    #    gear_name = expand_abbrev[gear_name]
    #
    #if not gear_name in gears:
    #    print('unknown gear:', gear_name)
    #    usage()
    #    exit(2)
    #
    #target_gear = gears[gear_name]

    relays_on = int(sys.argv[1], 16)
    relays_off = 0
    if len(sys.argv) > 2:
        relays_off = int(sys.argv[2], 16)
    rospy.loginfo('setting relays on: ' + hex(relays_on)
                  + ', off: ' + hex(relays_off))

    try:
        relays(relays_on, relays_off)
    except rospy.ROSInterruptException: pass

    exit(0)
