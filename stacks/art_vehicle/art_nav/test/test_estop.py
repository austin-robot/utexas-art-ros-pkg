#!/usr/bin/env python
#
# ART navigator unit test for E-stop state
#
#   Copyright (C) 2010 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

# make print compatible with python 2.6 and 3.0
from __future__ import print_function

import roslib;
roslib.load_manifest('art_nav')

import rospy
from art_msgs.msg import Behavior
from art_msgs.msg import EstopState
from art_msgs.msg import MapID
from art_msgs.msg import NavigatorCommand
from art_msgs.msg import NavigatorState

import sys

last_state_ = None

def log_state(state_msg):
    "log navigator state information"
    global last_state_
    # TODO print state labels, not just numbers
    rospy.loginfo('E-stop: ' + str(state_msg.estop.state)
                  + ' Road: ' + str(state_msg.road.state)
                  + ' last way-point: ' + str(state_msg.last_waypt.seg)
                  + '.' + str(state_msg.last_waypt.lane)
                  + '.' + str(state_msg.last_waypt.pt))
    last_state_ = state_msg

def estop(behavior, new_state):
    cmd = rospy.Publisher('navigator/cmd', NavigatorCommand)
    rospy.Subscriber('navigator/state', NavigatorState, log_state)
    rospy.init_node('estop')
    rospy.loginfo('sending behavior: ' + str(behavior))
    rospy.loginfo('setting E-stop state to: ' + str(new_state))

    cmd_msg = NavigatorCommand()     # navigator command msg
    cmd_msg.header.frame_id = '/map'
    cmd_msg.order.behavior.value = behavior

    # keep sending this behavior until requesting state reached
    while not rospy.is_shutdown():
        if last_state_ and last_state_.estop.state == new_state:
            break
        cmd_msg.header.stamp = rospy.Time.now()
        cmd.publish(cmd_msg)
        rospy.sleep(0.2)

    rospy.loginfo('E-stop state set to ' + str(new_state))

def usage():
    "print usage message"
    print('usage: estop.py <state>')
    print('')
    print('options')
    print('    p    Pause')
    print('    q    Quit (Done)')
    print('    r    Run')
    sys.exit(9)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('exactly one parameter expected')
        usage()

    option = sys.argv[1]
    if option == 'p':
        new_state = EstopState.Pause
        behavior = Behavior.Pause
    elif option == 'r':
        new_state = EstopState.Run
        behavior = Behavior.Run
    elif option == 'q':
        new_state = EstopState.Done
        behavior = Behavior.Quit
    else:
        print('unknown parameter:', option)
        usage()

    try:
        estop(behavior, new_state)
    except rospy.ROSInterruptException: pass
