#!/usr/bin/env python
#
# ART navigator unit test for E-stop state
#
#   Copyright (C) 2010 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

import roslib;
roslib.load_manifest('art_nav')

import rospy
from art_nav.msg import Behavior
from art_nav.msg import EstopState
from art_map.msg import MapID
from art_nav.msg import NavigatorCommand
from art_nav.msg import NavigatorState

import sys

last_state_ = None

def log_state(state_msg):
    "log navigator state information"
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
    rospy.init_node('navigator_state')
    rospy.loginfo('setting navigator E-stop state to: ' + str(behavior))

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
    print 'usage: estop.py <state>'
    print ''
    print 'options'
    print '    p    Pause'
    print '    q    Quit (Done)'
    print '    r    Run'
    sys.exit(9)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print 'exactly one parameter expected'
        usage()

    new_state = sys.argv[1]
    if new_state == 'r':
        new_state = EstopState.Pause
        behavior = Behavior.Pause
    elif new_state == 'p':
        new_state = EstopState.Run
        behavior = Behavior.Run
    elif new_state == 'q':
        new_state = EstopState.Done
        behavior = Behavior.Quit
    else:
        print 'unknown parameter:', new_state
        usage()

    try:
        estop(behavior, new_state)
    except rospy.ROSInterruptException: pass
