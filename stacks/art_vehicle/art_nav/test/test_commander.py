#!/usr/bin/env python
#
# Unit test for ART commander node
#
#   Copyright (C) 2010 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# This is essentially a scaffold version of the navigator.
#
# $Id$

import roslib;
roslib.load_manifest('art_nav')

import rospy
from art_common.msg import ArtHertz
from art_nav.msg import Behavior
from art_nav.msg import EstopState
from art_nav.msg import NavigatorCommand
from art_nav.msg import NavigatorState

last_order = None

def log_cmd(cmd):
    global last_order
    if not last_order:
        rospy.loginfo("first navigator command received")
    last_order = cmd.order
    rospy.loginfo('order ' + str(cmd.order.behavior.value) + ' received')

def log_state(state_msg):
    rospy.logdebug('publishing ' + str(state_msg.header.seq))

def test():
    topic = rospy.Publisher('navigator/state', NavigatorState)
    rospy.Subscriber('navigator/cmd', NavigatorCommand, log_cmd)
    rospy.init_node('test_commander')

    rospy.loginfo('starting commander test')

    state_msg = NavigatorState()     # navigator state msg with header
    state_msg.header.frame_id = '/map'

    while not rospy.is_shutdown():

        state_msg.header.stamp = rospy.Time.now()

        # echo last order, if any received yet
        if last_order:
            state_msg.last_order = last_order

            # do some E-stop state transitions
            if last_order.behavior.value == Behavior.Run:
                if state_msg.estop.state != EstopState.Run:
                    state_msg.estop.state = EstopState.Run
                    rospy.loginfo('entering Run state')
            elif last_order.behavior.value == Behavior.Pause:
                if state_msg.estop.state != EstopState.Pause:
                    state_msg.estop.state = EstopState.Pause
                    rospy.loginfo('entering Pause state')
            elif (last_order.behavior.value == Behavior.Quit
                  or last_order.behavior.value == Behavior.Abort):
                if state_msg.estop.state != EstopState.Done:
                    state_msg.estop.state = EstopState.Done
                    rospy.loginfo('entering Done state')

        log_state(state_msg)
        topic.publish(state_msg)

        rospy.sleep(1.0/ArtHertz.NAVIGATOR)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('commander test completed')
