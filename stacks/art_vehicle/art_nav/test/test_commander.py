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

from art_msgs.msg import ArtHertz
from art_msgs.msg import Behavior
from art_msgs.msg import EstopState
from art_msgs.msg import MapID
from art_msgs.msg import NavigatorCommand
from art_msgs.msg import NavigatorState
from art_msgs.msg import RoadState

last_order = None

def log_cmd(cmd):
    global last_order
    if not last_order:
        rospy.loginfo("first navigator command received")
    last_order = cmd.order
    rospy.loginfo('order ' + str(cmd.order.behavior.value) + ' received')

def log_state(state_msg):
    rospy.logdebug('publishing ' + str(state_msg.header.seq))

def estop_state_changes(state_msg):
    "do some E-stop state transitions, returns updated state message"
    if last_order.behavior.value == Behavior.Run:
        if state_msg.estop.state != EstopState.Run:
            state_msg.estop.state = EstopState.Run
            rospy.loginfo('entering E-stop Run state')
    elif last_order.behavior.value == Behavior.Pause:
        if state_msg.estop.state != EstopState.Pause:
            state_msg.estop.state = EstopState.Pause
            rospy.loginfo('entering E-stop Pause state')
    elif (last_order.behavior.value == Behavior.Quit
          or last_order.behavior.value == Behavior.Abort):
        if state_msg.estop.state != EstopState.Done:
            state_msg.estop.state = EstopState.Done
            rospy.loginfo('entering E-stop Done state')
    return state_msg

def road_state_changes(state_msg):
    """do some road state transitions when running
       returns updated state message"""
    rospy.loginfo('Running, checking behavior')
    if last_order.behavior.value == Behavior.Initialize:
        if state_msg.road.state == RoadState.Init:
            state_msg.road.state = RoadState.Follow
            # set initial way-point to 1.1.1
            state_msg.last_waypt.seg = 1
            state_msg.last_waypt.lane = 1
            state_msg.last_waypt.pt = 1
            rospy.loginfo('entering Road Follow state')
    return state_msg

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
            state_msg = estop_state_changes(state_msg)
            if state_msg.estop.state == EstopState.Run:
                state_msg = road_state_changes(state_msg)
            state_msg.last_order = last_order

        log_state(state_msg)
        topic.publish(state_msg)

        rospy.sleep(1.0/ArtHertz.NAVIGATOR)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('commander test completed')
