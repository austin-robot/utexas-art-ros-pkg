#!/usr/bin/env python
#
# Description:  Unit test for ART commander node
#
#   Copyright (C) 2010 Austin Robot Technology
#
#   License: Modified BSD Software License Agreement
#
#   Author: Jack O'Quin
#
#   $Id$

import roslib;
roslib.load_manifest('art_nav')

import rospy
from art_nav.msg import NavigatorCommand
from art_nav.msg import NavigatorState

last_order = None

def log_cmd(cmd):
    global last_order
    if not have_cmd:
        rospy.loginfo("first navigator command received")
        rospy.loginfo(str(cmd))
        have_cmd = True;
    last_order = cmd.order
    rospy.loginfo('order ' + str(cmd.behavior.value) + ' received')

def log_state(state_msg):
    #rospy.loginfo('publishing ' + str(state_msg))
    rospy.loginfo('publishing ' + str(state_msg.header.seq))

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

        log_state(state_msg)
        topic.publish(state_msg)

        rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('commander test completed')
