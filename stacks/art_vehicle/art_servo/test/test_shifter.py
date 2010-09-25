#!/usr/bin/env python
#
# Description:  Unit test for ART shifter interface to ioadr driver
#
#   Copyright (C) 2005, 2007, 2009 Austin Robot Technology
#
#   License: Modified BSD Software License Agreement
#
# $Id$
#

import roslib;
roslib.load_manifest('art_servo')

import rospy
from art_msgs.msg import Shifter

def log_shifter_state(state):
    rospy.logdebug("shifter gear: %u, relays: %02x (time %.6f)",
                   state.gear, state.relays, state.header.stamp.to_sec())

def log_shifter_cmd(cmd):
    #rospy.loginfo(str(cmd))
    rospy.loginfo("sending shifter command: " + str(cmd.gear))

def test():
    topic = rospy.Publisher('shifter/cmd', Shifter)
    rospy.Subscriber('shifter/state', Shifter, log_shifter_state)
    rospy.init_node('test_shifter')

    rospy.loginfo('starting shifter test')

    # initially reset shifter
    cmd = Shifter()                     # shifter message
    cmd.gear = Shifter.Park
    reset_cmd = Shifter()               # reset shifter message
    reset_cmd.gear = Shifter.Reset

    while not rospy.is_shutdown():
        if cmd.gear > Shifter.Drive:
            cmd.gear = Shifter.Park

        # shift to next gear
        topic.publish(cmd)
        log_shifter_cmd(cmd)
        cmd.gear += 1
        rospy.sleep(1.0)                # only change once a second

        # reset shifter
        topic.publish(reset_cmd)
        log_shifter_cmd(reset_cmd)
        rospy.sleep(1.0)                # only change once a second

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('shifter test completed')
