#!/usr/bin/env python
#
# Description:  Unit test for ART brake driver
#
#   Copyright (C) 2009 Austin Robot Technology
#
#   License: Modified BSD Software License Agreement
#
# $Id$
#

import roslib;
roslib.load_manifest('art_servo')

import rospy
from art_msgs.msg import BrakeCommand
from art_msgs.msg import BrakeState

def log_brake_state(state):
    rospy.logdebug("brake position: %.3f  (time %.6f)",
                   state.position, state.header.stamp.to_sec())
    rospy.logdebug("brake pot, enc, press = (%.3f, %.3f, %.3f)  (time %.6f)",
                  state.potentiometer, state.encoder, state.pressure,
                  state.header.stamp.to_sec())

def log_brake_cmd(cmd):
    rospy.loginfo("sending brake command: " + str(cmd.request)
                  + ", " + str(cmd.position))

def test():
    topic = rospy.Publisher('brake/cmd', BrakeCommand)
    rospy.Subscriber('brake/state', BrakeState, log_brake_state)
    rospy.init_node('test_brake')

    rospy.loginfo('starting brake test')

    # initially apply full brake
    cmd = BrakeCommand()                # brake command msg
    cmd.request = BrakeCommand.Absolute
    npos = 1.0                          # next position to request
    delta = 0.5                         # change requested per cycle

    while not rospy.is_shutdown():

        # check for limits reached
        if npos <= 0.0:
            npos = 0.0
            delta = -delta              # change direction
        elif npos >= 1.0:
            npos = 1.0
            delta = -delta              # change direction

        # format brake command and publish it
        cmd.header.stamp = rospy.Time.now()
        cmd.position = npos
        log_brake_cmd(cmd)
        topic.publish(cmd)
        npos += delta                   # update next position
        rospy.sleep(8.0)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('brake test completed')
