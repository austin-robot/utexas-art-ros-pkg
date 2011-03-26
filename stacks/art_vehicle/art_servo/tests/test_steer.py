#!/usr/bin/env python
#
# Description:  Unit test for ART steering driver
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
from art_msgs.msg import ArtVehicle
from art_msgs.msg import SteeringCommand
from art_msgs.msg import SteeringState

max_steer_degrees = ArtVehicle.max_steer_degrees

def log_steering_state(state):
    rospy.logdebug("steering angle, sensor: %.3f %.3f (time %.6f)",
                   state.angle, state.sensor, state.header.stamp.to_sec())

def log_steering_cmd(cmd):
    rospy.loginfo("sending steering angle: " + str(cmd.angle))

def test():
    topic = rospy.Publisher('steering/cmd', SteeringCommand)
    rospy.Subscriber('steering/state', SteeringState, log_steering_state)
    rospy.init_node('test_steering')

    rospy.loginfo('starting steering test')

    # initially, steering wheel is centered
    cmd = SteeringCommand()             # steering command msg
    cmd.request = SteeringCommand.Degrees
    cmd.angle = 0.0                     # last angle requested
    delta = 5.0                         # change requested per second

    while not rospy.is_shutdown():
        if cmd.angle <= -max_steer_degrees or cmd.angle >= max_steer_degrees:
            delta = -delta
        cmd.angle += delta
        log_steering_cmd(cmd)
        topic.publish(cmd)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('steering test completed')
