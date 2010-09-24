#!/usr/bin/env python
#
# Description:  Unit test for ART pilot node
#
#   Copyright (C) 2009 Austin Robot Technology
#
#   License: Modified BSD Software License Agreement
#
#   Author: Jack O'Quin
#
#   $Id$

import roslib;
roslib.load_manifest('art_nav')
roslib.load_manifest('nav_msgs')

import rospy
from art_msgs.msg import CarCommand
from art_msgs.msg import CarControl
from nav_msgs.msg import Odometry

have_odom = False;
maxSpeed = 10.0                         # top speed (meters/second)
maxAngle = 29.0                         # maximum steering angle

def log_odometry(odom):
    global have_odom
    # rospy.loginfo(str(odom))          # simple but verbose approach
    if not have_odom:
        rospy.loginfo("first odometry message received")
        have_odom = True;

def log_pilot_cmd(cmd):
    #rospy.loginfo(str(cmd))
    rospy.loginfo("sending pilot velocity: " + str(cmd.velocity)
                  + ", angle: " + str(cmd.angle))

def test():
    topic = rospy.Publisher('pilot/cmd', CarCommand)
    rospy.Subscriber('odom', Odometry, log_odometry)
    rospy.init_node('test_pilot')

    rospy.loginfo('starting pilot test')

    ctl = CarControl()
    cmd_msg = CarCommand()              # pilot command msg with header
    aDelta = 1.0                        # steering angle change (deg/sec)
    vDelta = 1.0                        # velocity change

    while not rospy.is_shutdown():

        if have_odom:                   # wait until odometry received

            ctl.velocity += vDelta
            if ctl.velocity < 0.0 or ctl.velocity > maxSpeed:
                vDelta = -vDelta
                ctl.velocity += vDelta

            ctl.angle += aDelta
            if ctl.angle < -maxAngle or ctl.angle > maxAngle:
                aDelta = -aDelta
                ctl.angle += aDelta

            log_pilot_cmd(ctl)

            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.control = ctl
            topic.publish(cmd_msg)

        rospy.sleep(3.0)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('pilot test completed')
