#!/usr/bin/python
#
# Qt python script to send tele-operation commands to pilot from a joystick
#
#   Copyright (C) 2009 Austin Robot Technology
#
#   License: Modified BSD Software License Agreement
#
#   Author: Jack O'Quin
#
# $Id$

PKG_NAME = 'art_nav'

import sys
import os
import roslib;
roslib.load_manifest(PKG_NAME)

import rospy
from art_nav.msg import CarCommand
from joy.msg import Joy

class JoyNode():
    "Vehicle joystick tele-operation node."

    def __init__(self):
        "JoyNode constructor"
        self.carcmd = rospy.Publisher('pilot/cmd', CarCommand)
        self.car_msg = CarCommand()
        self.car_msg.header.stamp = rospy.Time.now()
        self.car_msg.velocity = 0.0
        self.car_msg.angle = 0.0
        self.carcmd.publish(self.car_msg)
        rospy.Subscriber('joy', Joy, self.joyCallback)

    def joyCallback(self, joy):
        "invoked every time a joystick message arrives"
        rospy.logdebug('joystick input\n' + str(joy))

        self.updateStatus()

    def updateStatus(self):
        rospy.loginfo('speed: ' + str(self.car_msg.velocity)
                       + ' m/s, angle: ' + str(self.car_msg.angle) + ' deg')

    def adjustCarCmd(self, v, a):
        self.car_msg.header.stamp = rospy.Time.now()
        self.car_msg.velocity += v
        self.car_msg.angle += a
        if self.car_msg.angle > 29.0:
            self.car_msg.angle = 29.0
        if self.car_msg.angle < -29.0:
            self.car_msg.angle = -29.0
        self.carcmd.publish(self.car_msg)
        self.updateStatusBar()

    def center_wheel(self):
        "center steering wheel"
        self.adjustCarCmd(0.0, -self.car_msg.angle)

    def go_left(self):
        "steer left"
        self.adjustCarCmd(0.0, 1.0)

    def go_left_more(self):
        "steer more to left"
        self.adjustCarCmd(0.0, 4.0)

    def go_left_less(self):
        "steer a little to left"
        self.adjustCarCmd(0.0, 0.25)

    def go_right(self):
        "steer right"
        self.adjustCarCmd(0.0, -1.0)

    def go_right_more(self):
        "steer more to right"
        self.adjustCarCmd(0.0, -4.0)

    def go_right_less(self):
        "steer far to right"
        self.adjustCarCmd(0.0, -0.25)

    def slow_down(self):
        "go one m/s slower"
        self.adjustCarCmd(-1.0, 0.0)    # one m/s slower

    def speed_up(self):
        "go one m/s faster"
        self.adjustCarCmd(1.0, 0.0)     # one m/s faster

    def stop_car(self):
        "stop car immediately"
        self.adjustCarCmd(-self.car_msg.velocity, 0.0)

def main():

    rospy.init_node('joy_teleop')
    joynode = JoyNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
