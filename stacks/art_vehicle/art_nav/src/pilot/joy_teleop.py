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

max_wheel_angle = 29.0                  # degrees
max_speed = 10.0                        # meters/second
max_speed_reverse = -3.0                # meters/second

class JoyNode():
    "Vehicle joystick tele-operation node."

    def __init__(self):
        "JoyNode constructor"
        self.car_msg = CarCommand()
        self.car_msg.velocity = 0.0
        self.car_msg.angle = 0.0

        self.carcmd = rospy.Publisher('pilot/cmd', CarCommand)
        self.joy = rospy.Subscriber('joy', Joy, self.joyCallback)
        rospy.init_node('joy_teleop')

        self.car_msg.header.stamp = rospy.Time.now()
        self.carcmd.publish(self.car_msg)

    def joyCallback(self, joy):
        "invoked every time a joystick message arrives"
        rospy.logdebug('joystick input\n' + str(joy))

        # TODO: add buttons for gear shifter
        if joy.buttons[1]:
            # stop car immediately
            self.car_msg.velocity = 0.0
        else:
            # set steering angle
            self.setAngle(joy.axes[3])
            # adjust speed
            self.adjustSpeed(joy.axes[2])

        self.car_msg.header.stamp = rospy.Time.now()
        self.carcmd.publish(self.car_msg)

    def adjustSpeed(self, dv):
        "adjust speed by dv meters/second"

        # never shift gears via speed control
        if -dv > self.car_msg.velocity:
            self.car_msg.velocity = 0.0
        else:
            self.car_msg.velocity += dv

        # ensure speed limits never exceeded
        if self.car_msg.velocity > max_speed:
            self.car_msg.velocity = max_speed
        if self.car_msg.velocity < max_speed_reverse:
            self.car_msg.velocity = max_speed_reverse

    def setAngle(self, turn):
        "set wheel angle"
        # use square of turn value to improve sensitivity
        self.car_msg.angle = turn * turn * max_wheel_angle

        # ensure maximum wheel angle never exceeded
        if self.car_msg.angle > max_wheel_angle:
            self.car_msg.angle = max_wheel_angle
        if self.car_msg.angle < -max_wheel_angle:
            self.car_msg.angle = -max_wheel_angle

def main():

    joynode = JoyNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
