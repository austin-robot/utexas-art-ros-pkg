#!/usr/bin/python
#
#  send tele-operation commands to pilot from a joystick
#
#   Copyright (C) 2009 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'art_teleop'

# standard Python packages
import sys
import math

# ROS node setup
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

# ROS messages
from art_msgs.msg import ArtVehicle
from art_msgs.msg import CarAccel
from art_msgs.msg import CarControl2
from joy.msg import Joy

# global constants
max_speed = 6.0                         # meters/second
max_speed_reverse = -3.0                # meters/second

class JoyNode():
    "Vehicle joystick tele-operation node."

    def __init__(self):
        "JoyNode constructor"
        self.car_ctl = CarControl2()
        self.car_msg = CarAccel()

        self.steer = 3                  # steering axis (right)
        self.speed = 1                  # speed axis (left)
        self.direction = 1.0            # gear direction (drive)

        rospy.init_node('joy_teleop')
        self.topic = rospy.Publisher('pilot/accel', CarAccel)
        self.joy = rospy.Subscriber('joy', Joy, self.joyCallback)

        self.car_msg.header.stamp = rospy.Time.now()
        self.car_msg.control = self.car_ctl
        self.topic.publish(self.car_msg)

    def joyCallback(self, joy):
        "invoked every time a joystick message arrives"
        rospy.logdebug('joystick input:\n' + str(joy))

        # handle various buttons (when appropriate)
        if joy.buttons[0] and self.car_ctl.goal_velocity == 0.0:
            if self.direction != -1.0:
                self.direction = -1.0       # shift to reverse
                self.car_ctl.gear = CarControl2.Reverse
                rospy.loginfo('shifting to reverse')

        elif joy.buttons[1]:
            if self.car_ctl.goal_velocity != 0.0:
                rospy.logwarn('emergency stop')
            self.car_ctl.goal_velocity = 0.0     # stop car immediately

        elif joy.buttons[2] and self.car_ctl.goal_velocity == 0.0:
            if self.direction != 1.0:
                self.direction = 1.0        # shift to drive
                self.car_ctl.gear = CarControl2.Drive
                rospy.loginfo('shifting to drive')

        elif joy.buttons[10]:               # select left joystick
            self.steer = 0                  #  steering axis
            self.speed = 1                  #  speed axis
            rospy.loginfo('left joystick selected')

        elif joy.buttons[11]:               # select right joystick
            self.steer = 3                  #  steering axis
            self.speed = 2                  #  speed axis
            rospy.loginfo('right joystick selected')

        else:                               # normal analog control
            # set steering angle
            self.setAngle(joy.axes[self.steer])
            # adjust speed
            self.adjustSpeed(joy.axes[self.speed])

        self.car_msg.header.stamp = rospy.Time.now()
        self.car_msg.control = self.car_ctl
        self.topic.publish(self.car_msg)

    def adjustSpeed(self, dv):
        "accelerate dv meters/second/second"

        dv *= 0.05                      # scale by cycle duration (dt)
        self.car_ctl.acceleration = dv

        # take absolute value of goal_velocity
        vabs = self.car_ctl.goal_velocity * self.direction

        # never shift gears via speed controller, stop at zero
        if -dv > vabs:
            vabs = 0.0
        else:
            vabs += dv

        self.car_ctl.goal_velocity = vabs * self.direction

        # ensure forward and reverse speed limits never exceeded
        if self.car_ctl.goal_velocity > max_speed:
            self.car_ctl.goal_velocity = max_speed
        elif self.car_ctl.goal_velocity < max_speed_reverse:
            self.car_ctl.goal_velocity = max_speed_reverse

    def setAngle(self, turn):
        "set wheel angle"

        # use various functions with domain [-1..1] to improve
        # sensitivity while retaining sign

        #self.car_ctl.steering_angle = turn * turn * turn * ArtVehicle.max_steer_radians
        self.car_ctl.steering_angle = math.pow(turn, 5) * ArtVehicle.max_steer_radians
        #self.car_ctl.steering_angle = math.tan(turn) * ArtVehicle.max_steer_radians

        # ensure maximum wheel angle never exceeded
        if self.car_ctl.steering_angle > ArtVehicle.max_steer_radians:
            self.car_ctl.steering_angle = ArtVehicle.max_steer_radians
        elif self.car_ctl.steering_angle < -ArtVehicle.max_steer_radians:
            self.car_ctl.steering_angle = -ArtVehicle.max_steer_radians

joynode = None

def main():
    global joynode
    joynode = JoyNode()
    rospy.loginfo('joystick vehicle controller starting')
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
    rospy.loginfo('joystick vehicle controller finished')

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
