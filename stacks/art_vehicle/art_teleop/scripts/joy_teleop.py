#!/usr/bin/python
#
#  send tele-operation commands to pilot from an HKS racing controller
#
#   Copyright (C) 2011 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'art_teleop'

# standard Python packages
import sys
import math

import pilot_cmd                # ART pilot interface
import nav_estop                # ART navigator E-stop package

# ROS node setup
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

# ROS messages
from art_msgs.msg import ArtVehicle
from art_msgs.msg import CarControl2
from joy.msg import Joy

# global constants
max_speed = 6.0                         # meters/second
max_speed_reverse = -3.0                # meters/second

def clamp(minimum, value, maximum):
    "constrain value to the range [minimum .. maximum]"
    return max(minimum, min(value, maximum))

class JoyNode():
    "Vehicle joystick tele-operation node."

    def __init__(self):
        "JoyNode constructor"

        # estop controls
        self.run = 3                    # start autonomous run
        self.suspend = 12               # suspend autonomous running
        self.estop = 13                 # emergency stop

        # tele-operation controls
        self.steer = 0                  # steering axis (wheel)
        self.drive = 4                  # shift to Drive (^)
        self.reverse = 6                # shift to Reverse (v)
        self.throttle = 18              # throttle axis (X)
        self.throttle_start = True
        self.brake = 19                 # brake axis (square)
        self.brake_start = True

        # initialize ROS topics
        rospy.init_node('joy_teleop')
        self.use_navigator = rospy.get_param('~use_navigator', True)
        rospy.loginfo('use navigator = ' + str(self.use_navigator))
        self.est = nav_estop.EstopNavigator(self.use_navigator)
        self.pilot = pilot_cmd.PilotCommand(max_speed, max_speed_reverse)
        self.joy = rospy.Subscriber('joy', Joy, self.joyCallback)

    def joyCallback(self, joy):
        "invoked every time a joystick message arrives"
        rospy.logdebug('joystick input:\n' + str(joy))

        # handle various buttons (when appropriate)
        if joy.buttons[self.reverse]:
            self.pilot.shift(CarControl2.Reverse)
            rospy.loginfo('shifting to reverse')

        elif joy.buttons[self.estop]:
            rospy.logwarn('emergency stop')
            if self.use_navigator:
                self.est.pause()
            else:
                self.pilot.accelerate(-9.8)
                

        elif joy.buttons[self.run] and self.use_navigator:
            rospy.logwarn('start autonomous operation')
            self.est.run()

        elif joy.buttons[self.suspend] and self.use_navigator:
            rospy.logwarn('suspend autonomous operation')
            self.est.suspend()

        elif joy.buttons[self.drive]:
            self.pilot.shift(CarControl2.Drive)
            rospy.loginfo('shifting to drive')

        else:                           # normal "analog" control
            # set steering angle
            self.setAngle(joy.axes[self.steer])

            # adjust speed -- the brake and throttle controls both
            # return 1.0 when released, -1.0 when completely on
            br = joy.axes[self.brake]
            th = joy.axes[self.throttle]
            rospy.logdebug('joystick brake, throttle: '
                           + str(br) + ', ' + str(th))

            # initially the ROS /joy topic sends zeroes until a button
            # press occurs
            if self.brake_start and br == 0.0:
                br = 1.0
            else:
                self.brake_start = False
            if self.throttle_start and th == 0.0:
                th = 1.0
            else:
                self.throttle_start = False

            # set acceleration from brake and throttle controls
            dv = 0.0
            if br < 1.0:
                dv = br - 1.0
            elif th < 1.0:
                dv = 1.0 - th
            self.pilot.accelerate(dv)

        if self.est.is_suspended(): # OK to issue command?
            self.pilot.publish()
        else:
            rospy.logwarn('car running autonomously (command ignored)')
            
    def setAngle(self, turn):
        "set wheel angle"

        # Try various functions with domain [-1..1] to improve
        # sensitivity while retaining sign. At higher speeds, it may
        # make sense to limit the range, avoiding too-sharp turns and
        # providing better control sensitivity.
        turn = math.pow(turn, 3) * ArtVehicle.max_steer_radians
        #turn = math.tan(turn) * ArtVehicle.max_steer_radians

        # ensure maximum wheel angle never exceeded
        self.pilot.steer(turn)

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
