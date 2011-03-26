#!/usr/bin/python
#
#  send pilot commands and update state
#
#   Copyright (C) 2011 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'art_teleop'

import math

# ROS node setup
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

# ROS messages
from art_msgs.msg import ArtVehicle
from art_msgs.msg import CarAccel
from art_msgs.msg import CarControl2
from art_msgs.msg import DriverState
from art_msgs.msg import PilotState

def clamp(minimum, value, maximum):
    "constrain value to the range [minimum .. maximum]"
    return max(minimum, min(value, maximum))

class PilotCommand():
    "ART pilot command interface."

    def __init__(self, maxspeed=6.0, minspeed=-3.0):
        "PilotCommand constructor"
        self.maxspeed = maxspeed
        self.minspeed = minspeed

        self.pstate = PilotState()
        self.car_ctl = CarControl2()
        self.car_msg = CarAccel()
        self.pub = rospy.Publisher('pilot/accel', CarAccel)
        self.sub = rospy.Subscriber('pilot/state', PilotState, self.pilotCallback)

        self.publish()

    def accelerate(self, dv):
        "accelerate dv meters/second/second"
        rospy.logdebug('acceleration: ' + str(dv))

        self.car_ctl.acceleration = dv

        # update goal_velocity
        dv *= 0.05                      # scale by cycle duration (dt)
        vabs = math.fabs(self.car_ctl.goal_velocity)

        # never shift gears via acceleration, stop at zero
        if -dv > vabs:
            vabs = 0.0
        else:
            vabs += dv

        if self.car_ctl.gear == CarControl2.Drive:
            self.car_ctl.goal_velocity = vabs
        elif self.car_ctl.gear == CarControl2.Reverse:
            self.car_ctl.goal_velocity = -vabs
        else:                   # do nothing in Park
            self.car_ctl.goal_velocity = 0.0
            self.car_ctl.acceleration = 0.0

        # never exceeded forward or reverse speed limits
        self.car_ctl.goal_velocity = clamp(self.minspeed,
                                           self.car_ctl.goal_velocity,
                                           self.maxspeed)

    def is_running(self):
        "return True if pilot node is RUNNING"
        return (self.pstate == DriverState.RUNNING)

    def pilotCallback(self, pstate):
        "handle pilot state message"
        self.pstate = pstate
        # Base future commands on latest target instead of current.
        # That may provide smoother transitions: pilot is working on
        # that request already and might be finished before a new one
        # arrives.
        self.car_ctl = pstate.target

    def publish(self):
        "publish pilot command message"
        self.car_msg.header.stamp = rospy.Time.now()
        self.car_msg.control = self.car_ctl
        self.pub.publish(self.car_msg)

    def shift(self, gear):
        "set gear request"
        self.car_ctl.gear = gear

    def steer(self, angle):
        "set wheel angle (radians)"
        # ensure maximum wheel angle never exceeded
        self.car_ctl.steering_angle = clamp(-ArtVehicle.max_steer_radians,
                                             angle,
                                             ArtVehicle.max_steer_radians)


# standalone test of package
if __name__ == '__main__':

    rospy.init_node('pilot_cmd')

    # make an instance
    pilot = PilotCommand()

    # run the program
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
