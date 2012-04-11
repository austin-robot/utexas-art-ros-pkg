#!/usr/bin/python
#
#  send pilot commands and update state
#
#   Copyright (C) 2011 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'ackermann_hks'

# ROS node setup
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

# ROS messages
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
#from art_msgs.msg import DriverState
#from art_msgs.msg import Epsilon
#from art_msgs.msg import Gear
#from art_msgs.msg import PilotState

def clamp(minimum, value, maximum):
    "constrain value to the range [minimum .. maximum]"
    return max(minimum, min(value, maximum))

class PilotCommand():
    "Ackermann steering command interface."

    def __init__(self, limit_forward=6.0, limit_reverse=3.0):
        "PilotCommand constructor"
        self.reconfigure(limit_forward, limit_reverse)
        self.pstate = PilotState()
        self.car_ctl = AckermannDrive()
        self.car_msg = AckermannDriveStamped()
        self.pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped)
        #self.sub = rospy.Subscriber('pilot/state', PilotState,
        #                            self.pilotCallback)

    def accelerate(self, dv):
        "accelerate dv meters/second^2"
        rospy.logdebug('acceleration: ' + str(dv))

        self.car_ctl.acceleration = abs(dv)

        # update speed
        dv *= 0.05                      # scale by cycle duration (dt)
        vabs = abs(self.car_ctl.speed)

        # never shift gears via acceleration, stop at zero
        if -dv > vabs:
            vabs = 0.0
        else:
            vabs += dv

        # never exceeded forward or reverse speed limits
        if self.car_ctl.gear.value == Gear.Drive:
            if vabs > self.limit_forward:
                vabs = self.limit_forward
        elif self.car_ctl.gear.value == Gear.Reverse:
            if vabs > self.limit_reverse:
                vabs = self.limit_reverse
        else:                   # do nothing in Park or Neutral
            vabs = 0.0
            self.car_ctl.acceleration = 0.0

        self.car_ctl.speed = vabs

    def halt(self):
        "halt car immediately"
        self.car_ctl.speed = 0.0
        self.car_ctl.acceleration = 0.0

    def is_running(self):
        "return True if pilot node is RUNNING"
        # TODO check that time stamp is not too old
        return (self.pstate.pilot.state == DriverState.RUNNING)

    def is_stopped(self):
        "return True if vehicle is stopped"
        return (abs(self.car_ctl.speed) < Epsilon.speed)

    def pilotCallback(self, pstate):
        "handle pilot state message"
        self.pstate = pstate
        # Base future commands on current state, not target.
        self.car_ctl = pstate.current

    def publish(self):
        "publish pilot command message"
        self.car_msg.header.stamp = rospy.Time.now()
        self.car_msg.control = self.car_ctl
        self.pub.publish(self.car_msg)

    def reconfigure(self, limit_forward, limit_reverse):
        "reconfigure forward and reverse speed limits"
        self.limit_forward = limit_forward
        self.limit_reverse = abs(limit_reverse)

    def shift(self, gear):
        "set gear request (only if stopped)"
        if self.is_stopped():
            self.car_ctl.gear.value = gear

    def steer(self, angle):
        "set wheel angle (radians)"
        # ensure maximum wheel angle never exceeded
        self.car_ctl.steering_angle = clamp(self.config.min_steering,
                                            angle,
                                            self.config.max_steering)


# standalone test of package
if __name__ == '__main__':

    rospy.init_node('pilot_cmd')

    # make an instance
    pilot = PilotCommand()

    # run the program
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
