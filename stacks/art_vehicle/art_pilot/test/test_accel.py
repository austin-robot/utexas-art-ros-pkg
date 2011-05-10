#!/usr/bin/python
#
#  send pilot commands and update state
#
#   Copyright (C) 2011 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id: pilot_cmd.py 1321 2011-04-19 20:23:37Z jack.oquin $

PKG_NAME = 'art_pilot'

# ROS node setup
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

# ROS messages
from art_msgs.msg import ArtVehicle
from art_msgs.msg import CarDriveStamped
from art_msgs.msg import CarDrive
from art_msgs.msg import DriverState
from art_msgs.msg import Epsilon
from art_msgs.msg import Gear
from art_msgs.msg import PilotState

def clamp(minimum, value, maximum):
    "constrain value to the range [minimum .. maximum]"
    return max(minimum, min(value, maximum))

class PilotCommand():
    "ART pilot command interface."

    def __init__(self, maxspeed=6.0, minspeed=-3.0):
        "PilotCommand constructor"
        self.reconfigure(maxspeed, minspeed)
        self.pstate = PilotState()
        self.car_ctl = CarDrive()
        self.car_msg = CarDriveStamped()
        self.pub = rospy.Publisher('pilot/drive', CarDriveStamped)
        self.sub = rospy.Subscriber('pilot/state', PilotState,
                                    self.pilotCallback)

    def accelerate(self, dv):
        "accelerate dv meters/second^2"
        rospy.logdebug('acceleration: ' + str(dv))

        self.car_ctl.acceleration = dv

        # update speed
        dv *= 0.05                      # scale by cycle duration (dt)
        vabs = abs(self.car_ctl.speed)

        # never shift gears via acceleration, stop at zero
        if -dv > vabs:
            vabs = 0.0
        else:
            vabs += dv

        if self.car_ctl.gear.value == Gear.Drive:
            self.car_ctl.speed = vabs
        elif self.car_ctl.gear.value == Gear.Reverse:
            self.car_ctl.speed = -vabs
        else:                   # do nothing in Park
            self.car_ctl.speed = 0.0
            self.car_ctl.acceleration = 0.0

        # never exceeded forward or reverse speed limits
        self.car_ctl.speed = clamp(self.minspeed,
                                   self.car_ctl.speed,
                                   self.maxspeed)

    def command(self, speed, accel):
        "set pilot command parameters"
        self.car_ctl.speed = speed
        self.car_ctl.acceleration = accel

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
        #rospy.loginfo(str(pstate))
        # Base future commands on current state, not target.
        self.car_ctl = pstate.current

    def publish(self):
        "publish pilot command message"
        self.car_msg.header.stamp = rospy.Time.now()
        self.car_msg.control = self.car_ctl
        self.pub.publish(self.car_msg)

    def reconfigure(self, maxspeed, minspeed):
        "reconfigure forward and reverse speed limits"
        self.maxspeed = maxspeed
        self.minspeed = minspeed

    def shift(self, gear):
        "set gear request (only if stopped)"
        if self.is_stopped():
            self.car_ctl.gear.value = gear

    def steer(self, angle):
        "set wheel angle (radians)"
        # ensure maximum wheel angle never exceeded
        self.car_ctl.steering_angle = clamp(-ArtVehicle.max_steer_radians,
                                             angle,
                                             ArtVehicle.max_steer_radians)


def test(pilot):

    # list of tuples: (speed, acceleration, duration)
    requests = [(3.0, 20.0, 8.0),
                (6.0, 20.0, 8.0),
                (9.0, 20.0, 8.0),
                (6.0, 20.0, 8.0),
                (3.0, 20.0, 8.0),
                (0.0, 20.0, 4.0),
                (3.0, 0.5, 16.0),
                (6.0, 0.5, 16.0),
                (9.0, 0.5, 16.0),
                (6.0, 0.5, 16.0),
                (3.0, 0.5, 16.0),
                (0.0, 0.5, 16.0)]

    while not rospy.is_shutdown():

        if pilot.is_running():
            # start running the test

            if len(requests) == 0:
                # stop car after test completed
                vel = 0.0
                accel = 0.0
                duration = 0.0
            else:
                # get first tuple in the requests list
                (vel, accel, duration) = requests.pop(0)

            rospy.loginfo('requesting v: ' + str(vel)
                          + ' a: ' + str(accel)
                          + ' t: ' + str(duration))

            if pilot.is_stopped():
                pilot.shift(Gear.Drive)

            pilot.command(vel, accel)
            pilot.publish()

            if duration == 0.0:
                break           # test finished

            rospy.sleep(duration)

        else:
            # wait until pilot is running
            rospy.loginfo('waiting for pilot to run')
            rospy.sleep(1.0)

if __name__ == '__main__':
    rospy.init_node('pilot_cmd')
    rospy.loginfo('starting acceleration test')
    pilot = PilotCommand(10.0)
    try:
        test(pilot)
    except rospy.ROSInterruptException: pass

    rospy.loginfo('acceleration test completed')
