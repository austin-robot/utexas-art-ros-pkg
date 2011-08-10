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
import Queue

import pilot_cmd                # ART pilot interface
import nav_estop                # ART navigator E-stop package

# ROS node setup
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

# ROS messages
from art_msgs.msg import ArtVehicle
from art_msgs.msg import Gear
from art_msgs.msg import SteeringState

# In ROS Electric, the Joy message gained a header and moved to
# sensor_msgs.  Use that if it's available.
try:
    from sensor_msgs.msg import Joy
except ImportError:
    from joy.msg import Joy

# dynamic parameter reconfiguration
from driver_base.msg import SensorLevels
from dynamic_reconfigure.server import Server as ReconfigureServer
import art_teleop.cfg.JoyConfig as JoyConfig

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
        self.park = 7                   # shift to Park
        self.throttle = 18              # throttle axis (X)
        self.throttle_start = True
        self.brake = 19                 # brake axis (square)
        self.brake_start = True
	self.steeringQueue = Queue.Queue()
	self.counter = 0

        # initialize ROS topics
        rospy.init_node('jaime_teleop')
        self.pilot = pilot_cmd.PilotCommand()
	self.steering = SteeringState()
        self.reconf_server = ReconfigureServer(JoyConfig, self.reconfigure)
        self.joy = rospy.Subscriber('joy', Joy, self.joyCallback)

    def joyCallback(self, joy):
        "invoked every time a joystick message arrives"
        rospy.logdebug('joystick input:\n' + str(joy))

        # handle E-stop buttons
        if joy.buttons[self.estop]:
            rospy.logwarn('emergency stop')
            if self.use_navigator:
                self.nav.pause()        # tell navigator to pause
            else:
                self.pilot.halt()       # halt car using pilot
        elif joy.buttons[self.suspend] and self.use_navigator:
            rospy.logwarn('suspend autonomous operation')
            self.nav.suspend()
        elif joy.buttons[self.run] and self.use_navigator:
            rospy.logwarn('start autonomous operation')
            self.nav.run()

        # handle shifter buttons
        if joy.buttons[self.drive]:
            self.pilot.shift(Gear.Drive)
            rospy.loginfo('shifting to drive')
        elif joy.buttons[self.reverse]:
            self.pilot.shift(Gear.Reverse)
            rospy.loginfo('shifting to reverse')
        elif joy.buttons[self.park]:
            self.pilot.shift(Gear.Park)
            rospy.loginfo('shifting to park')

        # set steering angle
        self.setAngle(joy.axes[self.steer])

        # adjust speed -- the brake and throttle controls both
        # return 1.0 when released, -1.0 when completely on
        br = joy.axes[self.brake]
        th = joy.axes[self.throttle]
        rospy.logdebug('joystick brake, throttle: '
                       + str(br) + ', ' + str(th))

        # initially the ROS /joy topic sends zeroes for each axis
        # until a button press occurs
        if self.brake_start:
            if br == 0.0:
                br = 1.0
            else:
                self.brake_start = False

        if self.throttle_start:
            if th == 0.0:
                th = 1.0
            else:
                self.throttle_start = False

        # set acceleration from brake and throttle controls
        dv = 0.0
        if br < 1.0:
            dv = br - 1.0
        elif th < 1.0:
            dv = 1.0 - th
        self.pilot.accelerate(dv * self.config['max_accel'])

        if self.nav.is_suspended(): # OK to issue command?
            self.pilot.publish()
        else:
            rospy.logdebug('car running autonomously (command ignored)')

    def reconfigure(self, config, level):
        "Dynamic reconfigure server callback."
        rospy.loginfo('Reconfigure callback, level ' + str(level))
        rospy.loginfo('New config ' + str(config))

        if level & SensorLevels.RECONFIGURE_CLOSE:
            # create new EstopNavigator and PilotCommand instances 
            self.use_navigator = config['use_navigator']
            rospy.loginfo('use navigator = ' + str(self.use_navigator))

            self.nav = nav_estop.EstopNavigator(self.use_navigator)

        self.pilot.reconfigure(config['limit_forward'],
                               config['limit_reverse'])

        self.config = config
        return config

    def setAngle(self, turn):
        "set wheel angle"

        # Try various functions with domain [-1..1] to improve
        # sensitivity while retaining sign. At higher speeds, it may
        # make sense to limit the range, avoiding too-sharp turns and
        # providing better control sensitivity.
	#turn = math.pow(turn, 3) * ArtVehicle.max_steer_radians
	
	#enqueues first 5 data points based on the first movements on the wheel
	if(self.counter < 5):
		self.counter +=1
		self.steeringQueue.put(self.steering.angle/180)
	else:
		#dequeue oldest value
		oldVal = self.steeringQueue.get()
		#get rate of change of angle by subtracting final(turn) - initial(oldVal) and dividing by .25s which is time it takes to get 5 values
		currentRate = (self.steering.angle/180 - oldVal)/.25
		limit = 2.0361 * math.pow(.72898, self.pilot.pstate.current.speed)
		rospy.logwarn(str(currentRate) + ' ' + str(limit))
		#if currentRate is above limit, restrict currentRate to the limit at the target velocity, else if it's at or below limit don't do anything
		#get new value of turn by using equation below:
		#(final angle - initial angle)/change time = limit; angle final = limit * change in time + angle inital
		if(math.fabs(currentRate > limit)):
			rospy.logwarn('Success')
			turn = limit*.25 + oldVal
		#put new steering angle into queue

		self.steeringQueue.put(turn)


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
