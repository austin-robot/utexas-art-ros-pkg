#!/usr/bin/python
#
#  send Ackermann tele-operation commands from an HKS racing controller
#
#   Copyright (C) 2011 Jack O'Quin, Joshua Bennett
#   Copyright (C) 2012 Jack O'Quin
#   License: Modified BSD Software License Agreement
#
# $Id: joy_teleop.py 1321 2011-04-19 20:23:37Z jack.oquin $

PKG_NAME = 'ackermann_hks'

# standard Python packages
import sys
import math
import Queue

import pilot_cmd                # ART pilot interface

# ROS node setup
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

# ROS messages
from art_msgs.msg import Gear
from sensor_msgs.msg import Joy

# dynamic parameter reconfiguration
from driver_base.msg import SensorLevels
from dynamic_reconfigure.server import Server as ReconfigureServer
import ackermann_hks.cfg.HKSConfig as Config

def clamp(minimum, value, maximum):
    "constrain value to the range [minimum .. maximum]"
    return max(minimum, min(value, maximum))

class JoyNode():
    "Vehicle joystick tele-operation node."

    def __init__(self):
        "JoyNode constructor"

        # estop control
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
	self.counter = 0
	self.throttleQueue = Queue.Queue()
	self.cruiseSwitch = 1		# Press down Joystick
	self.cruise = False
	
        # initialize ROS topics
        rospy.init_node('hks_teleop')
        self.pilot = pilot_cmd.PilotCommand()
        self.reconf_server = ReconfigureServer(Config, self.reconfigure)
        self.joy = rospy.Subscriber('joy', Joy, self.joyCallback)


    def joyCallback(self, joy):
        "invoked every time a joystick message arrives"
        rospy.logdebug('joystick input:\n' + str(joy))

        # handle E-stop button
        if joy.buttons[self.estop]:
            rospy.logwarn('emergency stop')
            self.pilot.halt()           # halt car using pilot

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

	if joy.buttons[self.cruiseSwitch]:
            if self.cruise:
                self.cruise = False
            else:
                self.cruise = True
                self.pilot.pstate.target.speed = self.pilot.pstate.current.speed

        # set steering angle
	self.setAngle(joy.axes[self.steer])

	if self.cruise:
		self.pilot.accelerate(0)

	else:
		# adjust speed -- the brake and throttle controls both
		# return 1.0 when released, -1.0 when completely on
		# Convert the -1 to 1 to 0 to 1 in increments of .01
		br = (-joy.axes[self.brake] + 1) / 2
		th = (-joy.axes[self.throttle] + 1) / 2
		rospy.logdebug('joystick brake, throttle: '
		               + str(br) + ', ' + str(th))

		if self.counter < 1:
			self.counter = 1
			self.throttleQueue.put(th)
		else:
			initial_th = self.throttleQueue.get()
			self.throttleQueue.put(th)

		# initially the ROS /joy topic sends zeroes for each axis
		# until a button press occurs
	
	
		if self.brake_start:
		    if br == 0.5:
		        br = 0
		    else:
		        self.brake_start = False

		if self.throttle_start:
		    if th == 0.5:
		        th = 0
		    else:
		        self.throttle_start = False

		# set dv according to throttle or brake's current state.
		if br > 0:
			dv = -br * 3

		elif th == 0 and self.pilot.pstate.current.speed > 0:
			dv = -.1

		elif th > 0:
			if initial_th < th:
		   		dv = th
		    	elif initial_th > th:
				dv = -.1 
			elif initial_th == th and self.pilot.pstate.current.acceleration >= 0:
				dv = th
			elif initial_th == th and self.pilot.pstate.current.acceleration < 0:
				dv = -.1
			else:
				dv = 0
		else:
			dv = 0

		# set acceleration and speed from brake and throttle controls
		self.pilot.accelerate(dv*10)

        if self.nav.is_suspended(): # OK to issue command?
            self.pilot.publish()
        else:
            rospy.logdebug('car running autonomously (command ignored)')

    def reconfigure(self, config, level):
        "Dynamic reconfigure server callback."
        rospy.loginfo('Reconfigure callback, level ' + str(level))
        rospy.loginfo('New config ' + str(config))
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
        #turn = math.pow(turn, 3) * self.config.max_steering
        #turn = math.tan(turn) * self.config.max_steering
	if self.pilot.pstate.current.speed == 0:
		percentage = 1
	# Expirimental steering over speed preference
	#if self.pilot.pstate.current.speed <= 5:
	#	percentage = 1
	
	else:
		#percentage = -0.2738*(math.log(math.fabs(self.pilot.pstate.current.speed))) + 0.6937
		percentage = (-math.atan2(math.fabs(self.pilot.pstate.current.speed)-3, 1) / 1.5) + 1
	turn = turn * percentage * self.config.max_steering

        # ensure maximum wheel angle never exceeded
        self.pilot.steer(turn)

	def maxFinder(self):
		if self.pilot.pstate.current.speed > self.config['limit_forward']:
			return self.pilot.state.current
		return self.config['limit_forward']

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
