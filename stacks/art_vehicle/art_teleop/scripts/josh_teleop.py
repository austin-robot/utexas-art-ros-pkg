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
from art_msgs.msg import Gear
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
	self.lowincrease_max = 11       # Increase max .5 (R1)
	self.highincrease_max = 9       # Increase max 2 (R2)
	self.lowdecrease_max = 10	# Decrease max .5 (L1)
	self.highdecrease_max =	8	# Decrease max 2 (L2)
	

        # initialize ROS topics
        rospy.init_node('josh_teleop')
        self.pilot = pilot_cmd.PilotCommand()
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

	# handle max increases/decreases
	if joy.buttons[self.lowincrease_max]:
		self.config['limit_forward'] += .5
	if joy.buttons[self.highincrease_max]:
		self.config['limit_forward'] += 2
	if joy.buttons[self.lowdecrease_max]:
		self.config['limit_forward'] -= .5
	if joy.buttons[self.highdecrease_max]:
		self.config['limit_forward'] -= 2

        # set steering angle
	self.setAngle(joy.axes[self.steer])

        # adjust speed -- the brake and throttle controls both
        # return 1.0 when released, -1.0 when completely on
	# Convert the -1 to 1 to 0 to 1 in increments of .01
        br = (-joy.axes[self.brake] + 1) / 2
        th = (-joy.axes[self.throttle] + 1) / 2
        rospy.logdebug('joystick brake, throttle: '
                       + str(br) + ', ' + str(th))

        # initially the ROS /joy topic sends zeroes for each axis
        # until a button press occurs
        dv = 0
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

	if br > 0:
		dv = -br * 3

	elif th == 0 and self.pilot.pstate.current.speed > 0:
		dv = -.1
		# This error makes the script work, although I'm not sure why.
		#self.pilot.pstate.plan.goal_acceleration = -.2

        elif th > 0:
		if self.pilot.pstate.current.speed < self.config['limit_forward']*th:
           		dv = self.config['limit_forward']*th - self.pilot.pstate.current.speed
			if dv > 1:
				dv = 1 + math.pow(dv/self.config['limit_forward'], 2) # varies from 1 to 2
			else:
				dv = math.pow(dv, 2) # varies from 0 to 1
            	elif self.pilot.pstate.current.speed > self.config['limit_forward']*th:
			dv = -.1
                else:
                    dv = 0
	else:
		dv = 0

        # set acceleration and speed from brake and throttle controls
	self.pilot.car_ctl.acceleration = dv * 10
	if self.pilot.car_ctl.gear.value == Gear.Drive:
            	self.pilot.car_ctl.speed = self.config['limit_forward']*th
        elif self.pilot.car_ctl.gear.value == Gear.Reverse:
            	self.pilot.car_ctl.speed = -self.config['limit_reverse']*th
       	else:                   # do nothing in Park
            	self.pilot.car_ctl.speed = 0.0
            	self.pilot.car_ctl.acceleration = 0.0


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
        #turn = math.tan(turn) * ArtVehicle.max_steer_radians

	# currently doesn't work in reverse
	if self.pilot.pstate.current.speed == 0:
		percentage = 1
	else:
		percentage = -0.2738*(math.log(math.fabs(self.pilot.pstate.current.speed))) + 0.6937
	turn = turn * percentage * ArtVehicle.max_steer_radians

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
