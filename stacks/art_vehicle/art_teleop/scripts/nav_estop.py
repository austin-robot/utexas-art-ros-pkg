#!/usr/bin/env python
#
#  ART vehicle navigator E-stop interface for teleoperation
#
#   Copyright (C) 2011 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'art_teleop'
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

from art_msgs.msg import Behavior
from art_msgs.msg import EstopState
from art_msgs.msg import NavigatorCommand
from art_msgs.msg import NavigatorState

class EstopNavigator():
    "ART navigator E-stop control interface."

    def __init__(self, use_navigator=True):
        "constructor: binds to ROS topics"
        self.use_navigator = use_navigator
        self.last_state_ = EstopState()
        self.new_state_ = EstopState()
        self.new_behavior_ = Behavior.NONE
        self.cmd = rospy.Publisher('navigator/cmd', NavigatorCommand)
        self.topic = rospy.Subscriber('navigator/state', NavigatorState,
                                      self.check_state)

    def check_state(self, state_msg):
        "check navigator state, request change if not desired state"
        self.last_state_ = state_msg.estop.state
        rospy.logdebug('E-stop state: ' + str(self.last_state_))
        if (self.last_state_ != self.new_state_
            and self.new_behavior_ != Behavior.NONE):
            # send navigator command msg requesting new behavior
            cmd_msg = NavigatorCommand()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.header.frame_id = '/map'
            cmd_msg.order.behavior.value = self.new_behavior_
            self.cmd.publish(cmd_msg)

    def pause(self):
        "request immediate stop"
        self.new_state_ = EstopState.Pause
        self.new_behavior_ = Behavior.Pause
        rospy.loginfo('Stopping')

    def run(self):
        "request autonomous running"
        self.new_state_ = EstopState.Run
        self.new_behavior_ = Behavior.Run
        rospy.loginfo('Running autonomously')

    def suspend(self):
        "request suspension of autonomous operation"
        self.new_state_ = EstopState.Suspend
        self.new_behavior_ = Behavior.Suspend
        rospy.loginfo('Suspending autonomous operation')

    def is_suspended(self):
        "return True if navigator operation suspended (or not using navigator)"
        return (not self.use_navigator
                or (self.last_state_ == EstopState.Suspend))

# standalone test of package
if __name__ == '__main__':

    rospy.init_node('nav_estop')

    # make an instance
    est = EstopNavigator()

    # run the program
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
