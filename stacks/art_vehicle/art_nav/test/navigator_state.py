#!/usr/bin/env python
#
# Monitor ART navigator state
#
#   Copyright (C) 2010 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

import roslib;
roslib.load_manifest('art_nav')

import rospy
from art_msgs.msg import MapID
from art_msgs.msg import EstopState
from art_msgs.msg import NavigatorState
from art_msgs.msg import RoadState

def log_state(state_msg):
    "log navigator state information"
    # TODO print state labels, not just numbers
    rospy.loginfo('E-stop: ' + str(state_msg.estop.state)
                  + ', Road: ' + str(state_msg.road.state)
                  + ', last way-point: ' + str(state_msg.last_waypt.seg)
                  + '.' + str(state_msg.last_waypt.lane)
                  + '.' + str(state_msg.last_waypt.pt))

def test():
    rospy.Subscriber('navigator/state', NavigatorState, log_state)
    rospy.init_node('navigator_state')
    rospy.loginfo('monitoring navigator state')
    rospy.spin()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass

    rospy.loginfo('navigator state monitoring completed')
