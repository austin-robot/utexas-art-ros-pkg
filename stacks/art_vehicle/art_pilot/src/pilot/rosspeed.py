#!/usr/bin/env python
#
# Monitor ROS speed control topics for data analysis.
#
#   Copyright (C) 2009 Austin Robot Technology
#
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'art_pilot'

# standard Python packages
import sys
import getopt
import os
import threading

# ROS node setup
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

# ROS messages
from nav_msgs.msg import Odometry
from art_msgs.msg import BrakeCommand
from art_msgs.msg import BrakeState
from art_msgs.msg import CarCommand
from art_msgs.msg import CarDriveStamped
from art_msgs.msg import ThrottleCommand
from art_msgs.msg import ThrottleState

import plot_speed                       # speed data tools

class commandThread(threading.Thread):
    "Separate thread for shell commands."

    def __init__(self, command):
        "Shell command thread initialization."
        self.command = command
        threading.Thread.__init__(self)

    def run(self):
        "Run a shell command in this thread."
        rc = os.system(self.command)
        rospy.signal_shutdown('bag finished')
        return rc

class speedTopics:
    "Monitor ROS speed topics for data analysis."

    def __init__(self, name=None):
        "Constructor method."

        # set default name prefix for plots
        if name:
            self.name = name
        else:
            self.name = 'speed'
        
        self.plt = plot_speed.speedData()

    def get_pilot_drive(self, cmd):
        "ROS callback for /pilot/drive topic."
        self.plt.set_pilot_cmd(cmd.header.stamp.to_sec(),
                               cmd.control.speed)

    def get_pilot_cmd(self, cmd):
        "ROS callback for /pilot/cmd topic."
        self.plt.set_pilot_cmd(cmd.header.stamp.to_sec(),
                               cmd.control.velocity)

    def get_odometry(self, odom):
        "ROS callback for /odom topic."
        if odom.header.stamp.to_sec() != 0.0:
            self.plt.set_odometry(odom.header.stamp.to_sec(),
                                  odom.twist.twist.linear.x)

    def get_brake_cmd(self, cmd):
        "ROS callback for /brake/cmd topic."
        self.plt.set_brake_cmd(cmd.header.stamp.to_sec(), cmd.position)

    def get_brake_state(self, state):
        "ROS callback for /brake/state topic."
        self.plt.set_brake_state(state.header.stamp.to_sec(), state.position)

    def get_throttle_cmd(self, cmd):
        "ROS callback for /throttle/cmd topic."
        self.plt.set_throttle_cmd(cmd.header.stamp.to_sec(), cmd.position)

    def get_throttle_state(self, state):
        "ROS callback for /throttle/state topic."
        self.plt.set_throttle_state(state.header.stamp.to_sec(), state.position)

    def get_bag(self, filename):
        "Acquire speed topic data from ROS bag."
        filename = os.path.expanduser(filename)
        self.name, fileext = os.path.splitext(os.path.basename(filename))
        commandThread('rosplay -a ' + filename).start()
        self.get_data(False)

    def get_data(self, verbose=True):
        "Acquire speed topic data until ROS shutdown."
        rospy.Subscriber('pilot/drive', CarDriveStamped, self.get_pilot_drive)
        rospy.Subscriber('pilot/cmd', CarCommand, self.get_pilot_cmd)
        rospy.Subscriber('odom', Odometry, self.get_odometry)
        rospy.Subscriber('brake/cmd', BrakeCommand, self.get_brake_cmd)
        rospy.Subscriber('brake/state', BrakeState, self.get_brake_state)
        rospy.Subscriber('throttle/cmd', ThrottleCommand, self.get_throttle_cmd)
        rospy.Subscriber('throttle/state', ThrottleState,
                         self.get_throttle_state)

        rospy.init_node('rosspeed')
        if verbose:
            print 'begin acquiring speed data -- interrupt when finished'
        try:
            rospy.spin()           # invoke callbacks as data arrive
        except rospy.ROSInterruptException: pass
        if verbose:
            print 'done acquiring speed data'

    def plot(self, save=False, plotall=False):
        """ Plot some interesting data from the speed test run.

        When save=True write the figures to PNG files, otherwise show
        them interactively.
        """
        self.plt.name = self.name       # set title for plots
        if plotall:
            self.plt.plot(save)
        else:
            self.plt.plot_summary(save)
        

b = speedTopics()

def usage(progname):
    "Print usage message."
    print "\n", progname, """[-h] [ <file.bag> ]

 -a, --all         generate all available plots (default: just position)
 -h, --help        print this message
 -i, --interactive display plots to terminal (default: save as files)

 <file.bag>\tname of ROS bag file to analyze

If a bag file is specified, read speed topics from it, saving plots of
the data.  Otherwise, read the ROS topics directly from the running
system.
 """

# main program -- for either script or interactive use
def main(argv=None):
    "Main program, called as a script or interactively."

    if argv is None:
        argv = sys.argv                 # use command args
    argv = rospy.myargv(argv)           # filter out ROS args

    # extract base name of command, will be '' when imported
    progname = os.path.basename(argv[0])
    if progname is "":
        progname = "rosspeed.py"

    # process parameters
    try:
        opts, files = getopt.gnu_getopt(argv[1:], 'ahi',
                                        ('all', 'help', 'interactive'))
    except getopt.error, msg:
        print msg
        print "for help use --help"
        return 9

    plotall = False
    save = True

    for k,v in opts:
        if k in ("-a", "--all"):
            plotall = True
        if k in ("-h", "--help"):
            usage(progname)
            return 2
        if k in ("-i", "--interactive"):
            save = False

    if len(files) < 1:
        print "no bag file, reading speed topics directly"
        b.get_data()
        b.plot(save, plotall)

    elif len(files) == 1:
        print "Analyzing " + files[0]
        b.get_bag(files[0])
        b.plot(save, plotall)

    else:
        print "only one ROS bag file can be processed at a time"
        usage(progname)
        return 9

    return 0

# when called as a script or via python-send-buffer
if __name__ == "__main__":
    # run main function and exit
    sys.exit(main())
