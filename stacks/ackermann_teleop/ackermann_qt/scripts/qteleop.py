#!/usr/bin/python
#
# Qt python script to send tele-operation commands to vehicle
#
#   Copyright (C) 2009, 2011, 2012 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'ackermann_qt'

import sys
import os
import math
import threading

from PyQt4 import QtGui
from PyQt4 import QtCore

import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped

# dynamic parameter reconfiguration
from dynamic_reconfigure.server import Server as ReconfigureServer
import ackermann_qt.cfg.QTeleopConfig as Config

g_topic = rospy.Publisher('ackermann_cmd', AckermannDriveStamped)
rospy.init_node('qteleop')

# set path name for resolving icons
icons_path = os.path.join(roslib.packages.get_pkg_dir(PKG_NAME),
                          os.path.join("icons", "teleop"))

def find_icon(dir, basename, extlist=['.svg', '.png']):
    """Find icon file with basename in dir.

    extlist: a list of possible extensions
    returns: full path name if file found, None otherwise
"""
    for ext in extlist:
        pathname = os.path.join(dir, basename + ext)
        if os.path.exists(pathname):
            return pathname
    return None

def pkg_icon(name):
    """Resolve package-relative icon name to path name.
    """
    pname = find_icon(icons_path, name)
    if pname == None:
        raise NameError, 'icon file not found'
    return pname


class MainWindow(QtGui.QMainWindow):
    "Create vehicle tele-operation control window."

    def __init__(self, topic):
        QtGui.QMainWindow.__init__(self)

        self.topic = topic
        self.reconf_server = ReconfigureServer(Config, self.reconfigure)

        self.resize(350, 250)
        self.setIconSize(QtCore.QSize(32,32))
        self.setWindowTitle('Vehicle Tele-Operation')

        qexit = QtGui.QAction(QtGui.QIcon(pkg_icon('exit')), 'Exit', self)
        qexit.setShortcut('Ctrl+Q')
        self.connect(qexit, QtCore.SIGNAL('triggered()'), QtCore.SLOT('close()'))

        center_wheel = QtGui.QAction(QtGui.QIcon(pkg_icon('go-first')),
                            'center wheel', self)
        center_wheel.setShortcut(QtCore.Qt.Key_Home)
        self.connect(center_wheel, QtCore.SIGNAL('triggered()'),
                     self.center_wheel)

        go_left = QtGui.QAction(QtGui.QIcon(pkg_icon('go-previous')),
                            'steer left', self)
        go_left.setShortcut(QtCore.Qt.Key_Left)
        self.connect(go_left, QtCore.SIGNAL('triggered()'), self.go_left)

        go_left_more = QtGui.QAction('steer left more', self)
        go_left_more.setShortcut(QtGui.QKeySequence.SelectPreviousWord)
        self.connect(go_left_more, QtCore.SIGNAL('triggered()'),
                     self.go_left_more)

        go_left_less = QtGui.QAction('steer left less', self)
        go_left_less.setShortcut(QtGui.QKeySequence.MoveToPreviousWord)
        self.connect(go_left_less, QtCore.SIGNAL('triggered()'),
                     self.go_left_less)

        slow_down = QtGui.QAction(QtGui.QIcon(pkg_icon('go-down')),
                            'slow down', self)
        slow_down.setShortcut(QtCore.Qt.Key_Down)
        self.connect(slow_down, QtCore.SIGNAL('triggered()'), self.slow_down)

        speed_up = QtGui.QAction(QtGui.QIcon(pkg_icon('go-up')),
                              'speed up', self)
        speed_up.setShortcut(QtCore.Qt.Key_Up)
        self.connect(speed_up, QtCore.SIGNAL('triggered()'), self.speed_up)

        go_right = QtGui.QAction(QtGui.QIcon(pkg_icon('go-next')),
                              'steer right', self)
        go_right.setShortcut(QtCore.Qt.Key_Right)
        self.connect(go_right, QtCore.SIGNAL('triggered()'), self.go_right)

        go_right_less = QtGui.QAction('steer right less', self)
        go_right_less.setShortcut(QtGui.QKeySequence.MoveToNextWord)
        self.connect(go_right_less, QtCore.SIGNAL('triggered()'),
                     self.go_right_less)

        go_right_more = QtGui.QAction('steer right more', self)
        go_right_more.setShortcut(QtGui.QKeySequence.SelectNextWord)
        self.connect(go_right_more, QtCore.SIGNAL('triggered()'),
                     self.go_right_more)

        stop_car = QtGui.QAction(QtGui.QIcon(pkg_icon('go-bottom')),
                                 'stop car', self)
        stop_car.setShortcut(QtCore.Qt.Key_End)
        self.connect(stop_car, QtCore.SIGNAL('triggered()'), self.stop_car)


        menubar = self.menuBar()
        mfile = menubar.addMenu('&File')
        mfile.addAction(qexit)

        speed = menubar.addMenu('&Speed')
        speed.addAction(slow_down)
        speed.addAction(stop_car)
        speed.addAction(speed_up)

        wheel = menubar.addMenu('&Wheel')
        wheel.addAction(center_wheel)
        wheel.addAction(go_left)
        wheel.addAction(go_left_less)
        wheel.addAction(go_left_more)
        wheel.addAction(go_right)
        wheel.addAction(go_right_less)
        wheel.addAction(go_right_more)

        toolbar = self.addToolBar('Controls')
        toolbar.addAction(qexit)
        toolbar.addAction(center_wheel)
        toolbar.addAction(go_left)
        toolbar.addAction(slow_down)
        toolbar.addAction(stop_car)
        toolbar.addAction(speed_up)
        toolbar.addAction(go_right)

        self.ack_cmd = AckermannDriveStamped()
        self.ack_cmd.header.stamp = rospy.Time.now()
        self.drive = AckermannDrive()
        self.ack_cmd.drive = self.drive
        self.topic.publish(self.ack_cmd)

        self.updateStatusBar()

    def updateStatusBar(self):
        self.statusBar().showMessage('speed: '
                                     + str(self.drive.speed)
                                     + ' m/s,    angle: '
                                     + str(math.degrees(self.drive.steering_angle))
                                     + ' deg')

    def adjustCarCmd(self, v, a):
        """adjust AckermannDrive command

              @param v change in velocity (m/s)
              @param a change in steering angle (radians)
        """

        self.drive.speed += v
        self.drive.steering_angle += a

        # impose limits on commanded angle
        if self.drive.steering_angle > self.config.max_steering:
            self.drive.steering_angle = self.config.max_steering
        if self.drive.steering_angle < self.config.min_steering:
            self.drive.steering_angle = self.config.min_steering

        # clean up angle if it is very close to zero
        if math.fabs(self.drive.steering_angle) < self.config.epsilon_steering:
            self.drive.steering_angle = 0.0

        self.ack_cmd.header.stamp = rospy.Time.now()
        self.ack_cmd.drive = self.drive
        self.topic.publish(self.ack_cmd)

        self.updateStatusBar()

    def center_wheel(self):
        "center steering wheel"
        self.adjustCarCmd(0.0, -self.drive.steering_angle)

    def go_left(self):
        "steer left"
        self.adjustCarCmd(0.0, math.radians(1.0))

    def go_left_more(self):
        "steer more to left"
        self.adjustCarCmd(0.0, math.radians(4.0))

    def go_left_less(self):
        "steer a little to left"
        self.adjustCarCmd(0.0, math.radians(0.25))

    def go_right(self):
        "steer right"
        self.adjustCarCmd(0.0, math.radians(-1.0))

    def go_right_more(self):
        "steer more to right"
        self.adjustCarCmd(0.0, math.radians(-4.0))

    def go_right_less(self):
        "steer far to right"
        self.adjustCarCmd(0.0, math.radians(-0.25))

    def slow_down(self):
        "go one m/s slower"
        self.adjustCarCmd(-1.0, 0.0)    # one m/s slower

    def speed_up(self):
        "go one m/s faster"
        self.adjustCarCmd(1.0, 0.0)     # one m/s faster

    def stop_car(self):
        "stop car immediately"
        self.adjustCarCmd(-self.drive.speed, 0.0)

    def reconfigure(self, config, level):
        "Dynamic reconfigure server callback."
        rospy.logdebug('Reconfigure callback, level ' + str(level))
        rospy.logdebug('New config ' + str(config))
        self.config = config
        return config


class QtThread(threading.Thread):

    def __init__(self, topic):
        self.topic = topic
        threading.Thread.__init__(self)

    def run(self):
        # run the program
        app = QtGui.QApplication(sys.argv)
        
        teleop = MainWindow(self.topic)
        teleop.show()

        # run Qt main loop until window terminates
        exit_status = app.exec_()

        rospy.signal_shutdown('Qt exit')

        sys.exit(exit_status)


if __name__ == '__main__':

    #rospy.loginfo('starting tele-operation')

    try:
        QtThread(g_topic).start()
        rospy.spin()
    except rospy.ROSInterruptException: pass

    #rospy.loginfo('tele-operation completed')
