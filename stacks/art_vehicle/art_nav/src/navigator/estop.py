#!/usr/bin/env python
#
#  E-stop GUI for ART vehicle
#
#   Copyright (C) 2010 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

import sys
import os
import threading
import wx

PKG_NAME = 'art_nav'
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

from art_msgs.msg import Behavior
from art_msgs.msg import EstopState
from art_msgs.msg import NavigatorCommand
from art_msgs.msg import NavigatorState

last_state_ = EstopState()
new_state_ = EstopState()
new_behavior_ = Behavior.NONE

cmd_ = rospy.Publisher('navigator/cmd', NavigatorCommand)
rospy.init_node('estop')

# set path name for resolving icons
icons_path = os.path.join(roslib.packages.get_pkg_dir(PKG_NAME),
                          os.path.join("icons", "estop"))

def check_state(state_msg):
    "check navigator state, request change if not desired state"
    global last_state_, new_state_, new_behavior_
    last_state_ = state_msg.estop.state
    rospy.logdebug('E-stop state: ' + str(last_state_))
    if last_state_ != new_state_ and new_behavior_ != Behavior.NONE:
        # send navigator command msg requesting new behavior
        cmd_msg = NavigatorCommand()
        cmd_msg.header.frame_id = '/map'
        cmd_msg.order.behavior.value = new_behavior_
        cmd_.publish(cmd_msg)

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

    # remember last state received
    last_state_ = state_msg

def pkg_icon(name):
    """Resolve package-relative icon name to path name.
    """
    pname = find_icon(icons_path, name)
    if pname == None:
        raise NameError, 'icon file not found'
    return pname

class MainWindow(wx.Frame):
    "Create main robot E-stop control window."

    def __init__(self, parent, title):

        wx.Frame.__init__(self, parent, title=title, size=wx.Size(600,200))
        self.statusbar = self.CreateStatusBar()

        # Setting up the menu.
        filemenu= wx.Menu()
        menuExit = filemenu.Append(wx.ID_EXIT,"E&xit"," Terminate the program")
        #filemenu.AppendSeparator()

        # Creating the menubar.
        menuBar = wx.MenuBar()
        menuBar.Append(filemenu,"&File")
        self.SetMenuBar(menuBar)

        toolbar = self.CreateToolBar()
        #toolbar.SetToolBitmapSize((48, 48))
        toolbar.AddLabelTool(wx.ID_EXIT, '', wx.Bitmap(pkg_icon('exit')))
        toolbar.AddLabelTool(wx.ID_FORWARD, '',
                             wx.Bitmap(pkg_icon('player_play')))
        toolbar.AddLabelTool(wx.ID_CANCEL, '',
                             wx.Bitmap(pkg_icon('player_pause')))
        toolbar.AddLabelTool(wx.ID_STOP, '',
                             wx.Bitmap(pkg_icon('player_stop')))
	toolbar.Realize()

        self.Bind(wx.EVT_TOOL, self.halt, id=wx.ID_EXIT)
        self.Bind(wx.EVT_MENU, self.halt, menuExit)

        self.Bind(wx.EVT_TOOL, self.run, id=wx.ID_FORWARD)
        self.Bind(wx.EVT_TOOL, self.pause, id=wx.ID_STOP)
        self.Bind(wx.EVT_TOOL, self.suspend, id=wx.ID_CANCEL)

        panel = wx.Panel(self, -1)
        panel.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        panel.SetFocus()

        self.Show(True)

    def halt(self, e):
        # send done command
        rospy.signal_shutdown('GUI exit')
        self.Close(True)                # Close the frame.

    def OnKeyDown(self, event):
        keycode = event.GetKeyCode()
        if keycode == wx.WXK_UP:
            self.run(event)
        elif keycode == wx.WXK_DOWN:
            self.pause(event)
        elif keycode == wx.WXK_PAUSE:
            self.suspend(event)
        else:
            event.Skip()

    def pause(self, e):
        "request immediate stop"
        global new_behavior_, new_state_
        new_state_ = EstopState.Pause
        new_behavior_ = Behavior.Pause
        self.statusbar.SetStatusText('Stopping')
        pass

    def run(self, e):
        "request autonomous run"
        global new_behavior_, new_state_
        new_state_ = EstopState.Run
        new_behavior_ = Behavior.Run
        self.statusbar.SetStatusText('Running')

    def suspend(self, e):
        "request suspension of autonomous operation"
        global new_behavior_, new_state_
        new_state_ = EstopState.Suspend
        new_behavior_ = Behavior.Suspend
        self.statusbar.SetStatusText('Suspending')


class wxThread(threading.Thread):

    def __init__(self):
        self.topic = rospy.Subscriber('navigator/state', NavigatorState, check_state)
        threading.Thread.__init__(self)

    def run(self):
        # run the GUI
        app = wx.App(False)
        frame = MainWindow(None, 'ART robot E-stop control')
        exit_status = app.MainLoop()
        sys.exit(exit_status)

if __name__ == '__main__':

    # run the program
    # needs two threads: GUI main loop and ROS event loop
    try:
        wxThread().start()
        rospy.spin()
    except rospy.ROSInterruptException: pass
