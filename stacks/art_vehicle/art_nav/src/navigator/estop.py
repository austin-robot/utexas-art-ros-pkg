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
import wx

PKG_NAME = 'art_nav'
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

rospy.init_node('teleop')

# set path name for resolving icons
icons_path = os.path.join(roslib.packages.get_pkg_dir(PKG_NAME),
                          os.path.join("icons", "estop"))

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

class MainWindow(wx.Frame):
    "Create main robot E-stop control window."

    def __init__(self, parent, title):

        wx.Frame.__init__(self, parent, title=title, size=wx.Size(450,200))
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
        toolbar.AddLabelTool(wx.ID_STOP, '',
                             wx.Bitmap(pkg_icon('player_pause')))
	toolbar.Realize()

        self.Bind(wx.EVT_TOOL, self.halt, id=wx.ID_EXIT)
        self.Bind(wx.EVT_MENU, self.halt, menuExit)

        self.Bind(wx.EVT_TOOL, self.run, id=wx.ID_FORWARD)
        self.Bind(wx.EVT_TOOL, self.pause, id=wx.ID_STOP)

        self.Show(True)

    def halt(self, e):
        # send done command
        self.Close(True)                # Close the frame.

    def pause(self, e):
        self.statusbar.SetStatusText('Paused')
        pass

    def run(self, e):
        self.statusbar.SetStatusText('Running')
        pass

if __name__ == '__main__':

    # run the program
    app = wx.App(False)
    frame = MainWindow(None, 'ART robot E-stop control')
    exit_status = app.MainLoop()
    sys.exit(exit_status)
