#!/usr/bin/env python
"""
/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file
 
     ART autonomous vehicle speed controller interface.
 
        SpeedControl -- virtual base speed controller class
   
        SpeedControlMatrix -- speed control using acceleration matrix

        SpeedControlPID -- speed control using direct PID of throttle
                           and brake

     \author Jack O'Quin, David Kraft-Ishihama

 */
"""

# This file is a combination of the speed.cc and speed.h files
# written in python. I have no idea if the code here works, but
# it should have the same behavior as the C++ version.

import roslib
roslib.load_manifest('art_nav')

import rospy
import pid

EPSILON_BRAKE = 0.01
EPSILON_THROTTLE = 0.01

#from art_common.msg import conversions
#from art_common.msg import hertz

class SpeedControl :
  def __init__(self) :
    # How should the node_ variable be handled in the python version?
    # Do I just leave this out? Then what do brakePID and throttlePID
    # take as a function argument?

    # self.node_ = ros::NodeHandle("~")
    self.brake_position_ = 0.0
    self.throttle_position_ = 0.0

  def set_brake_position(self, position) :
    self.brake_position_ = position

  def set_throttle_position(self, position) :
    self.throttle_position_ = position
###############################

  # acceleration matrix dimensions
N_DELTAS = 13
DELTA_0 = (N_DELTAS-1)/2 # middle row: delta == 0
N_SPEEDS = 6

  # Velocity control

  # Acceleration matrix: rows are indexed according to the requested
  # speed delta in MPH, columns are indexed by current speed in MPH.
  # The main reason for making all this table-driven is so we can
  # adjust some values without affecting all the others.  
  #
  # NOTE: values in this table represent a percentage change per second,
  # so their effect builds up quickly.

#each pair of tuples is (brake, throttle)
accel_matrix = (
      #              0        <=4       <=8       <=16      <=32      more  
      ((1000,-90), (12,-30), (12,-30), (12,-40), (12,-50),(20,-70)),#-32
      ((1000,-60), ( 6,-20), ( 6,-20), ( 6,-30), ( 6,-40),(10,-55)),#-16
      ((1000,-40), ( 4,-15), ( 4,-15), ( 4,-20), ( 4,-30),( 7,-40)),#-8
      ((500, -20), ( 3,-10), ( 3,-10), ( 3,-10), ( 3,-20),( 5,-25)),#-4
      ((200, -10), ( 2, -5), ( 2, -5), ( 2, -5), ( 2,-10),( 3,-10)),#-2
      ((100,  -5), ( 1, -2), ( 1, -2), ( 1, -2), ( 1, -5),( 1, -5)),#-1
      ((100,   0), ( 0,  0), ( 0,  0), ( 0,  0), ( 0,  0),( 0,  0)),#0
      ((-100,  1), (-2,  2), (-2,  2), (-2,  2), (-2,  5),(-2,  5)),#1
      ((-200,  2), (-5,  5), (-5,  5), (-5,  5), (-5, 10),(-5, 10)),#2
      ((-300,  3), (-10,10), (-10,10), (-10,10), (-10,20),(-10,25)),#4
      ((-500,  4), (-15,15), (-15,15), (-15,20), (-15,30),(-15,40)),#8
      ((-1000, 5), (-20,20), (-20,20), (-20,30), (-20,40),(-20,55)),#16
      ((-1000, 6), (-30,30), (-30,30), (-30,40), (-30,50),(-30,70)) #32
    )
  # If it turns out that the matrix indices should be different, adjust
  # the conversions in these functions.  The logarithmic intervals are
  # based on intuition.

def delta_row(mph) :
  row = 0
  row_limits = [0.0, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0]
  mph_sign = 1

  if (mph < 0) :
    mph_sign = -1

  abs_mph = mph * mph_sign

  while row < len(row_limits) :
    if (abs_mph <= row_limits[row]) :
      return DELTA_0 + mph_sign * row
    row += 1   

    # if delta outside the table, use the first or last entry
  return DELTA_0 + mph_sign * DELTA_0
  
def speed_col(mph) :
  col = 0
  col_limits = [0.0, 4.0, 8.0, 16.0, 32.0, 64.0]

  while col < len(col_limits) :
    if (mph <= col_limits[col]) :
      return col
    col += 1

  # if speed beyond the table, use the last entry
  return N_SPEEDS-1
#######################


class SpeedControlMatrix (SpeedControl):
  def __init__(self) :
    SpeedControl.__init__(self)
    self.velpid_ = Pid("speed", 2.0, 0.0, 32.0)
    self.configure()
    self.reset()

  def adjust(self, speed, error, throttle_req, brake_req) :
    delta = velpid_.Update(error, speed)
    delta_mph = mps2mph(delta)

    # index into the acceleration matrix
    row = delta_row(delta_mph)
    col = speed_col(mps2mph(speed))
    brake_delta = (accel_matrix[row][col][0] / HERTZ_PILOT) / 100.0
    throttle_delta = (accel_matrix[row][col][1] / HERTZ_PILOT) / 100.0

    rospy.logdebug("accel_matrix[%d][%d] contains {%.3f, %.3f}", row, col, brake_delta, throttle_delta)

    # Do not add braking unless nearly idle throttle was previously
    # requested, or throttle unless the brake is nearly off.
    if (throttle_req > 0.0 and brake_delta > 0.0) :
      brake_delta = 0.0
    if (brake_req > 0.0 and throttle_delta > 0.0) :
      throttle_delta = 0.0
    brake_req += brake_delta
    throttle_req += throttle_delta
    return (throttle_req, brake_req)

  def configure(self) :
    # Commented out because I'm not sure what to do with node_ in the
    # base class
    #self.velpid_.configure(self.node_)
    return

  def reset(self) :
    self.velpid_.clear()
    return

class SpeedControlPID (SpeedControl) :
  def __init__(self) :
    SpeedControl.__init__(self)
    self.braking_ = True
    self.brake_pid_ = Pid("brake", -0.2, -0.02, -1.6, 1.0, 0.0, 5000.0)
    self.throttle_pid_ = Pid("throttle", 0.12, 0.001, 0.54, 0.4, 0.0, 5000.0)

    self.configure()
    self.reset()

  def adjust(self, speed, error, throttle_req, brake_req) :
    if self.braking_ :
      ### controlling with brake: ###
      brake_req = brake_pid_.Update(error, speed)
      throttle_req = 0.0
      
      # If requesting brake off, switch to throttle control.
      if (True) :
      # Must check reported brake position, too.  Otherwise there
      # will be considerable overlap, applying throttle while the
      # brake is still on.  That can cause mechanical damage to the
      # transmission.
        if ((self.brake_position_ < EPSILON_BRAKE)
            and (brake_req < pilot.EPSILON_BRAKE)) :
          brake_req = 0.0              # brake off
          self.braking_ = False             # using throttle now
          self.throttle_pid.Clear()       # reset PID controller
        
      else :     
        # Allow more overlap, to damp the oscillations that occur when
        # switching back and forth between throttle and brake.
        if (brake_req < EPSILON_BRAKE) :
          brake_req = 0.0;                  # brake off
          self.braking_ = False             # using throttle now
          self.throttle_pid_.Clear()      # reset PID controller
    
    else :
      # controlling with throttle:
      throttle_req = self.throttle_pid.Update(error, speed);
      brake_req = 0.0

      # If requesting throttle off, switch to brake control.

      # Since throttle responds much faster than brake, it will reach
      # idle before the brake really starts engaging.  So, not
      # checking throttle_position_ here is an option, which reduces
      # latency when slowing down.
      if (throttle_req < EPSILON_THROTTLE) :
          throttle_req = 0.0           # throttle off
          self.braking_ = true         # using brake now
          self.brake_pid.Clear()         # reset PID controller   

    return (throttle_req, brake_req)

  def configure(self) :
    #self.brake_pid_.Configure(self.node_)
    #self.throttle_pid_.Configure(self.node_)
    return

  def reset(self) :
    self.brake_pid_.Clear()
    self.throttle_pid_.Clear()  
