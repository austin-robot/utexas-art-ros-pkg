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

import speed
import time

class SpeedControl(object) :
  def __init__(self) :
    self.brake_position_ = 0.0
    self.throttle_position_ = 0.0

  def set_brake_position(self, position) :
    self.brake_position_ = position

  def set_throttle_position(self, position) :
    self.throttle_position_ = position


class SpeedControlPID (SpeedControl) :
  def __init__(self) :
    SpeedControl.__init__(self)
    self.braking_ = True
    self.brake_pid_ = pid.Pid("brake", -0.2, -0.02, -1.6, 1.0, 0.0, 5000.0)
    self.throttle_pid_ = pid.Pid("throttle", 0.12, 0.001, 0.54, 0.4, 0.0, 5000.0)

    self.configure()
    self.reset()

  def adjust(self, speed, error, throttle_req, brake_req) :
    if self.braking_ :
      ### controlling with brake: ###
      brake_req = self.brake_pid_.Update(error, speed)
      throttle_req = 0.0
      
      # If requesting brake off, switch to throttle control.
      if (True) :
      # Must check reported brake position, too.  Otherwise there
      # will be considerable overlap, applying throttle while the
      # brake is still on.  That can cause mechanical damage to the
      # transmission.
        if ((self.brake_position_ < EPSILON_BRAKE)
            and (brake_req < EPSILON_BRAKE)) :
          brake_req = 0.0              # brake off
          self.braking_ = False             # using throttle now
          self.throttle_pid_.Clear()       # reset PID controller
        
      else :     
        # Allow more overlap, to damp the oscillations that occur when
        # switching back and forth between throttle and brake.
        if (brake_req < EPSILON_BRAKE) :
          brake_req = 0.0;                  # brake off
          self.braking_ = False             # using throttle now
          self.throttle_pid_.Clear()      # reset PID controller
    
    else :
      # controlling with throttle:
      throttle_req = self.throttle_pid_.Update(error, speed);
      brake_req = 0.0

      # If requesting throttle off, switch to brake control.

      # Since throttle responds much faster than brake, it will reach
      # idle before the brake really starts engaging.  So, not
      # checking throttle_position_ here is an option, which reduces
      # latency when slowing down.
      if (throttle_req < EPSILON_THROTTLE) :
          throttle_req = 0.0           # throttle off
          self.braking_ = True         # using brake now
          self.brake_pid_.Clear()         # reset PID controller   

    return (throttle_req, brake_req)

  def configure(self) :
    self.brake_pid_.Configure()
    self.throttle_pid_.Configure()
    return

  def reset(self) :
    self.brake_pid_.Clear()
    self.throttle_pid_.Clear() 


# TODO: Finish working on this SpeedControl subclass
class SpeedControlRL (SpeedControl) :
  def __init__ (self) :
    SpeedControl.__init__(self)

  def adjust(self, speed, error, throttle_req, brake_req) :
    #error = goal - speed
    # if speed > goal, then error < 0, brake
    # if speed < goal, then error > 0, accelerate
    if error < 0 :
      throttle_req = 1.0
      brake_req = 0.0
    elif error > 0 :
      throttle_req = 0.0
      brake_req = 1.0
    else :
      throttle_req = 0.0
      brake_req = 0.0
    return throttle_req, brake_req

  def configure(self) :
    # Do some reinforcement learning updates
    return

  def reset(self) :
    # What to do with reset? Nothing?
    return
