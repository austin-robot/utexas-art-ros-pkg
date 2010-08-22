//
// Navigator safety controller
//
// Safety control runs at the end of the Navigator cycle.  Its job is
// to avoid any obstacle in the vehicle's immediate path, regardless
// of states or lanes.  If we are about to hit something, either halt
// or swerve.
//
// TODO: seems like a good idea, implement it!
//
//  Copyright (C) 2007 Austin Robot Technology
//  All Rights Reserved. Licensed Software.
//
//  This is unpublished proprietary source code of Austin Robot
//  Technology, Inc.  The copyright notice above does not evidence any
//  actual or intended publication of such source code.
//
//  PROPRIETARY INFORMATION, PROPERTY OF AUSTIN ROBOT TECHNOLOGY
//
//  $Id$
//
//  Author: Jack O'Quin
//

#ifndef __SAFETY_HH__
#define __SAFETY_HH__

#include "Controller.h"

class Safety_Distance;

class Safety: public Controller
{
public:
  
  Safety(Navigator *navptr, int _verbose, int mode=0);
  ~Safety();
  
  // null configuration method
  void configure(ConfigFile* cf, int section);
  
  // safety controller
  result_t control(pilot_command_t &pcmd);
  
  // reset all subordinate controllers
  void reset(void)
  {
    // TODO: implement this
  }

private:
  int _mode;

  float far_safety_time;
  float near_safety_time;
  float collision_safety_time;

  float far_slow_ratio;
  float near_slow_ratio;
  float safety_speed;

  Safety_Distance *safety;

  Controller::result_t halt_immediately(pilot_command_t &pcmd);
};

#endif // __SAFETY_HH__
