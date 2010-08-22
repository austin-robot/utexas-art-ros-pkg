//
// Navigator stop line safety area controller
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

#ifndef __STOP_AREA_HH__
#define __STOP_AREA_HH__

class StopArea: public Controller
{
public:

  StopArea(Navigator *navptr, int _verbose);
  ~StopArea();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:
  // .cfg variables
  float stop_approach_speed;		// speed while approaching stop

  // controller state
  bool in_safety_area;

  void reset_me(void);
};

#endif // __STOP_AREA_HH__
