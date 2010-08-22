//
// Navigator halt controller
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

#ifndef __HALT_HH__
#define __HALT_HH__

#include "course.h"

class Halt: public Controller
{
public:

  Halt(Navigator *navptr, int _verbose):
    Controller(navptr, _verbose) {};

  ~Halt() {};

  // halt immediately
  result_t control(pilot_command_t &pcmd)
  {
    course->no_waypoint_reached();
    pcmd.velocity = 0.0;
    pcmd.yawRate = 0.0;
    return OK;
  };

private:
};

#endif // __HALT_HH__
