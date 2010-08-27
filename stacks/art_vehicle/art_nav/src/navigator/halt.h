/* -*- mode: C++ -*-
 *
 *  Navigator halt controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

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
