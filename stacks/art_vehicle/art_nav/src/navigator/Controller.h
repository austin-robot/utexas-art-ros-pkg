//
// Navigator controller base class
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

#ifndef __CONTROLLER_HH__
#define __CONTROLLER_HH__

#include "navigator_internal.h"

class Controller
{
public:

  typedef enum
    {
      // safety results come first, their order is significant
      OK,
      Caution,
      Beware,
      Unsafe,
      Blocked,
      // other results follow safety
      Collision,
      Finished,
      NotApplicable,
      NotImplemented,
      N_results
    } result_t;

  // return result name as a C string
  const char *result_name(result_t result)
  {
    static const char *rname[N_results] =
      {
	"OK",
	"Caution",
	"Beware",
	"Unsafe",
	"Blocked",
	"Collision",
	"Finished",
	"NotApplicable",
	"NotImplemented",
      };
    return rname[result];
  }

  Controller(Navigator *navptr, int _verbose)
  {
    verbose = _verbose;
    nav = navptr;

    // save pointers to frequently-used Navigator data
    cycle = nav->cycle;
    course = nav->course;
    estimate = &nav->estimate;
    navdata = &nav->navdata;
    obstacle = nav->obstacle;
    odom = nav->odometry;
    order = &nav->order;
    pops = nav->pops;

    // start the controller from its reset state
    // (any subordinate controller constructors will reset themselves)
    reset_me();
  };

  virtual ~Controller() {};

  // null configuration method
  virtual void configure(ConfigFile* cf, int section) {};

  // generic controller method
  virtual result_t control(pilot_command_t &pcmd)
  {
    return OK;
  };

  // reset the controller and all its subordinates to a known state
  virtual void reset(void) {};

  // trace controller state
  virtual void trace(const char *name, const pilot_command_t &pcmd)
  {
    if (verbose >= 4)
      ART_MSG(7, "%s: pcmd = (%.3f, %.3f) ",
	      name, pcmd.velocity, pcmd.yawRate);
  }

  // trace controller state and result
  virtual void trace(const char *name, const pilot_command_t &pcmd,
		     result_t res)
  {
    if (verbose >= 4)
      ART_MSG(7, "%s: pcmd = (%.3f, %.3f), result = %s",
	      name, pcmd.velocity, pcmd.yawRate,
	      result_name(res));
  }

  // trace controller resets
  virtual void trace_reset(const char *name)
  {
    if (verbose >= 2)
      ART_MSG(5, "%s::reset()", name);
  }

protected:
  Navigator *nav;
  int verbose;

  // convenience pointers to public Navigator control data
  Cycle *cycle;
  Course *course;
  Obstacle *obstacle;
  player_position2d_data_t *estimate;
  nav_state_msg_t *navdata;
  Odometry *odom;
  Order *order;
  PolyOps *pops;

  // reset this controller only
  virtual void reset_me(void) {};
};

#endif // __CONTROLLER_HH__
