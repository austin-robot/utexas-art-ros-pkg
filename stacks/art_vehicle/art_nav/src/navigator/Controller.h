/* -*- mode: C++ -*-
 *
 *  Base class for finite state machine states
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

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
    course = nav->course;
    estimate = &nav->estimate;
    navdata = &nav->navdata;
    obstacle = nav->obstacle;
    odom = nav->odometry;
    order = &nav->order;
    pops = nav->pops;
    config_ = &nav->config_;

    // start the controller from its reset state
    // (any subordinate controller constructors will reset themselves)
    reset_me();
  };

  virtual ~Controller() {};

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
    ROS_DEBUG_NAMED("trace", "%s: pcmd = (%.3f, %.3f) ",
                    name, pcmd.velocity, pcmd.yawRate);
  }

  // trace controller state and result
  virtual void trace(const char *name, const pilot_command_t &pcmd,
		     result_t res)
  {
    ROS_DEBUG_NAMED("trace", "%s: pcmd = (%.3f, %.3f), result = %s",
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
  Course *course;
  Obstacle *obstacle;
  nav_msgs::Odometry *estimate;
  art_msgs::NavigatorState *navdata;
  nav_msgs::Odometry *odom;
  art_msgs::Order *order;
  PolyOps *pops;
  const Config *config_;

  // reset this controller only
  virtual void reset_me(void) {};
};

#endif // __CONTROLLER_HH__
