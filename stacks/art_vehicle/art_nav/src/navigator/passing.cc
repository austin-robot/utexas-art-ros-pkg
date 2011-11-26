/*
 *  Navigator passing controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "obstacle.h"
#include "passing.h"

#include "halt.h"
#include "follow_safely.h"
#include "slow_for_curves.h"

#include <art/DARPA_rules.h>

Passing::Passing(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  //avoid = new Avoid(navptr, _verbose);
  halt = new Halt(navptr, _verbose);
  follow_safely = new FollowSafely(navptr, _verbose);
  slow_for_curves = new SlowForCurves(navptr, _verbose);
  reset_me();
};

Passing::~Passing()
{
  //delete avoid;
  delete halt;
  delete follow_safely;
  delete slow_for_curves;
};

// follow passing lane
//
// result:
//	OK if able to continue or waiting for passing lane to clear;
//	Blocked, if lane blocked;
//	Finished, if original lane reached.
Controller::result_t Passing::control(pilot_command_t &pcmd)
{
#if 0 // not doing avoid right now
  pilot_command_t incmd = pcmd;		// copy of original input
#endif // not doing avoid right now

  if (!in_passing_lane
      && course->in_lane(MapPose(estimate->pose.pose)))
    {
      // reached passing lane
      in_passing_lane = true;
    }

  if (!in_passing_lane			// not in passing lane yet?
      && !obstacle->passing_lane_clear()) // clear to go?
    {
      trace("passing (not clear) controller", pcmd);
      return halt->control(pcmd);
    }

  ROS_DEBUG("Go to waypoint %s in passing lane",
            ElementID(order->waypt[1].id).name().str);

  // reduce speed while passing
  nav->reduce_speed_with_min(pcmd, config_->passing_speed);

  // adjust speed for any upcoming curves
  result_t result = slow_for_curves->control(pcmd);

  if (done_passing())
    {
      ROS_WARN("passing completed");
      result = Finished;
    }

  // set heading to desired course
  course->desired_heading(pcmd);

  // check if way-point reached
  course->lane_waypoint_reached();

#if 0 // not doing avoid right now
  result_t avoid_result = avoid->control(pcmd, incmd);
  if (result == OK)
    {
      result = avoid_result;
    }
#endif // not doing avoid right now

  trace("passing controller", pcmd, result);

  return result;
};

// return true when passing completed
bool Passing::done_passing(void)
{
  bool done = false;
  double dist_since_passing =
    course->distance_in_plan(course->start_pass_location,
                             MapPose(estimate->pose.pose));
  if (dist_since_passing > config_->passing_distance)
    {
      // TODO: compute aim point for reentering lane?

      // see if clear to return to passed lane
      float ahead, behind;
      obstacle->closest_in_lane(course->passed_lane, ahead, behind);
      if (ahead >= config_->passing_clearance_ahead
	  && behind >= config_->passing_clearance_behind)
	{
	  done = true;
	}
    }
  return done;
}

// reset all subordinate controllers
void Passing::reset(void)
{
  trace_reset("Passing");
  reset_me();
  //avoid->reset();
  halt->reset();
  follow_safely->reset();
  slow_for_curves->reset();
}

// reset this controller only
void Passing::reset_me(void)
{
  navdata->reverse = false;
  in_passing_lane = false;
}
