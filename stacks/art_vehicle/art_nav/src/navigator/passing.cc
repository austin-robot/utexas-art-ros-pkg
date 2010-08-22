//
// Navigator passing controller
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

#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "obstacle.h"
#include "passing.h"

#include "avoid.h"
#include "halt.h"
#include "lane_heading.h"
#include "follow_safely.h"
#include "slow_for_curves.h"

#include <art/DARPA_rules.hh>

Passing::Passing(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  avoid = new Avoid(navptr, _verbose);
  halt = new Halt(navptr, _verbose);
  follow_safely = new FollowSafely(navptr, _verbose);
  lane_heading = new LaneHeading(navptr, _verbose);
  slow_for_curves = new SlowForCurves(navptr, _verbose);
  reset_me();
};

Passing::~Passing()
{
  delete avoid;
  delete halt;
  delete follow_safely;
  delete lane_heading;
  delete slow_for_curves;
};

void Passing::configure(ConfigFile* cf, int section)
{

  // minimum passing distance
  passing_distance = cf->ReadFloat(section, "passing_distance", 10.0);
  ART_MSG(2, "\tminimum passing distance is %.3f m", passing_distance);

  // required clearance ahead and behind to return from passing
  passing_distance_ahead = cf->ReadFloat(section, "passing_distance_ahead",
					 DARPA_rules::front_limit_after_pass
					 + ArtVehicle::front_bumper_px
					 + ArtVehicle::length);
  ART_MSG(2, "\tpassing distance ahead is %.3f m", passing_distance_ahead);

  passing_distance_behind = cf->ReadFloat(section, "passing_distance_behind",
					  DARPA_rules::min_rear_sep_after_pass
					  - ArtVehicle::rear_bumper_px
					  - ArtVehicle::halflength);
  ART_MSG(2, "\tpassing distance behind is %.3f m", passing_distance_behind);

  passing_speed = cf->ReadFloat(section, "passing_speed", 3.0);
  ART_MSG(2, "\tpassing speed is %.1f m/s", passing_speed);

  avoid->configure(cf, section);
  halt->configure(cf, section);
  follow_safely->configure(cf, section);
  slow_for_curves->configure(cf, section);
  lane_heading->configure(cf, section);
}

// follow passing lane
//
// result:
//	OK if able to continue or waiting for passing lane to clear;
//	Blocked, if lane blocked;
//	Finished, if original lane reached.
Controller::result_t Passing::control(pilot_command_t &pcmd)
{
  pilot_command_t incmd = pcmd;		// copy of original input

  if (!in_passing_lane
      && course->in_lane(estimate->pos)) // reached passing lane yet?
    in_passing_lane = true;

  if (!in_passing_lane			// not in passing lane yet?
      && !obstacle->passing_lane_clear()) // clear to go?
    {
      trace("passing (not clear) controller", pcmd);
      return halt->control(pcmd);
    }

  if (verbose >= 2)
    {
      ART_MSG(5, "Go to waypoint %s in passing lane",
	      order->waypt[1].id.name().str);
    }

  // reduce speed while passing
  nav->reduce_speed_with_min(pcmd, passing_speed);

  // adjust speed to maintain a safe following distance in our lane
  result_t result = follow_safely->control(pcmd);

  slow_for_curves->control(pcmd);

  if (done_passing())
    {
      ART_MSG(1, "passing completed");
      result = Finished;
    }

  // set desired heading for passing lane
  lane_heading->control(pcmd);		// should work OK

  course->lane_waypoint_reached();

  result_t avoid_result = avoid->control(pcmd, incmd);
  if (result == OK)
    {
      result = avoid_result;
    }

  trace("passing controller", pcmd, result);

  return result;
};

// return true when passing completed
bool Passing::done_passing(void)
{
  bool done = false;
  if (course->distance_in_plan(course->start_pass_location, estimate->pos)
      > passing_distance+DARPA_rules::min_forw_sep_travel
      + ArtVehicle::front_bumper_px);
    {
      // TODO: compute aim point for reentering lane?

      // see if clear to return to passed lane
      float ahead, behind;
      obstacle->closest_in_lane(course->passed_lane, ahead, behind);
      if (ahead >= passing_distance_ahead
	  && behind >= passing_distance_behind)
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
  avoid->reset();
  halt->reset();
  follow_safely->reset();
  lane_heading->reset();
  slow_for_curves->reset();
}

// reset this controller only
void Passing::reset_me(void)
{
  navdata->reverse = false;
  in_passing_lane = false;
}
