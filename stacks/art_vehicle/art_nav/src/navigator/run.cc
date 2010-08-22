//
// Navigator run controller
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
#include "ntimer.h"
#include "run.h"

#include "halt.h"
#include "road.h"
#include "safety.h"
#include "voronoi_zone.h"

#include <art/estimate.h>
#include <art_nav/ZoneOps.h>

static const char *state_name[] =
  {
    "Continue",
    "Escape",
    "Replan",
  };

Run::Run(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  halt =	new Halt(navptr, _verbose);
  road =	new Road(navptr, _verbose);
  safety =	new Safety(navptr, _verbose);
  unstuck =	new VoronoiZone(navptr, _verbose);

  escape_timer = new NavTimer(cycle);

  go_state = Continue;
};

Run::~Run()
{
  delete halt;
  delete road;
  delete safety;
  delete unstuck;

  delete escape_timer;
};

// go to escape state
void Run::begin_escape(void)
{
  unstuck->reset();
  blockage_pose = estimate->pos;
  blockage_waypt_dist =
    Euclidean::DistanceToWaypt(estimate->pos, order->waypt[1]);
  escape_timer->Start(escape_timeout_secs);
  set_go_state(Escape);
}

void Run::configure(ConfigFile* cf, int section)
{
  // attempt to escape if blockage detected
  escape = cf->ReadBool(section, "escape", true);
  ART_MSG(2, "\tescape is %s", (escape? "true": "false"));

  // distance to travel while escaping a blockage
  escape_distance = cf->ReadFloat(section, "escape_distance", 30.0);
  ART_MSG(2, "\tescape distance is %.3f m", escape_distance);

  // time limit for Escape state
  escape_timeout_secs = cf->ReadFloat(section, "escape_timeout_secs", 60.0);
  ART_MSG(2, "\tescape time is %.3f seconds", escape_timeout_secs);

  // make extra safety check at end of cycle (should not be necessary)
  extra_safety_check = cf->ReadBool(section, "extra_safety_check", false);
  ART_MSG(2, "\textra safety check is %s",
	  (extra_safety_check? "true": "false"));

  // look ahead distance for initialize behavior
  initialize_distance = cf->ReadFloat(section, "initialize_distance", 10.0);
  ART_MSG(2, "\tinitialize look ahead distance is %.3f m",
	  initialize_distance);

  // minimum angle to accept current containing poly as good
  initialize_min_angle = cf->ReadFloat(section, "initialize_min_angle", M_PI/4);
  ART_MSG(2, "\tinitialize minimum angle is %.3f radians",
	  initialize_min_angle);

  // maximum speed we will ever request (in meters/second)
  max_speed = cf->ReadFloat(section, "maxspeed", 15.0);
  ART_MSG(2, "\tmaximum speed is %.3f m/s", max_speed);

  // configure subordinate controllers
  halt->configure(cf, section);
  road->configure(cf, section);
  safety->configure(cf, section);
  unstuck->configure(cf, section);
}

// main controller whenever E-stop is in the Run state
//
//  Each order contains a behavior which determines the navigator
//  run state for this cycle.
//
// returns:
//	OK if able to continue running;
//	NotImplemented if unrecognized behavior in order;
//	NotApplicable if unable to determine starting way-point for
//		Initialize behavior;
//	other results from subordinate controllers.
//
Controller::result_t Run::control(pilot_command_t &pcmd)
{
  // Do nothing if the order is still Run.  Wait until Commander sends
  // a new behavior.  Also, wait until the course controller receives
  // data from the maplanes driver.
  if (order->behavior == NavBehavior::Run)
    //      || course->polygons.empty())
    {
      if (verbose >= 3)
	ART_MSG(8, "run controller not initialized, have %u polygons",
		course->polygons.size());
      // Run order in Run state is valid, but does nothing.
      // Do nothing until lanes data are initialized.
      return halt->control(pcmd);
    }
  
  // initialize pilot command to speed limit, straight ahead
  pcmd.yawRate = 0.0;
  pcmd.velocity = fminf(order->max_speed, max_speed);
  
  // estimate current dead reckoning position based on time of current
  // cycle and latest odometry course and speed

  Estimate::control_pose(odom->curr_pos, odom->time, cycle->Time(),
			 *estimate, verbose);

  course->begin_run_cycle();

  trace("begin run controller", pcmd);

  result_t result;

  // select subordinate controller based on order behavior
  switch (order->behavior.Value())
    {
    case NavBehavior::Go:
      result = go(pcmd);
      break;

    case NavBehavior::Initialize: 
      result = initialize(pcmd);
      break;
      
    default:
      ART_MSG(1, "unsupported Navigator behavior: %s (halting)",
	      order->behavior.Name());
      halt->control(pcmd);
      result = NotImplemented;
    }

  if (extra_safety_check)
    {
      // make safety check for obstacle in our immediate path
      result_t sres = safety->control(pcmd);
      if (sres != OK)
	result = sres;
    }


// #ifdef NQE
//   if (order->waypt[1].id.seg==1 ||
//       order->waypt[1].id.seg==2 ||
//       order->waypt[1].id.seg==41)
//     pcmd.gradual=false;
// #endif
  

  if (navdata->reverse)
    {
      // modify pilot command to use reverse motion
      pcmd.velocity = -pcmd.velocity;
    }

  course->end_run_cycle();

  trace("run controller", pcmd, result);

  return result;
};

// Initialize behavior
//
// returns:
//	OK if successful
//	NotApplicable is unable to determine initial way-point
// exit:
//	pcmd is a halt
//	navdata->last_way is starting way-point, if found
//
Controller::result_t Run::initialize(pilot_command_t &pcmd)
{
  result_t result;

  ElementID start_way = starting_waypt();
  if (start_way != ElementID())
    {
      course->new_waypoint_reached(start_way);
      result = OK;
    }
  else
    {
      // no starting way-point found
      ART_MSG(2, "no starting way-point found, run failed.");
      result = NotApplicable;
      course->no_waypoint_reached();
    }
  
  halt->control(pcmd);
  return result;
}


// controller for Go behavior.
//
//  Blockage handling:
// 
//  Set obstacle blockage timer when car stops.
// 
//  Restart timer in observer_clear() and in follow_lane when waiting
//  for an obstacle in a stop line (not intersection) safety area.
// 
//  When the blockage timer expires, remember the current pose, stop
//  calling the road controller, and instead call the unstuck
//  controller (VoronoiZone).  When it has gone 10m, run the
//  starting_waypt() algorithm to determine a new replan_waypt.  When
//  Commander sees replan_waypt set without the roadblock flag, it
//  will make a new plan.  When the new plan is received, reset the
//  road controller and attempt to continue.
// 
//  While blocked, mark waypt[1] reached whenever the front bumper is
//  within zone_waypoint_radius.  TODO: configure a larger radius.
// 
//  Commander looks at both the roadblock flag and the replan_waypt.
//  If replan_waypt is set it makes a new plan from there.  If the
//  roadblock flag is also set, it marks a blockage in the RNDF after
//  last_waypt.  Commander marks every order with a replan_num that is
//  incremented every time replan route runs, so Navigator can detect
//  when that has been done.
//
Controller::result_t Run::go(pilot_command_t &pcmd)
{
  if (order->waypt[0].id == order->waypt[1].id)
    {
      // halt and let Commander catch up (does not count as timeout)
      ART_MSG(1, "already reached all way-points in order, halting");
      return halt->control(pcmd);
    }

  // check whether car moving, set blockage timer appropriately
  obstacle->update_blockage_state();

  // increment and check blockage timer -- do once per cycle
  if (obstacle->blockage_timeout())
    {
      obstacle->unblocked();		// cancel timer
      if (escape)
	{
	  ART_MSG(1, "Run controller blocked, try to escape.");
	  begin_escape();
	}
      else
	ART_MSG(1, "Run controller blocked, but not trying to escape.");
    }

  switch (go_state)
    {
    case Escape:
      {
	float blockage_distance =
	  Euclidean::DistanceTo(blockage_pose, estimate->pos);

	if (!escape_timer->Check()
#if 0
	    && blockage_distance < escape_distance + blockage_waypt_dist
#else
	    && blockage_distance < escape_distance
#endif
	    )
	  {
	    // vehicle blocked, try to escape (reach waypt[1] if close)
	    result_t result = unstuck->control(pcmd);
	    if (result != Finished)
		return result;

	    ART_MSG(1, "Unstuck controller returns Finished");
	  }

	ART_MSG(1, "Car moved %.3fm since blocking, try to replan.",
		blockage_distance);
	escape_timer->Cancel();
	last_replan = order->replan_num;
	set_go_state(Replan);
	// fall through...
      }

    case Replan:
      if (order->replan_num == last_replan)
	{
	  navdata->replan_waypt = starting_waypt();
	  if (navdata->replan_waypt == ElementID())
	    {
	      ART_MSG(1, "Unable to replan from here, keep trying to escape.");
	      begin_escape();
	    }
	  return halt->control(pcmd);
	}

      // Commander issued new replanned order
      navdata->replan_waypt = ElementID();
      road->reset();
      set_go_state(Continue);
      // fall through...

    case Continue:
      // normal processing -- run the road state machine
      return road->control(pcmd);

    default:
      // not reached, only to avoid compiler warning
      return NotImplemented;
    }
}

// reset all subordinate controllers
void Run::reset(void)
{
  go_state = Continue;

  halt->reset();
  road->reset();
  safety->reset();
}

// set new Go behavior state
void Run::set_go_state(state_t newstate)
{
  if (go_state != newstate)
    {
      if (verbose)
	ART_MSG(4, "Go behavior state changing from %s to %s",
		state_name[go_state], state_name[newstate]);
      go_state = newstate;
    }
}

// find starting way-point for Commander
//
// returns:
//	starting way-point for planning, if available;
//	null ElementID, otherwise.
//
ElementID Run::starting_waypt(void)
{
  ElementID waypt;

  // first look for a containing zone
  segment_id_t starting_zone =
    ZoneOps::containing_zone(course->zones, MapXY(estimate->pos));

  if (verbose >= 4)
    ART_MSG(2, "Run::starting_waypt() containing zone is %d", starting_zone);

  if(starting_zone > 0) {
    ZonePerimeter zone = ZoneOps::get_zone_by_id(course->zones, starting_zone);
    
    waypt = (ZoneOps::starting_node_for_zone(zone)).id;
    
    if (waypt != ElementID())
      {
	ART_MSG(2, "Starting at %s in zone %d",
		waypt.name().str, zone.zone_id);
	return waypt;
      }
  }

  // not in a zone -- find an appropriate lane polygon
  int index = pops->getStartingPoly(estimate->pos,
				    course->polygons,
				    initialize_distance,
				    initialize_min_angle);

  if (index < 0)
    {
      // no starting way-point found
      ART_MSG(2, "getStartingPoly() failed, returning %d", index);
    }
  else
    {
      waypt = course->polygons[index].start_way;
      ART_MSG(2, "starting_waypt() is %s, polygon %u",
	      waypt.name().str, course->polygons[index].poly_id);
    }

  return waypt;
}
