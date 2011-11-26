/*
 *  Navigator follow lane controller
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
#include "follow_lane.h"

#include "avoid.h"
#include "follow_safely.h"
#include "slow_for_curves.h"
#include "stop_area.h"
#include "stop_line.h"

typedef struct
{
  const char *name;
  bool	do_stop;
} way_table_t;
  
static const way_table_t way_table[] =
  {
    {"Lane", false},
    {"LaneChange", true},
    {"Merge", true},
    {"Stop", true},
    {"Uturn", true},
    {"Zone", false},	// set to true so stop_line marks WPT reached
    {"ZoneExit", true},	// set to true so stop_line marks WPT reached
  };

// return upcoming significant way-point type
//
// exit: sets stop_point, depending on way-point type
//
FollowLane::way_type_t
FollowLane::approaching_waypoint_type(WayPointNode &stop_point)
{
  way_type_t wtype = Lane;
  unsigned i = 0;
  while (++i < Order::N_WAYPTS)
    {
      // only look in the goal lane
      if (!course->same_lane(order->waypt[1].id, order->waypt[i].id))
	break;
      if (order->waypt[i].is_perimeter &&
	  order->waypt[i].is_exit)
	{
	  wtype = ZoneExit;
	  break;
	}
      else if (order->waypt[i].is_perimeter)
	{
	  wtype = Zone;
	  break;
	}
      else if (order->waypt[i].is_stop)
	{
	  wtype = Stop;
	  break;
	}
      else if (course->uturn_waypt(i))
	{
	  wtype = Uturn;
	  break;
	}
      else if (order->waypt[i].is_lane_change)
	{
	  wtype = LaneChange;
	  break;
	}
      else if (order->waypt[i].is_exit
	       && i+1 < Order::N_WAYPTS
	       && order->waypt[i+1].is_entry
	       && !course->same_lane(order->waypt[i].id,
				     order->waypt[i+1].id))
	{
	  if (course->nqe_special(i,i+1))
	    ART_MSG(3, "Not going to merge for special case");
	  else
	    {
	      // This is a transition between a pair of exit and entry
	      // way-points in different lanes (or from end to start of a
	      // single lane).  Treat that as a merge operation: stop and
	      // wait until it is clear to go.
	      
	      // TODO: ignore this if it is not really an intersection.
	      // That could theoretically be determined when there are no
	      // transition polygons from that exit to any other
	      // way-point.  Hard to do here, and only medium priority.
	      wtype = Merge;
	      break;
	    }
	}
    }

  if (way_table[wtype].do_stop)
    stop_point = order->waypt[i];
  else
    stop_point.id = ElementID();

  return wtype;
}

FollowLane::FollowLane(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  //avoid = new Avoid(navptr, _verbose);
  follow_safely = new FollowSafely(navptr, _verbose);
  slow_for_curves = new SlowForCurves(navptr, _verbose);
  stop_area =	new StopArea(navptr, _verbose);
  stop_line =	new StopLine(navptr, _verbose);
  reset_me();
};

FollowLane::~FollowLane()
{
  //delete avoid;
  delete follow_safely;
  delete slow_for_curves;
  delete stop_area;
  delete stop_line;
};

// follow lane in the normal direction
//
// result:
//	OK if able to continue or queued within safety area;
//	Finished, if stop, intersection or U-turn way-point reached;
//	Blocked, if lane blocked and outside safety area;
//	safety controller results other than Blocked;
//
Controller::result_t FollowLane::control(pilot_command_t &pcmd)
{
  if (order->waypt[1].is_perimeter)
    pcmd.velocity=fminf(pcmd.velocity,1.0); //Make this config


#if 0 // not doing avoid right now
  pilot_command_t incmd = pcmd;		// copy of original input
#endif // not doing avoid right now

  // set approaching way-point type
  way_type_t wtype = approaching_waypoint_type(course->stop_waypt);

  // A safety area includes an intersection and the last 30m of any
  // lane leading up to a stop line.
  bool in_intersection = (!course->same_lane(order->waypt[0].id,
					     order->waypt[1].id)
			  || order->waypt[0].is_stop);

  // the first time through there is no course->plan yet
  if (course->in_lane(MapPose(estimate->pose.pose))) // in travel lane?
    {
      if (in_intersection)
	{
	  ROS_DEBUG("Cross intersection to waypoint %s (%s ahead)",
                    ElementID(order->waypt[1].id).name().str,
                    way_table[wtype].name);
	  // TODO: make .cfg option
	  //	  pcmd.velocity = fminf(pcmd.velocity, 3.0);
	}
      else
	{
          ROS_DEBUG("Follow lane to waypoint %s (%s ahead)",
		    ElementID(order->waypt[1].id).name().str,
		    way_table[wtype].name);
      
	  // if signalling while joining a lane, turn signals off now
	  course->turn_signals_off();
	}

      // fill in path to the next way-points
      course->find_travel_lane(false);	// not rejoining the lane
    }
  else
    {
      // set a path to join the travel lane
      ROS_DEBUG("Go to lane containing waypoint %s (%s ahead)",
		ElementID(order->waypt[1].id).name().str,
		way_table[wtype].name);

      // fill in path to the next way-points
      course->find_travel_lane(true);	// rejoin lane
    }

  // slow down if no plan
  if (course->plan.empty())
    nav->reduce_speed_with_min(pcmd, config_->lost_speed);

  bool in_safety_area = (in_intersection
			 || stop_area->control(pcmd) == OK);

  // adjust speed to maintain a safe following distance in the lane
  result_t result = follow_safely->control(pcmd);

  // reduce speed if approaching sharp turn.
  slow_for_curves->control(pcmd);

  if (in_safety_area)
    {
      // Do not return Blocked or Collision in safety area, because no
      // passing is allowed.
      if (result == Blocked)
	{
	  // reset blockage timer while queued within stop area
	  if (!in_intersection)
	    {
// #ifdef NQE
// 	      if (order->waypt[1].id.seg!=4)
// 		obstacle->blocked();
// #else
	      obstacle->blocked();
// #endif
	    }
	  
	  result = Unsafe;
	}
      else if (result == Collision)
	result = Unsafe;
    }
  
  if (way_table[wtype].do_stop)		// stopping point in order?
    {
      result_t stop_result = stop_line->control(pcmd);
      if (stop_result == Finished)	// stopping point reached?
	result = stop_result;		// ignore follow_safely result
    }

  // set heading to desired course
  course->desired_heading(pcmd);

  // check if way-point reached, ignoring stop lines and U-turns
  course->lane_waypoint_reached();

#if 0 // not doing avoid right now
  if (result == OK)			// still moving, no Collision?
    {
      result = avoid->control(pcmd, incmd);
      if (result == Blocked)
	result = Unsafe;		// return Blocked only for passing
    }
#endif // not doing avoid right now

  trace("follow_lane controller", pcmd, result);

  return result;
};

// reset all subordinate controllers
void FollowLane::reset(void)
{
  trace_reset("FollowLane");
  reset_me();
  //avoid->reset();
  follow_safely->reset();
  slow_for_curves->reset();
  stop_area->reset();
  stop_line->reset();
}

// reset this controller only
void FollowLane::reset_me(void)
{
  navdata->reverse = false;
}
