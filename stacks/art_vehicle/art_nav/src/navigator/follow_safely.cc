/*
 *  Navigator safe following distance controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include "navigator_internal.h"
#include "Controller.h"
#include "obstacle.h"
#include "follow_safely.h"

#include <art/DARPA_rules.h>

FollowSafely::FollowSafely(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose) {};

FollowSafely::~FollowSafely() {};

// Set desired speed for avoiding obstacles within a lane.
//
//  returns:
//	Blocked if lane blocked;
//	Collision if car approaching from ahead in this lane;
//	OK otherwise.
//  exit:
//	sets obstacle->last_obst as a side-effect
//
// We may be following a moving car that could stop at any time or
// already be stopped.  If it stops, stop at a safe distance and set
// lane_blocked.  Higher level controllers will decide if we need to
// pass.  Until told to pass, wait for the obstacle to move.
//
// When stopping, the front bumper must be at least one vehicle length
// (DARPA_rules::min_forw_sep_travel) from the obstacle.
//
// Based on pseudo-code from Dr. Peter Stone.
//
Controller::result_t FollowSafely::control(pilot_command_t &pcmd)
{
  bool was_blocked = navdata->lane_blocked;
  navdata->lane_blocked = false;

  result_t result = OK;
  art_msgs::Observation fobs =
    obstacle->observation(art_msgs::Observation::Nearest_forward);

  ROS_DEBUG("Nearest_forward: C%d A%d, dist %.3f, time %.3f, vel %.3f",
            fobs.clear, fobs.applicable,
            fobs.distance, fobs.time, fobs.velocity);

  if (!fobs.applicable
      || fobs.distance >= obstacle->maximum_range())
    {
      // no obstacle that matters, leave pcmd unmodified
      ROS_DEBUG("no obstacle ahead");
      return result;
    }

  ROS_DEBUG("obstacle is %.3f sec ahead, closing at %.3f m/s",
	    fobs.time, fobs.velocity);

  // A 2 sec minimum following time at 10mph will cause us to brake
  // hard when still about two car lengths away (~9m).  One length is
  // the minimum distance allowed by the DARPA rules.
  if ((fobs.time <= config_->min_following_time)
      || (fobs.distance <= config_->close_stopping_distance))
    {
      // be safe, request immediate stop
      pcmd.velocity = 0.0;
      ROS_DEBUG("Obstacle avoidance requesting immediate halt");

      // when fully stopped, initiate blocked lane behavior
      // (may already be doing it)
      if (navdata->stopped)
	{
	  navdata->lane_blocked = true;
	  if (!was_blocked)
	    {
	      // flag not already set, new obstacle
	      ROS_INFO("New obstacle detected!");
	    }
	  result = Blocked;
	}
    }
  else if (fobs.time < config_->desired_following_time)
    {
      adjust_speed(pcmd, fobs.distance); // speed up a bit
    }
  else if (nav->navdata.stopped
	   || (fobs.time > config_->desired_following_time))
    {
      adjust_speed(pcmd, fobs.distance); // slow down a bit
    }

  // The multiple calls to adjust_speed() above could be tightened
  // up some, but they help make the logic clearer.

  // See if there is anyone coming towards us ahead in this lane.
  if (obstacle->car_approaching())
    {
      ROS_ERROR("Possible collision ahead!");
      result = Collision;
    }

  trace("follow_safely controller", pcmd, result);
  return result;
};


// private methods

// adjust speed to maintain a safe following distance
void FollowSafely::adjust_speed(pilot_command_t &pcmd, float obs_dist)
{
  // try to adjust speed to achieve desired following time, obeying
  // the speed limit (ignore order->min_speed when following)
  float adjusted_speed = fminf(obs_dist / config_->desired_following_time,
			       order->max_speed);

  // at any rate, do not go faster than the desired speed already set
  pcmd.velocity = fminf(pcmd.velocity, adjusted_speed);
}
