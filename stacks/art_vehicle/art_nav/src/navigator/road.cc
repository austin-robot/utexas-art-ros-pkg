/*
 *  Navigator road controller finite state machine
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "obstacle.h"
#include "road.h"

//#include "evade.h"
#include "follow_lane.h"
#include "follow_safely.h"
#include "halt.h"
//#include "real_zone.h"
#include "passing.h"
#include "ntimer.h"
#include "uturn.h"
//#include "voronoi_zone.h"

Road::Road(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  passing_first=true;

  // initialize transition table, unused entries cause an error action
  for (int event = 0; event < (int) NavRoadEvent::N_events; ++event)
    for (int state = 0; state < (int) NavRoadState::N_states; ++state)
      {
	transtion_t *xp = &ttable[event][state];
	xp->action = &Road::ActionError;
	xp->next = (NavRoadState::state_t) state;
      }

  // valid transitions alphabetically by previous state, event
  
  Add(NavRoadEvent::Collision,		&Road::ActionToEvade,
      NavRoadState::Block,		NavRoadState::Evade);
  Add(NavRoadEvent::FollowLane,		&Road::ActionToFollow,
      NavRoadState::Block,		NavRoadState::Follow);
  Add(NavRoadEvent::None,		&Road::ActionInBlock,
      NavRoadState::Block,		NavRoadState::Block);
  Add(NavRoadEvent::Uturn,		&Road::ActionToUturn,
      NavRoadState::Block,		NavRoadState::Uturn);
  
#if 0 // JOQ: I don't think this is possible
  Add(NavRoadEvent::Block,		&Road::ActionInEvade,
      NavRoadState::Evade,		NavRoadState::Evade);
#endif
  Add(NavRoadEvent::FollowLane,		&Road::ActionEvadeToFollow,
      NavRoadState::Evade,		NavRoadState::Follow);
  Add(NavRoadEvent::None,		&Road::ActionInEvade,
      NavRoadState::Evade,		NavRoadState::Evade);
  Add(NavRoadEvent::Pass,		&Road::ActionToPass,
      NavRoadState::Evade,		NavRoadState::Pass);
  // JOQ: this looks wrong:
  Add(NavRoadEvent::WaitPass,		&Road::ActionInEvade,
      NavRoadState::Evade,		NavRoadState::Evade);
  
  Add(NavRoadEvent::Block,		&Road::ActionToBlock,
      NavRoadState::Follow,		NavRoadState::Block);
  Add(NavRoadEvent::ChangeLane,		&Road::ActionToWaitLane,
      NavRoadState::Follow,		NavRoadState::WaitLane);
  Add(NavRoadEvent::Collision,		&Road::ActionToEvade,
      NavRoadState::Follow,		NavRoadState::Evade);
  Add(NavRoadEvent::FollowLane,		&Road::ActionInFollow,
      NavRoadState::Follow,		NavRoadState::Follow);
  Add(NavRoadEvent::Merge,		&Road::ActionToWaitCross,
      NavRoadState::Follow,		NavRoadState::WaitCross);
  Add(NavRoadEvent::None,		&Road::ActionInFollow,
      NavRoadState::Follow,		NavRoadState::Follow);
  Add(NavRoadEvent::Perimeter,		&Road::ActionToZone,
      NavRoadState::Follow,		NavRoadState::Zone);
  Add(NavRoadEvent::StopLine,		&Road::ActionToWaitStop,
      NavRoadState::Follow,		NavRoadState::WaitStop);
  Add(NavRoadEvent::Uturn,		&Road::ActionToUturn,
      NavRoadState::Follow,		NavRoadState::Uturn);
  Add(NavRoadEvent::WaitPass,		&Road::ActionToWaitPass,
      NavRoadState::Follow,		NavRoadState::WaitPass);

  Add(NavRoadEvent::FollowLane,		&Road::ActionToFollow,
      NavRoadState::Init,		NavRoadState::Follow);
  Add(NavRoadEvent::Merge,		&Road::ActionToWaitCross,
      NavRoadState::Init,		NavRoadState::WaitCross);
  Add(NavRoadEvent::None,		&Road::ActionInInit,
      NavRoadState::Init,		NavRoadState::Init);
  Add(NavRoadEvent::Perimeter,		&Road::ActionToZone,
      NavRoadState::Init,		NavRoadState::Zone);
  
  Add(NavRoadEvent::Block,		&Road::ActionPassToBlock,
      NavRoadState::Pass,		NavRoadState::Block);
  Add(NavRoadEvent::Collision,		&Road::ActionPassToEvade,
      NavRoadState::Pass,		NavRoadState::Evade);
  Add(NavRoadEvent::FollowLane,		&Road::ActionPassToFollow,
      NavRoadState::Pass,		NavRoadState::Follow);
  Add(NavRoadEvent::None,		&Road::ActionInPass,
      NavRoadState::Pass,		NavRoadState::Pass);
  Add(NavRoadEvent::Pass,		&Road::ActionInPass,
      NavRoadState::Pass,		NavRoadState::Pass);

  Add(NavRoadEvent::FollowLane,		&Road::ActionToFollow,
      NavRoadState::Uturn,		NavRoadState::Follow);
  Add(NavRoadEvent::None,		&Road::ActionInUturn,
      NavRoadState::Uturn,		NavRoadState::Uturn);

  Add(NavRoadEvent::FollowLane,		&Road::ActionToFollow,
      NavRoadState::WaitCross,		NavRoadState::Follow);
  Add(NavRoadEvent::None,		&Road::ActionInWaitCross,
      NavRoadState::WaitCross,		NavRoadState::WaitCross);
  Add(NavRoadEvent::Merge,		&Road::ActionInWaitCross,
      NavRoadState::WaitCross,		NavRoadState::WaitCross);

  Add(NavRoadEvent::FollowLane,		&Road::ActionToFollow,
      NavRoadState::WaitLane,		NavRoadState::Follow);
  Add(NavRoadEvent::None,		&Road::ActionInWaitLane,
      NavRoadState::WaitLane,		NavRoadState::WaitLane);
  Add(NavRoadEvent::ChangeLane,		&Road::ActionInWaitLane,
      NavRoadState::WaitLane,		NavRoadState::WaitLane);

  Add(NavRoadEvent::Block,		&Road::ActionToBlock,
      NavRoadState::WaitPass,		NavRoadState::Block);
  Add(NavRoadEvent::Collision,		&Road::ActionToEvade,
      NavRoadState::WaitPass,		NavRoadState::Evade);
  Add(NavRoadEvent::FollowLane,		&Road::ActionWaitPassToFollow,
      NavRoadState::WaitPass,		NavRoadState::Follow);
  Add(NavRoadEvent::None,		&Road::ActionInWaitPass,
      NavRoadState::WaitPass,		NavRoadState::WaitPass);
  Add(NavRoadEvent::Pass,		&Road::ActionToPass,
      NavRoadState::WaitPass,		NavRoadState::Pass);
  Add(NavRoadEvent::WaitPass,		&Road::ActionInWaitPass,
      NavRoadState::WaitPass,		NavRoadState::WaitPass);

  Add(NavRoadEvent::Merge,		&Road::ActionToWaitCross,
      NavRoadState::WaitStop,		NavRoadState::WaitCross);
  Add(NavRoadEvent::None,		&Road::ActionInWaitStop,
      NavRoadState::WaitStop,		NavRoadState::WaitStop);
  Add(NavRoadEvent::StopLine,		&Road::ActionInWaitStop,
      NavRoadState::WaitStop,		NavRoadState::WaitStop);

  Add(NavRoadEvent::None,		&Road::ActionInZone,
      NavRoadState::Zone,		NavRoadState::Zone);
  Add(NavRoadEvent::Perimeter,		&Road::ActionZoneToWaitCross,
      NavRoadState::Zone,		NavRoadState::WaitCross);

  // allocate subordinate controllers
  //evade = new Evade(navptr, _verbose);
  follow_lane = new FollowLane(navptr, _verbose);
  follow_safely = new FollowSafely(navptr, _verbose);
  halt = new Halt(navptr, _verbose);
  passing = new Passing(navptr, _verbose);
  uturn = new Uturn(navptr, _verbose);
  //zone = new RealZone(navptr, _verbose);

  // allocate timers
  passing_timer = new NavTimer();
  precedence_timer = new NavTimer();
  roadblock_timer = new NavTimer();
  stop_line_timer = new NavTimer();

  // reset this controller only
  reset_me();
}

Road::~Road()
{
  //delete evade;
  delete follow_lane;
  delete follow_safely;
  delete halt;
  delete passing;
  delete uturn;
  //delete zone;

  delete passing_timer;
  delete precedence_timer;
  delete roadblock_timer;
  delete stop_line_timer;
};

// add a transition to the table
void Road::Add(NavRoadEvent::event_t event, action_t action,
	       NavRoadState::state_t from_state,
	       NavRoadState::state_t to_state)
{
  transtion_t *xp = &ttable[event][from_state];
  xp->action = action;
  xp->next = to_state;
}

void Road::cancel_all_timers(void)
{
  passing_timer->Cancel();
  precedence_timer->Cancel();
  roadblock_timer->Cancel();
  stop_line_timer->Cancel();
}

Controller::result_t Road::control(pilot_command_t &pcmd)
{
  // get next event
  event = pending_event;
  pending_event = NavRoadEvent::None;

  // state transition table pointer
  transtion_t *xp = &ttable[event.Value()][state.Value()];

  // do state transition
  prev = state;
  state = xp->next;
  if (state != prev)
    {
      if (verbose)
	ART_MSG(4, "Navigator road state changing from %s to %s, event = %s",
		prev.Name(), state.Name(), event.Name());
      navdata->road.state = state.Value(); // update data state message
    }
  else
    {
      if (verbose >= 4)
	ART_MSG(4, "Navigator road state is %s, event = %s",
		state.Name(), event.Name());
    }

  // perform transition action, returning next Pilot command
  action_t action = xp->action;
  return (this->*action)(pcmd);
}

// reset this controller and all subordinates
void Road::reset(void)
{
  trace_reset("Road");
  reset_me();
  //evade->reset();
  follow_lane->reset();
  follow_safely->reset();
  halt->reset();
  passing->reset();
  uturn->reset();
  //zone->reset();
}

// reset this controller only
void Road::reset_me(void)
{
  state = NavRoadState();		// initial state
  event = NavRoadEvent::None;
  pending_event = NavRoadEvent::None;
  cancel_all_timers();
}

void Road::set_waypt_event(void)
{
  if (ElementID(navdata->last_waypt) == ElementID(order->waypt[1].id))
    {
      // new way-point reached, be careful of the order of these tests
      if (order->waypt[1].is_perimeter)
	pending_event = NavRoadEvent::Perimeter;
      else if (order->waypt[1].is_stop)
	pending_event = NavRoadEvent::StopLine;
      else if (course->uturn_waypt(1))
	pending_event = NavRoadEvent::Uturn;
      else if (order->waypt[1].is_lane_change)
	pending_event = NavRoadEvent::ChangeLane;
      else if (order->waypt[1].is_exit
	       && !course->same_lane(order->waypt[1].id,
				     order->waypt[2].id)) {
	if (course->nqe_special(1,2))
	  ART_MSG(3, "Not going to merge for special case");
	else
	  pending_event = NavRoadEvent::Merge;
      }
    }
}

//////////////////////////////////////////////////////////////////////
// state transition action methods
//////////////////////////////////////////////////////////////////////

Controller::result_t Road::ActionError(pilot_command_t &pcmd)
{
  ART_MSG(4, "Invalid Navigator event %s in road state %s",
	  event.Name(), prev.Name());
  halt->control(pcmd);
  return NotApplicable;
}

Controller::result_t Road::ActionFail(pilot_command_t &pcmd)
{
  ART_MSG(4, "Navigator road FSM failure!");
  halt->control(pcmd);
  return NotImplemented;
}

// steady state actions

Controller::result_t Road::ActionInBlock(pilot_command_t &pcmd)
{
  result_t result = Blocked;
  if (course->new_waypts())
    {
      // Commander issued an order with new way-points
      // TODO: cancel timeout
      if (course->uturn_waypt(0))
	{
	  pending_event = NavRoadEvent::Uturn;    
	  ART_MSG(1, "Doing U-turn after road block");
	}
      else 
	{
	  pending_event = NavRoadEvent::FollowLane;
	  ART_MSG(1, "Following lane (NOT U-turn) after road block");
	}
      navdata->road_blocked = false;
      navdata->replan_waypt = ElementID().toMapID();
      halt->control(pcmd);		// do the order next cycle
    }
  else
    {
      // no new order, keep trying to execute the old one
      // TODO: check for timeout and call unstuck controller
      result = follow_lane->control(pcmd);
      if (result == Collision)
	{
	  // either pass or drive off road immediately
	  pending_event = NavRoadEvent::Collision;
	}
      else if (result != Blocked)
	{
	  ART_MSG(1, "No longer blocked, following lane again.");
	  pending_event = NavRoadEvent::FollowLane;

	  // maybe generate another event, if way-point reached
	  set_waypt_event();

	  // no longer blocked
	  navdata->road_blocked = false;
	  navdata->replan_waypt = ElementID().toMapID();
	  // TODO: cancel timeout
	}
    }
  return result;
}  

Controller::result_t Road::ActionInEvade(pilot_command_t &pcmd)
{
  /** @todo Either implement the Evade controller or delete it. */
  //result_t result = evade->control(pcmd);
  result_t result = halt->control(pcmd);
  if (result == Finished)
    {
      pending_event = NavRoadEvent::FollowLane;
      result = OK;
    }
  return result;
}

Controller::result_t Road::ActionInFollow(pilot_command_t &pcmd)
{
  // cancel passing timer when moving in the Follow state
  if (!navdata->stopped)
    passing_timer->Cancel();

  result_t result = follow_lane->control(pcmd);
  if (result == Blocked)
    {
      pending_event = NavRoadEvent::WaitPass;
      result = OK;
    }
  else if (result == Collision)
    {
      // either pass or drive off road immediately
      pending_event = NavRoadEvent::Collision;
    }
  else
    {
      // generate event depending on way-point reached
      set_waypt_event();
      result = OK;
    }
  return result;
}

Controller::result_t Road::ActionInInit(pilot_command_t &pcmd)
{
  if (order->waypt[0].is_perimeter || order->waypt[0].is_spot)
    pending_event = NavRoadEvent::Perimeter;
  else
    //pending_event = NavRoadEvent::Merge;
    pending_event = NavRoadEvent::FollowLane;
  return halt->control(pcmd);
}

Controller::result_t Road::ActionInPass(pilot_command_t &pcmd)
{
  result_t result = passing->control(pcmd);
  if (result == Finished)
    {
      pending_event = NavRoadEvent::FollowLane;
      roadblock_timer->Cancel();
      result = OK;
    }
  else if (result == Blocked)
    {
      // don't report road blocked immediately, wait a while and see
      // if it stays blocked before giving up on this part of the road
      if (roadblock_timer->Check())
	{
	  pending_event = NavRoadEvent::Block;
	  roadblock_timer->Cancel();
	}
      else
	{
	  if (passing_first)
	    {
	      passing_first=false;
	      // TODO: should only reset blockage timer the first time through
	      obstacle->blocked();
	      roadblock_timer->Restart(config_->roadblock_delay);
	      result = Unsafe;
	    }
	}
    }
  else if (result == Collision)
    {
      // pass or drive off road immediately
      pending_event = NavRoadEvent::Collision;
      roadblock_timer->Cancel();
    }
  return result;
}

Controller::result_t Road::ActionInUturn(pilot_command_t &pcmd)
{
  result_t result = uturn->control(pcmd);
  if (result == Finished)
    {
      pending_event = NavRoadEvent::FollowLane;
      result = OK;
    }
  return result;
}  

Controller::result_t Road::ActionInWaitCross(pilot_command_t &pcmd)
{
  result_t result = halt->control(pcmd); // stay put this cycle
  // TODO: add a timeout for when the observer is not applicable?
  if (obstacle->observer_clear(crossing_observer))
    {
      pending_event = NavRoadEvent::FollowLane;
    }
  return result;
}  

Controller::result_t Road::ActionInWaitLane(pilot_command_t &pcmd)
{
  result_t result = halt->control(pcmd); // stay put this cycle
  if (lane_direction == Course::Straight) // missing polygons?
    {
      ART_MSG(1, "Not waiting for lane to clear -- polygons missing.");
      pending_event = NavRoadEvent::FollowLane;
    }
  else
    {
      // direction known, query the relevant observer
      Observation::_oid_type lane_observer;
      if (lane_direction == Course::Left)
	{
	  ART_MSG(1, "Waiting for nearest left lane to clear.");
	  lane_observer = Observation::Adjacent_left;
	}
      else
	{
	  ART_MSG(1, "Waiting for nearest right lane to clear.");
	  lane_observer = Observation::Adjacent_right;
	}
      // TODO: add a timeout for when the observer is not applicable?
      if (obstacle->observer_clear(lane_observer))
	{
	  pending_event = NavRoadEvent::FollowLane;
	}
    }
  return result;
}

Controller::result_t Road::ActionInWaitPass(pilot_command_t &pcmd)
{
  // wait a while to give obstacle a chance to move
  if (passing_timer->Check()		// timer expired?
      && obstacle->passing_lane_clear()) // clear to go?
    {
      // passing lane is clear, go now
      pending_event = NavRoadEvent::Pass;
    }
  
  // continue checking whether lane is still blocked, sets
  // obstacle->last_obst as a side-effect
  result_t result = follow_safely->control(pcmd);
  if (result == OK) // no longer blocked?
    {
      pending_event = NavRoadEvent::FollowLane;
    }
  else if (result == Collision)
    {
      pending_event = NavRoadEvent::Collision;
    }

  // stay put for this cycle
  halt->control(pcmd);
  return result;
}  

Controller::result_t Road::ActionInWaitStop(pilot_command_t &pcmd)
{
  halt->control(pcmd);			// stay put this cycle

  // wait until stopped for a while, then query observer
  if (stop_line_timer->Check()		// initial timer expired?
      && obstacle->observer_clear(Observation::Intersection))
    {
      ART_MSG(1, "Our turn to cross intersection.");
      stop_line_timer->Cancel();
      precedence_timer->Cancel();
      pending_event = NavRoadEvent::Merge;
    }

  // restart precedence timer if number of cars remaining changed
  Observation obs = obstacle->observation(Observation::Intersection);
  if (obs.applicable && obs.nobjects != prev_nobjects)
    {
      prev_nobjects = obs.nobjects;
      ART_MSG(1, "Cars having precedence at this intersection: %d.",
	      prev_nobjects);
      precedence_timer->Start(config_->precedence_delay);
    }

  // catch intersection precedence violation
  if (precedence_timer->Check())
    {
      ART_MSG(1, "Intersection precedence %.1f second timeout, go ahead.",
	      config_->precedence_delay);
      stop_line_timer->Cancel();
      precedence_timer->Cancel();
      pending_event = NavRoadEvent::Merge;
    }
  return OK;
}  

Controller::result_t Road::ActionInZone(pilot_command_t &pcmd)
{
  //result_t result = zone->control(pcmd);
  result_t result = NotApplicable;      // scaffolding
  if (ElementID(navdata->last_waypt) == ElementID(order->waypt[1].id)
      && order->waypt[1].is_perimeter)
    {
      // reached zone perimeter way-point
      pending_event = NavRoadEvent::Perimeter;
    }
  else if (result == NotApplicable)
    {
      pending_event = NavRoadEvent::Perimeter;
      result = OK;
    }
#if 1
  else
    // restart blockage timer (no point running Escape, still in Zone)
    obstacle->blocked();
#endif

  return result;
}

// state entry actions

Controller::result_t Road::ActionEvadeToFollow(pilot_command_t &pcmd)
{
  ART_MSG(1, "Collision evasion done, continue in original lane");
  course->reset();			// JOQ: is this needed?
  return ActionToFollow(pcmd);
}

Controller::result_t Road::ActionPassToBlock(pilot_command_t &pcmd)
{
  ART_MSG(1, "passing blocked, replan route from here");
  course->reset();
  course->plan = course->passed_lane;	// restore original plan
  return ActionToBlock(pcmd);
}

Controller::result_t Road::ActionPassToEvade(pilot_command_t &pcmd)
{
  ART_MSG(1, "danger while passing, try to evade");
  course->reset();
  course->plan = course->passed_lane;	// restore original plan
  return ActionToEvade(pcmd);
}

Controller::result_t Road::ActionPassToFollow(pilot_command_t &pcmd)
{
  ART_MSG(1, "passing completed, returning to original lane");
  course->reset();
  course->signal_pass_return();
  return ActionToFollow(pcmd);
}

Controller::result_t Road::ActionToBlock(pilot_command_t &pcmd)
{
  ART_MSG(1, "Road blocked!");
  course->turn_signals_off();
  navdata->road_blocked = true;
  navdata->replan_waypt = course->replan_roadblock().toMapID();
  follow_lane->reset();			// JOQ: is this needed?
  return ActionInBlock(pcmd);		// wait for commander to replan
}

Controller::result_t Road::ActionToEvade(pilot_command_t &pcmd)
{
#if 1  // don't try to pass for now, easier to unit test Evade logic
  // collision detected: first try to pass immediately
  if (course->find_passing_lane())
    {
      // found a passing lane
      if (obstacle->passing_lane_clear())
	{
	  // passing lane is clear
	  pending_event = NavRoadEvent::Pass;
	  return halt->control(pcmd);	// start passing next cycle
	}
    }
  // no passing lane: evade collision by leaving lane to the right
  //evade->reset();
#endif
  return ActionInEvade(pcmd);
}

Controller::result_t Road::ActionToFollow(pilot_command_t &pcmd)
{
  // leave passing_timer running so we handle oscillations between
  // Follow and WaitPass states (flickering obstacles, etc.
#if 0
  cancel_all_timers();
#endif
  follow_lane->reset();
  return ActionInFollow(pcmd);
}  

Controller::result_t Road::ActionToPass(pilot_command_t &pcmd)
{
  // obstacle->last_obst was set in the previous cycle, by the follow
  // safely controller.
  if (course->switch_to_passing_lane())
    {
      passing_timer->Cancel();
      passing->reset();
      return ActionInPass(pcmd);
    }
  else
    {
      // failed to switch lanes, road blocked
      // TODO: detect recursive blockage in course->find_passing_lane()
      pending_event = NavRoadEvent::Block;
      return halt->control(pcmd);
    }
}

Controller::result_t Road::ActionToUturn(pilot_command_t &pcmd)
{
  ART_MSG(1, "Start U-turn.");
  course->turn_signal_on(true);		// signal left
  uturn->reset();
  return ActionInUturn(pcmd);
}

Controller::result_t Road::ActionToWaitCross(pilot_command_t &pcmd)
{
  Course::direction_t crossing_direction = course->intersection_direction();
#if 0  // always use Merge_across_all, Merge_into_nearest is broken
  if (crossing_direction == Course::Right)
    {
      // assume always headed for nearest right lane
      // TODO: how do I verify that?
      ART_MSG(1, "Waiting for nearest intersection lane to clear.");
      crossing_observer = Observation::Merge_into_nearest;
    }
  else
#endif
    {
      // going Straight or Left, wait for all lanes to clear
      ART_MSG(1, "Waiting for all intersection lanes to clear.");
      crossing_observer = Observation::Merge_across_all;
    }
  course->signal_for_direction(crossing_direction);
  return ActionInWaitCross(pcmd);
}  

Controller::result_t Road::ActionToWaitLane(pilot_command_t &pcmd)
{
  lane_direction = course->lane_change_direction();
  course->signal_for_direction(lane_direction);
  return ActionInWaitLane(pcmd);
}  

Controller::result_t Road::ActionToWaitPass(pilot_command_t &pcmd)
{
  passing_first=true;

  // select passing lane
  if (course->find_passing_lane())
    {
      // success: prepare to pass
      course->signal_pass();
      passing_timer->Restart(config_->passing_delay);
      return ActionInWaitPass(pcmd);
    }
  else
    {
      // failed: road is blocked
      pending_event = NavRoadEvent::Block;
      halt->control(pcmd);
      return Blocked;
    }
}  

Controller::result_t Road::ActionToWaitStop(pilot_command_t &pcmd)
{
  ART_MSG(1, "Waiting for intersection precedence.");
  course->signal_for_direction(course->intersection_direction());
  stop_line_timer->Start(config_->stop_line_delay);
  precedence_timer->Start(config_->precedence_delay);
  prev_nobjects = -1;
  return ActionInWaitStop(pcmd);
}  

Controller::result_t Road::ActionToZone(pilot_command_t &pcmd)
{
  ART_MSG(1, "Entering zone");
  //zone->reset();
  return ActionInZone(pcmd);
}

Controller::result_t Road::ActionWaitPassToFollow(pilot_command_t &pcmd)
{
  ART_MSG(1, "no need to pass, continue in this lane");
  course->reset();
  course->turn_signals_off();
  return ActionToFollow(pcmd);
}

Controller::result_t Road::ActionZoneToWaitCross(pilot_command_t &pcmd)
{
  ART_MSG(1, "Leaving zone");
  return ActionToWaitCross(pcmd);
}
