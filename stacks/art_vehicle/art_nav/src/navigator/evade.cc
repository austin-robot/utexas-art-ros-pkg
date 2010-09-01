/*
 *  Navigator evade obstacle controller
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
#include "evade.h"
#include "obstacle.h"

#include "halt.h"
#include "lane_edge.h"
#include "safety.h"
#include "ntimer.h"

static const char *state_name[] =
  {
    "Init",
    "Leave",
    "Wait",
    "Return",
  };

Evade::Evade(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  evade_timer = new NavTimer(nav->cycle);
  halt = new Halt(navptr, _verbose);
  lane_edge = new LaneEdge(navptr, _verbose);
  safety = new Safety(navptr, verbose);
  reset_me();
}

Evade::~Evade()
{
  delete evade_timer;
  delete halt;
  delete lane_edge;
  delete safety;
}

void Evade::configure()
{
  evade_delay = cf->ReadFloat(section, "evade_delay", 7.0);
  ART_MSG(2, "\tevade delay is %.1f seconds", evade_delay);

  evade_offset_ratio = cf->ReadFloat(section, "evade_offset_ratio", -2.0);
  ART_MSG(2, "\tevade offset ratio is %.3f", evade_offset_ratio);

  evasion_speed = cf->ReadFloat(section, "evasion_speed", 3.0);
  ART_MSG(2, "\tevasion speed is %.1f m/s", evasion_speed);

  halt->configure(cf, section);
  lane_edge->configure(cf, section);
  safety->configure(cf, section);
}

// This controller is called from Road when a car is approaching in
// the current lane.  Its job is to leave the lane to the right, wait
// until there is no longer a car approaching, then return Finished.
//
// returns:
//	safety controller results, normally;
//	Finished, when clear to return to normal travel lane.
Controller::result_t Evade::control(pilot_command_t &pcmd)
{
  result_t result = Blocked;

  if (verbose >= 4)
    ART_MSG(2, "Evade state is %s", state_name[state]);

  switch (state)
    {
    case Init:
      // Initial state of the Evade controller, reached via reset().
      if (verbose)
	ART_MSG(1, "Begin emergency obstacle evasion");
      course->turn_signal_on(false);	// signal right
      set_state(Leave);
      // fall through

    case Leave:
      if (course->in_lane(estimate->pos))	// still in lane?
	{
	  // leave lane as long as car is still approaching.
	  if (obstacle->car_approaching())
	    {
	      result = leave_lane_right(pcmd);
	    }
	  else
	    {
	      if (verbose)
		ART_MSG(1, "No longer need to evade approaching car");
	      halt->control(pcmd);
	      course->turn_signal_on(true); // signal left
	      result = Finished;
	    }
	  break;			// done for this cycle
	}

      if (verbose)
	ART_MSG(1, "Left travel lane while evading");
      set_state(Wait);
      // TODO: include obstacle->observation(Nearest_forward).time in
      // the delay?
      course->turn_signals_both_on();
      evade_timer->Start(evade_delay);
      // fall through

    case Wait:
      // out of lane now, wait until timer expires.
      result = halt->control(pcmd);
      if (evade_timer->Check())		// timer expired?
	{
	  if (verbose)
	    ART_MSG(1, "Emergency obstacle evasion timeout");
	  course->turn_signal_on(true);	// signal left
	  set_state(Return);
	}
      break;

    case Return:
      // Car has been stopped long enough for the timer to expire.
      evade_timer->Cancel();
      halt->control(pcmd);

      // Regardless of timeout, stay put as long as there is still a
      // car approaching
      if (obstacle->car_approaching())
	result = OK;
      else
	result = Finished;
      break;
    }

  // mark way-point reached, if any was bypassed
  course->lane_waypoint_reached();

  if (result == Blocked)
    result = Unsafe;			// never return Blocked

  trace("evade controller", pcmd, result);
  return result;
}

// leave lane to the right
//
// returns: safety controller result
Controller::result_t Evade::leave_lane_right(pilot_command_t &pcmd)
{
  // go slowly while leaving lane
  pcmd.velocity = fminf(pcmd.velocity, evasion_speed);

  result_t result = Unsafe;
  if (obstacle->observer_clear(ObserverID::Adjacent_right))
    {
      result = lane_edge->control(pcmd, evade_offset_ratio);
      if (result < Unsafe)
	{
	  // evade right did not stop immediately
	  if (verbose >= 2)
	    ART_MSG(5, "leaving lane to right");
	}
    }
  else if (verbose >= 2)
    ART_MSG(5, "right lane blocked, not heading for that lane");
  return result;
}

// reset all subordinate controllers
void Evade::reset(void)
{
  trace_reset("Evade");
  reset_me();
  halt->reset();
  lane_edge->reset();
  safety->reset();
}

// reset this controller
void Evade::reset_me(void)
{
  evade_timer->Cancel();
  state = Init;
}

// perform state transition
void Evade::set_state(state_t newstate)
{
  if (state != newstate)
    {
      if (verbose)
	ART_MSG(4, "Evade state changing from %s to %s",
		state_name[state], state_name[newstate]);
      state = newstate;
    }
}
