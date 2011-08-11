/* -*- mode: C++ -*-
 *
 *  Navigator road controller finite state machine
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __ROAD_HH__
#define __ROAD_HH__

#include <art_nav/NavRoadState.h>
#include "NavRoadEvent.h"

//class Evade;
class FollowLane;
class FollowSafely;
class Halt;
class Passing;
class NavTimer;
//class RealZone;
class Uturn;

class Road: public Controller
{
public:

  Road(Navigator *navptr, int _verbose);
  ~Road();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

  NavRoadState State(void)
  {
    return state;
  }

private:

  bool passing_first;

  // state transition action method pointer
  typedef result_t (Road::*action_t)(pilot_command_t &pcmd);

  // state transition table entry
  typedef struct
  {
    NavRoadState next;
    action_t	 action;
  } transtion_t;

  NavRoadState	prev;
  NavRoadState	state;
  NavRoadEvent	event;			// current event
  NavRoadEvent	pending_event;
  Observation::_oid_type crossing_observer;
  Course::direction_t lane_direction;
  transtion_t	ttable[NavRoadEvent::N_events][NavRoadState::N_states];

  // ActionInWaitStop() state:
  int32_t prev_nobjects;		// previous number of cars

  // subordinate controllers
  //Evade	*evade;
  FollowLane	*follow_lane;
  FollowSafely	*follow_safely;
  Halt		*halt;
  Passing	*passing;
  Uturn	*uturn;
  //RealZone	*zone;

  // timers
  NavTimer		*passing_timer;
  NavTimer		*precedence_timer;
  NavTimer		*roadblock_timer;
  NavTimer		*stop_line_timer;

  // add a transition to the table
  void Add(NavRoadEvent::event_t event, action_t action,
	   NavRoadState::state_t from_state, NavRoadState::state_t to_state);

  // private methods
  void cancel_all_timers(void);
  void reset_me(void);
  void set_waypt_event(void);

  //////////////////////////////////////////////////////////////////////
  // state transition action methods
  //////////////////////////////////////////////////////////////////////

  result_t ActionError(pilot_command_t &pcmd);
  result_t ActionFail(pilot_command_t &pcmd);

  // steady state actions

  result_t ActionInBlock(pilot_command_t &pcmd);
  result_t ActionInEvade(pilot_command_t &pcmd);
  result_t ActionInFollow(pilot_command_t &pcmd);
  result_t ActionInInit(pilot_command_t &pcmd);
  result_t ActionInPass(pilot_command_t &pcmd);
  result_t ActionInUturn(pilot_command_t &pcmd);
  result_t ActionInWaitCross(pilot_command_t &pcmd);
  result_t ActionInWaitLane(pilot_command_t &pcmd);
  result_t ActionInWaitPass(pilot_command_t &pcmd);
  result_t ActionInWaitStop(pilot_command_t &pcmd);
  result_t ActionInZone(pilot_command_t &pcmd);

  // state entry actions

  result_t ActionEvadeToFollow(pilot_command_t &pcmd);
  result_t ActionPassToBlock(pilot_command_t &pcmd);
  result_t ActionPassToEvade(pilot_command_t &pcmd);
  result_t ActionPassToFollow(pilot_command_t &pcmd);
  result_t ActionToBlock(pilot_command_t &pcmd);
  result_t ActionToEvade(pilot_command_t &pcmd);
  result_t ActionToFollow(pilot_command_t &pcmd);
  result_t ActionToPass(pilot_command_t &pcmd);
  result_t ActionToUturn(pilot_command_t &pcmd);
  result_t ActionToWaitCross(pilot_command_t &pcmd);
  result_t ActionToWaitLane(pilot_command_t &pcmd);
  result_t ActionToWaitPass(pilot_command_t &pcmd);
  result_t ActionToWaitStop(pilot_command_t &pcmd);
  result_t ActionToZone(pilot_command_t &pcmd);
  result_t ActionWaitPassToFollow(pilot_command_t &pcmd);
  result_t ActionZoneToWaitCross(pilot_command_t &pcmd);
};

#endif // __ROAD_HH__
