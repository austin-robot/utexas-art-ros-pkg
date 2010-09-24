/* -*- mode: C++ -*-
 *
 * Commander finite state machine interface
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __CMDR_FSM_H__
#define __CMDR_FSM_H__

// Commander state transition design notes:
//
// States are nodes in a directed graph representation of the
// Commander finite state machine.  The arrows in this graph represent
// a transition from one state to a (possibly) different one.  Each
// arrow is labelled with an event which triggers that transition, and
// has an associated action method.
//
// Make a matrix of all state transitions indexed by state and event
// containing the action method and a (possibly) new state for each
// arrow in the FSM graph.  This table-driven design is complex, but
// it allows adding new states and events with minimal effect on the
// existing implementation.
//
// Commander() logic:
//
//   return do_transition(current_event());
//
// current_event() will prioritize all events, returning the most
// urgent.  It will check any timers that are running; timer
// expirations are one set of possible events.  The event priorities
// are independent of state.
//
// do_transition() updates the current state, then calls the
// transition-dependent action method from the state transition table.
// Every action method returns a Commander order for this cycle.
//
// Commander could do multiple state transitions in a single cycle.
// Since do_transition() performs the state change before calling the
// action method, in some cases that method may trigger another state
// transition, if necessary.  Currently they do not, which is simpler.

#include <iostream>
#include "State.h"

class CmdrFSM
{
public:

  // state transition action method pointer
  typedef art_msgs::Order (CmdrFSM::*action_t)(CmdrEvent event);

  // state transition table entry
  typedef struct
  {
    CmdrState	next;
    action_t	action;
  } transtion_t;

  CmdrFSM(Commander *cmdr_ptr, int verbosity);
  ~CmdrFSM() {};

  art_msgs::Order control(const art_msgs::NavigatorState *_navstate);

  CmdrState State(void)
  {
    return state;
  }

private:
  ElementID current_way;

  int verbose;
  Commander *cmdr;
  art_msgs::NavigatorState navstate;
  CmdrState prev;
  CmdrState state;
  transtion_t ttable[CmdrEvent::N_events][CmdrState::N_states];

  // Event state variables
  ElementID old_replan;
  bool was_in_route_network;

  // add a transition to the table
  void Add(CmdrEvent::event_t event, action_t action,
	   CmdrState::state_t from_state, CmdrState::state_t to_state);

  // return most urgent current event
  CmdrEvent current_event();


  //////////////////////////////////////////////////////////////////////
  // state transition action methods
  //////////////////////////////////////////////////////////////////////

  // error actions
  art_msgs::Order ActionError(CmdrEvent event);
  art_msgs::Order ActionFail(CmdrEvent event);
  art_msgs::Order ActionWait(CmdrEvent event);

  // steady state actions
  art_msgs::Order ActionInDone(CmdrEvent event);  
  art_msgs::Order ActionInInit(CmdrEvent event);  
  art_msgs::Order ActionInRoad(CmdrEvent event);  

  // state entry actions
  art_msgs::Order ActionToDone(CmdrEvent event);
  art_msgs::Order ActionToRoad(CmdrEvent event);

  // re-planning transitions
  art_msgs::Order BlockedInRoad(CmdrEvent event);
  art_msgs::Order ReplanInRoad(CmdrEvent event);
  art_msgs::Order InitToRoad(CmdrEvent event);
};

#endif // __CMDR_FSM_H__
