/* -*- mode: C++ -*-
 *
 * Navigator E-stop controller finite state machine
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __ESTOP_HH__
#define __ESTOP_HH__

#include <art_nav/NavEstopState.h>

#include "Controller.h"
#include "NavEstopEvent.h"

// forward declarations for subordinate classes
class Halt;
class Run;

class Estop: public Controller
{
public:

  Estop(Navigator *navptr, int _verbose);
  ~Estop();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

  NavEstopState State(void)
  {
    return state;
  }

private:

  // state transition action method pointer
  typedef result_t (Estop::*action_t)(pilot_command_t &pcmd);

  // state transition table entry
  typedef struct
  {
    NavEstopState  next;
    action_t       action;
  } transtion_t;

  NavEstopEvent event;
  NavEstopEvent	pending_event;
  NavEstopState prev;
  NavEstopState state;
  transtion_t ttable[NavEstopEvent::N_events][NavEstopState::N_states];

  // subordinate controllers
  Halt	*halt;
  Run	*run;

  // private methods

  // add a transition to the table
  void Add(NavEstopEvent::event_t event, action_t action,
	   NavEstopState::state_t from_state, NavEstopState::state_t to_state);

  // return highest-priority current event
  NavEstopEvent current_event(void);
  void reset_me(void);

  //////////////////////////////////////////////////////////////////////
  // state transition action methods
  //////////////////////////////////////////////////////////////////////

  result_t ActionError(pilot_command_t &pcmd);

  // steady state actions

  result_t ActionInDone(pilot_command_t &pcmd);
  result_t ActionInPause(pilot_command_t &pcmd);
  result_t ActionInRun(pilot_command_t &pcmd);
  result_t ActionInSuspend(pilot_command_t &pcmd);

  // state entry actions

  result_t ActionToAbort(pilot_command_t &pcmd);
  result_t ActionToDone(pilot_command_t &pcmd);
  result_t ActionToPause(pilot_command_t &pcmd);
  result_t ActionToRun(pilot_command_t &pcmd);
  result_t ActionToSuspend(pilot_command_t &pcmd);
};

#endif // __ESTOP_HH__
