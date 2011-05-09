/*
 *  Navigator E-stop controller finite state machine
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include "navigator_internal.h"
#include "Controller.h"
#include "estop.h"

#include "halt.h"
#include "run.h"

Estop::Estop(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  // initialize transition table, unused entries cause an error action
  for (int event = 0; event < (int) NavEstopEvent::N_events; ++event)
    for (int state = 0; state < (int) NavEstopState::N_states; ++state)
      {
	transtion_t *xp = &ttable[event][state];
	xp->action = &Estop::ActionError;
	xp->next = (NavEstopState::state_t) state;
      }

  // initialize transition table:

  Add(NavEstopEvent::Abort,	&Estop::ActionToAbort,
      NavEstopState::Pause,	NavEstopState::Done);
  Add(NavEstopEvent::Abort,	&Estop::ActionToAbort,
      NavEstopState::Run,	NavEstopState::Done);
  Add(NavEstopEvent::Abort,	&Estop::ActionToAbort,
      NavEstopState::Suspend,	NavEstopState::Done);
  Add(NavEstopEvent::Abort,	&Estop::ActionInDone,
      NavEstopState::Done,	NavEstopState::Done);

  Add(NavEstopEvent::None,	&Estop::ActionInPause,
      NavEstopState::Pause,	NavEstopState::Pause);
  Add(NavEstopEvent::None,	&Estop::ActionInRun,
      NavEstopState::Run,	NavEstopState::Run);
  Add(NavEstopEvent::None,	&Estop::ActionInSuspend,
      NavEstopState::Suspend,	NavEstopState::Suspend);
  Add(NavEstopEvent::None,	&Estop::ActionInDone,
      NavEstopState::Done,	NavEstopState::Done);

  Add(NavEstopEvent::Pause,	&Estop::ActionInPause,
      NavEstopState::Pause,	NavEstopState::Pause);
  Add(NavEstopEvent::Pause,	&Estop::ActionToPause,
      NavEstopState::Run,	NavEstopState::Pause);
  Add(NavEstopEvent::Pause,	&Estop::ActionToPause,
      NavEstopState::Suspend,	NavEstopState::Pause);
  Add(NavEstopEvent::Pause,	&Estop::ActionInDone,
      NavEstopState::Done,	NavEstopState::Done);

  Add(NavEstopEvent::Quit,	&Estop::ActionToDone,
      NavEstopState::Pause,	NavEstopState::Done);
  Add(NavEstopEvent::Quit,	&Estop::ActionToDone,
      NavEstopState::Run,	NavEstopState::Done);
  Add(NavEstopEvent::Quit,	&Estop::ActionToDone,
      NavEstopState::Suspend,	NavEstopState::Done);
  Add(NavEstopEvent::Quit,	&Estop::ActionInDone,
      NavEstopState::Done,	NavEstopState::Done);

  Add(NavEstopEvent::Run,	&Estop::ActionToRun,
      NavEstopState::Pause,	NavEstopState::Run);
  Add(NavEstopEvent::Run,	&Estop::ActionInRun,
      NavEstopState::Run,	NavEstopState::Run);
  Add(NavEstopEvent::Run,	&Estop::ActionToRun,
      NavEstopState::Suspend,	NavEstopState::Run);
  Add(NavEstopEvent::Run,	&Estop::ActionInDone,
      NavEstopState::Done,	NavEstopState::Done);

  // Note that the Suspend event is allowed even in the Done state.
  // This allows driving the car with the joystick after the mission
  // is complete.  Although it is possible to go from Done to Suspend
  // to Run, the commander has already shut down, so the car will not
  // begin a new mission.
  Add(NavEstopEvent::Suspend,	&Estop::ActionToSuspend,
      NavEstopState::Pause,	NavEstopState::Suspend);
  Add(NavEstopEvent::Suspend,	&Estop::ActionToSuspend,
      NavEstopState::Run,	NavEstopState::Suspend);
  Add(NavEstopEvent::Suspend,	&Estop::ActionInSuspend,
      NavEstopState::Suspend,	NavEstopState::Suspend);
  Add(NavEstopEvent::Suspend,	&Estop::ActionToSuspend,
      NavEstopState::Done,	NavEstopState::Suspend);

  // allocate subordinate controllers
  run = new Run(navptr, _verbose);
  halt = new Halt(navptr, _verbose);
  reset_me();
}

Estop::~Estop()
{
  delete halt;
  delete run;
};

// add a transition to the table
void Estop::Add(NavEstopEvent::event_t event, action_t action,
		NavEstopState::state_t from_state,
		NavEstopState::state_t to_state)
{
  transtion_t *xp = &ttable[event][from_state];
  xp->action = action;
  xp->next = to_state;
}

Controller::result_t Estop::control(pilot_command_t &pcmd)
{
  event = current_event();

  // state transition table pointer
  transtion_t *xp = &ttable[event.Value()][state.Value()];

  // do state transition
  prev = state;
  state = xp->next;

  if (state != prev)
    {
      // update navigator state message
      //ROS_DEBUG_STREAM
      ROS_INFO_STREAM("Navigator E-stop state changing from " << prev.Name()
                       << " to " << state.Name()
                       << ", event = " << event.Name());
      navdata->estop.state = (art_msgs::EstopState::_state_type) state.Value();
    }

  // perform transition action, returning next Pilot command
  action_t action = xp->action;
  return (this->*action)(pcmd);
}

NavEstopEvent Estop::current_event(void)
{
  NavEstopEvent nevent = pending_event;

  // see if event pending from previous cycle
  if (nevent == NavEstopEvent::None)
    {
      // no pending event, translate order behavior into FSM event
      switch (order->behavior.value)
	{
	case NavBehavior::Abort:
	  nevent = NavEstopEvent::Abort;
	  break;

	case NavBehavior::Pause:
	  nevent = NavEstopEvent::Pause;
	  break;

	case NavBehavior::Quit:
	  nevent = NavEstopEvent::Quit;
	  break;

	case NavBehavior::Run:
	  nevent = NavEstopEvent::Run;
	  break;

	case NavBehavior::Suspend:
	  nevent = NavEstopEvent::Suspend;
	  break;

	default:
	  // Other behaviors are handled by lower-level controllers,
	  // only while in Run state.  They do not affect the E-stop
	  // controller.
	  nevent = NavEstopEvent::None;
	}
    }
  else
    {
      // event pending from previous cycle
      pending_event = NavEstopEvent::None; // no longer pending
    }
  return nevent;
}

void Estop::reset(void)
{
  trace_reset("Estop");
  reset_me();
  halt->reset();
  run->reset();
}

void Estop::reset_me(void)
{
  state = NavEstopState();		// initial state
  pending_event = NavEstopEvent::None;
}


//////////////////////////////////////////////////////////////////////
// state transition action methods
//////////////////////////////////////////////////////////////////////

Controller::result_t Estop::ActionError(pilot_command_t &pcmd)
{
  ROS_FATAL("Invalid Navigator E-stop event %s, state %s",
            event.Name(), prev.Name());
  halt->control(pcmd);
  pending_event = NavEstopEvent::Abort;
  return NotImplemented;
}

// steady state actions

Controller::result_t Estop::ActionInDone(pilot_command_t &pcmd)
{
  navdata->flasher = false;
  navdata->alarm = false;
  return halt->control(pcmd);
}  

Controller::result_t Estop::ActionInPause(pilot_command_t &pcmd)
{
  navdata->flasher = true;
  navdata->alarm = false;
  return halt->control(pcmd);
}

Controller::result_t Estop::ActionInRun(pilot_command_t &pcmd)
{
  navdata->flasher = true;
  navdata->alarm = true;
  result_t result = run->control(pcmd);
  if (result == NotImplemented || 
      result == NotApplicable)
    pending_event = NavEstopEvent::Abort;
  return result;
}

Controller::result_t Estop::ActionInSuspend(pilot_command_t &pcmd)
{
  navdata->flasher = true;
  navdata->alarm = false;
  return halt->control(pcmd);
}

// state entry actions

Controller::result_t Estop::ActionToAbort(pilot_command_t &pcmd)
{
  ART_MSG(1, "Robot disabled!");
  return ActionInDone(pcmd);
}

Controller::result_t Estop::ActionToDone(pilot_command_t &pcmd)
{
  ART_MSG(1, "Mission accomplished!");
  return ActionInDone(pcmd);
}

Controller::result_t Estop::ActionToPause(pilot_command_t &pcmd)
{
  ART_MSG(1, "Robot pausing!");
  return ActionInPause(pcmd);
}

Controller::result_t Estop::ActionToRun(pilot_command_t &pcmd)
{
  /// @todo implement 5 second hesitation before actually starting
  ART_MSG(1, "Robot running!");
  return ActionInRun(pcmd);
}

Controller::result_t Estop::ActionToSuspend(pilot_command_t &pcmd)
{
  ART_MSG(1, "Autonomous operation suspended!");
  // reset last_waypt, so commander will Initialize again for next Run 
  navdata->last_waypt = ElementID().toMapID();
  return ActionInSuspend(pcmd);
}
