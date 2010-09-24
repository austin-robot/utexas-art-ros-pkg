/* -*- mode: C++ -*-
 *
 *  Commander finite state machine implementation
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 *
 *  Author: Jack O'Quin and Patrick Beeson
 */

#include "command.h"
#include "FSM.h"

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
//   return control();
//
// This will call current_event() to prioritize all events, returning
// the most urgent.  It will check any timers that are running; timer
// expirations are one set of possible events.  The event priorities
// are independent of state.
//
// The control() method updates the current state, then calls the
// transition-dependent action method from the state transition table.
// Every action method returns a Commander order for this cycle.
//
// Commander could do multiple state transitions in a single cycle.
// Since control() performs the state change before calling the action
// method, in some cases that method might trigger another state
// transition, if necessary.  Currently they do not, which is simpler.


CmdrFSM::CmdrFSM(Commander *cmdr_ptr, int verbosity)
{
  verbose = verbosity;
  cmdr = cmdr_ptr;

  // initialize transition table
  for (int event = 0; event < (int) CmdrEvent::N_events; ++event)
    for (int state = 0; state < (int) CmdrState::N_states; ++state)
      {
	transtion_t *xp = &ttable[event][state];
	xp->action = &CmdrFSM::ActionError;
	xp->next = (CmdrState::state_t) state;
      }

  // initialize transition table:

  Add(CmdrEvent::Blocked,	&CmdrFSM::ActionInDone,
      CmdrState::Done,		CmdrState::Done);
  Add(CmdrEvent::Blocked,	&CmdrFSM::ActionInInit,
      CmdrState::Init,		CmdrState::Init);
  Add(CmdrEvent::Blocked,	&CmdrFSM::BlockedInRoad,
      CmdrState::Road,		CmdrState::Road);

  Add(CmdrEvent::Done,		&CmdrFSM::ActionInDone,
      CmdrState::Done,		CmdrState::Done);
  Add(CmdrEvent::Done,		&CmdrFSM::ActionToDone,
      CmdrState::Init,		CmdrState::Done);
  Add(CmdrEvent::Done,		&CmdrFSM::ActionToDone,
      CmdrState::Road,		CmdrState::Done);

  Add(CmdrEvent::EnterLane,	&CmdrFSM::ActionInDone,
      CmdrState::Done,		CmdrState::Done);
  Add(CmdrEvent::EnterLane,	&CmdrFSM::InitToRoad,
      CmdrState::Init,		CmdrState::Road);
  Add(CmdrEvent::EnterLane,	&CmdrFSM::ActionInRoad,
      CmdrState::Road,		CmdrState::Road);

  Add(CmdrEvent::Fail,		&CmdrFSM::ActionInDone,
      CmdrState::Done,		CmdrState::Done);
  Add(CmdrEvent::Fail,		&CmdrFSM::ActionFail,
      CmdrState::Init,		CmdrState::Done);
  Add(CmdrEvent::Fail,		&CmdrFSM::ActionFail,
      CmdrState::Road,		CmdrState::Done);


  Add(CmdrEvent::None,		&CmdrFSM::ActionInDone,
      CmdrState::Done,		CmdrState::Done);
  Add(CmdrEvent::None,		&CmdrFSM::ActionInInit,
      CmdrState::Init,		CmdrState::Init);
  Add(CmdrEvent::None,		&CmdrFSM::ActionInRoad,
      CmdrState::Road,		CmdrState::Road);

  Add(CmdrEvent::Wait,		&CmdrFSM::ActionInDone,
      CmdrState::Done,		CmdrState::Done);
  Add(CmdrEvent::Wait,		&CmdrFSM::ActionInInit,
      CmdrState::Init,		CmdrState::Init);
  Add(CmdrEvent::Wait,		&CmdrFSM::ActionWait,
      CmdrState::Road,		CmdrState::Road);

  Add(CmdrEvent::Replan,	&CmdrFSM::ActionInDone,
      CmdrState::Done,		CmdrState::Done);
  Add(CmdrEvent::Replan,	&CmdrFSM::ActionInInit,
      CmdrState::Init,		CmdrState::Init);
  Add(CmdrEvent::Replan,	&CmdrFSM::ReplanInRoad,
      CmdrState::Road,		CmdrState::Road);
}

// add a transition to the table
void CmdrFSM::Add(CmdrEvent::event_t event, action_t action,
		  CmdrState::state_t from_state, CmdrState::state_t to_state)
{
  transtion_t *xp = &ttable[event][from_state];
  xp->action = action;
  xp->next = to_state;
}

art_msgs::Order CmdrFSM::control(const art_msgs::NavigatorState *_navstate)
{
  navstate = *_navstate;

  // get highest priority current event
  CmdrEvent event = current_event();

  // state transition structure pointer
  transtion_t *xp = &ttable[event.Value()][state.Value()];

  // do state transition
  prev = state;
  state = xp->next;
  if ((state != prev.Value()))
    ROS_INFO_STREAM("Commander state changing from " << prev.Name()
                     << " to " << state.Name()
                     << ", event = " << event.Name());

  // perform transition action, returning next order
  action_t action = xp->action;
  return (this->*action)(event);
}

// return most urgent current event
//
// entry:
//	Navigator in Run state.
//	Route initialized.
//	navstate points to current Navigator state message
//
// events with lower numeric values have priority
// less urgent events must remain pending
//
CmdrEvent CmdrFSM::current_event()
{
  
  // Route is only 0 before we have ever made a plan.
  if (cmdr->route->size()==0)
    {
      current_way = ElementID(navstate.last_waypt);

      // It's entirely possible that the starting point is our first
      // goal, if so, check it off now.
      if (current_way == cmdr->goal.id)
	cmdr->next_checkpoint();
      // needed to get from Init to Road state
      ROS_DEBUG("Calling EnterLane event");
      return CmdrEvent::EnterLane;
    }
  
  bool new_goal1 = false;
  bool new_goal2 = false;

  // Walk through plan ticking off states until we see last_waypt.
  // Mark if we've passed any goals along the way.
  if (ElementID(navstate.last_waypt) != current_way)
    {
      ElementID old_way=current_way;

      WayPointEdge first_edge;      

      do {

	// Get next edge from plan.  Usually last_waypt is the first node
	// in the edge.
	cmdr->route->pop_front();
	first_edge=cmdr->route->at(0);
	WayPointNode* current_node=
	  cmdr->graph->get_node_by_index(first_edge.startnode_index);
	if (current_node == NULL)
	  {
	    ROS_ERROR_STREAM("node " << first_edge.startnode_index
                             << " is not in the RNDF graph");
	    return CmdrEvent::Fail;
	  }
	
	current_way=current_node->id;

	// Check to see if we passed a goal recently
	if (current_way == cmdr->goal.id)
	  new_goal1=true;

	if (current_way == cmdr->goal2.id)
	  new_goal2=true;
	
      } while (ElementID(navstate.last_waypt) != current_way &&
	       cmdr->route->size() > 1);
      
      // If never found matching waypoint in plan, then its the last
      // thing in the plan.
      if (ElementID(navstate.last_waypt) != current_way)
	{
	  WayPointNode* current_node=
	    cmdr->graph->get_node_by_index(first_edge.endnode_index);
	  if (current_node == NULL)
	    {
	      ROS_ERROR_STREAM("node " << first_edge.endnode_index
                               << " is not in the RNDF graph");
	      return CmdrEvent::Fail;
	    }
	  
	  current_way=current_node->id;

	  // Check to see if we passed a goal recently
	  if (current_way == cmdr->goal.id)
	    new_goal1=true;
	  
	  if (current_way == cmdr->goal2.id)
	    new_goal2=true;
	}	

      ROS_INFO_STREAM("current waypoint changed from "
                      << old_way.name().str
                      << " to " << current_way.name().str);
    }
  
  bool finished=false;
  
  // Find new goals if we've passed one recently.
  if (new_goal1)
    finished = !cmdr->next_checkpoint();
  if (new_goal1 && new_goal2)
    finished = !cmdr->next_checkpoint();

  CmdrEvent event(CmdrEvent::None);	// default event
  
  // Process events in order of urgency.  Use only the first, leaving
  // the rest pending.  Review these priorities carefully.
  if (finished)
    {
      // no more checkpoints
      event = CmdrEvent::Done;
    }
  // Check for Blocked before normal replanning at goal
  else if (ElementID(navstate.replan_waypt) != old_replan)
    {
      old_replan=navstate.replan_waypt;
      if (ElementID(navstate.replan_waypt) != ElementID())
	{
	  if (navstate.road_blocked)
	    event = CmdrEvent::Blocked;
	  else event = CmdrEvent::Replan;
	}
    }
  else if (new_goal1 && !cmdr->replan_route())
    {
      // needed to re-plan, but could not
      event = CmdrEvent::Wait;
    }
  
  // log event selected and input states
  ROS_DEBUG_STREAM("Current event = " << event.Name());
  
  return event;
}


//////////////////////////////////////////////////////////////////////
// state transition action methods
//////////////////////////////////////////////////////////////////////

// error actions

art_msgs::Order CmdrFSM::ActionError(CmdrEvent event)
{
  ROS_ERROR_STREAM("Invalid Commander event " << event.Name()
                   << " in state " << prev.Name());
  return ActionFail(event);
}

art_msgs::Order CmdrFSM::ActionFail(CmdrEvent event)
{
  ROS_ERROR("Mission failure!");
  art_msgs::Order abort_order;
  abort_order.behavior.value = art_msgs::Behavior::Abort;
  return abort_order;

}

art_msgs::Order CmdrFSM::ActionWait(CmdrEvent event)
{
  ROS_INFO_THROTTLE(10, "No replan.  Just wait it out.");
  return cmdr->prepare_order(art_msgs::Behavior::Go);
}


// steady state actions

art_msgs::Order CmdrFSM::ActionInDone(CmdrEvent event)
{
  art_msgs::Order done_order;
  done_order.behavior.value = art_msgs::Behavior::Quit;
  return done_order;
}  

art_msgs::Order CmdrFSM::ActionInInit(CmdrEvent event)
{
  art_msgs::Order init_order;
  init_order.behavior.value = art_msgs::Behavior::Initialize;
  return init_order;
}  

art_msgs::Order CmdrFSM::ActionInRoad(CmdrEvent event)
{
  // prepare order for navigator driver
  return cmdr->prepare_order(art_msgs::Behavior::Go);
}  


// state entry actions

art_msgs::Order CmdrFSM::ActionToDone(CmdrEvent event)
{
  ROS_INFO("Mission completed!");
  return ActionInDone(event);
}

art_msgs::Order CmdrFSM::ActionToRoad(CmdrEvent event)
{
  ROS_INFO("On the road.");
  return ActionInRoad(event);
}

// re-planning transitions

art_msgs::Order CmdrFSM::BlockedInRoad(CmdrEvent event)
{
  ROS_INFO("Road blocked, making a new plan.");

  cmdr->blockages->add_block(navstate.replan_waypt);

  if (cmdr->replan_route() == false)
    return ActionWait(event);
  return ActionInRoad(event);
}


art_msgs::Order CmdrFSM::ReplanInRoad(CmdrEvent event)
{
  ROS_INFO("Making new plan.");
  
  navstate.last_waypt=navstate.replan_waypt;

  if (cmdr->replan_route() == false)
    return ActionWait(event);
  return ActionInRoad(event);
}


art_msgs::Order CmdrFSM::InitToRoad(CmdrEvent event)
{
  ROS_INFO("On the road, making initial plan.");
  
  if (cmdr->replan_route() == false)
    return ActionFail(event);

  return ActionInRoad(event);
}
