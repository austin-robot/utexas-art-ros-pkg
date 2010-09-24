/*
 *  ART vehicle commander class definition.
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 * $Id$
 */

#include <art_nav/GraphSearch.h>
#include <art_nav/NavEstopState.h>

#include "command.h"
#include "FSM.h"

Commander::Commander(int verbosity, double limit, 
		     Graph* _graph, Mission* _mission,
		     const ZonePerimeterList& _zones) 
{
  route = new Path();
  route->clear();
  speedlimit = limit;
  zones = _zones;
  verbose = verbosity;
  fsm = new CmdrFSM(this, verbose);
  graph = _graph;
  mission = _mission;
  blockages = new Blockages(graph, route);
  set_checkpoint_goals();
  replan_num = 0;
}

Commander::~Commander()
{
  delete blockages;
  delete fsm;
  delete route;
}



/** main command entry point -- called once per cycle
 *
 * @param _navstate has current Navigator state
 * @return next Navigator order, finished when behavior is DONE.
*/
art_msgs::Order Commander::command(const art_msgs::NavigatorState &_navstate)
{
  navstate = &_navstate;       // save state pointer in class variable

  order = art_msgs::Order();             // begin with empty order
  //order.max_speed = 0.0;
  //order.min_speed = 0.0;

  // handle initial startup sequence
  if (navstate->estop.state != art_msgs::EstopState::Run)
    {
      // do nothing if Navigator not running
      ROS_DEBUG("Waiting for Navigator to enter Run state.");
      order.behavior.value = NavBehavior::None;
      return order;
    }
  else if (ElementID(navstate->last_waypt) == ElementID())
    {
      // initialize navigator if last_waypt not set yet
      ROS_DEBUG("Waiting for Navigator to find initial way-point.");
      order.behavior.value = NavBehavior::Initialize;
      return order;
    }

  // determine most urgent event, and perform state transition for it,
  // returning next command
  return fsm->control(navstate);
}

////////////////////////////////////////////////////////////////////
// Private methods
////////////////////////////////////////////////////////////////////

// get next checkpoint, return true if any remain
//
// updates:
//	goal
//	goal2
//
// Called when the current checkpoint has just been reached.
//
// An MDF could repeat a checkpoint one or more times, but Commander
// does not handle that gracefully, so the parser removes duplicates.
//
bool Commander::next_checkpoint(void)
{


  // remove the former checkpoint from the head of the queue
  bool retval=mission->nextPoint();

  mission->save("mission_state");  
  
  
  if (!retval)				// none left?
    return false;			// done with mission

  set_checkpoint_goals();

  return true;
}

// prepare next Navigator order
//
// on entry: route contains one or more waypoints starting at current one
//
art_msgs::Order Commander::prepare_order(art_msgs::Behavior::_value_type behavior)
{
  order.behavior.value = behavior;
  ROS_DEBUG_STREAM("order.behavior = " << NavBehavior(order.behavior).Name());
  ROS_DEBUG_STREAM("goal = "<<goal.id.name().str);

  // include next two checkpoints for monitoring purposes
  order.chkpt[0] = goal.toWayPoint();
  order.chkpt[1] = goal2.toWayPoint();

  for (unsigned i = 0; i < art_msgs::Order::N_WAYPTS; ++i)
    {
      WayPointEdge curr_edge;
      
      WayPointNode* next_node;
      
      if (i==0) {
	curr_edge=route->at(0);
	
	if (curr_edge.distance<0) {
	  ROS_FATAL("Route is completely empty..");
	  order.behavior.value = NavBehavior::Abort;
	  return order;
	}

	order.min_speed=fmin(speedlimit,curr_edge.speed_min);
	order.max_speed=fmin(speedlimit,curr_edge.speed_max);      
	
	next_node=graph->get_node_by_index(curr_edge.startnode_index);	
      }
      else {
	curr_edge=route->at(i-1);
	
	if (curr_edge.distance<0) {
	  ROS_FATAL("Route is completely empty..");
	  order.behavior.value = NavBehavior::Abort;
	  return order;
	}
	
	order.waypt[i-1].is_lane_change=curr_edge.is_implicit;
	
	next_node=graph->get_node_by_index(curr_edge.endnode_index);
      }
      
      if (next_node==NULL)
	{
	  ROS_WARN_STREAM("plan waypt (id: " << next_node->id.name().str
                          << ") is not in the RNDF graph");
	  order.behavior.value = NavBehavior::Abort;
	  return order;
	}
     
      order.waypt[i] = next_node->toWayPoint();

      if (ElementID(order.waypt[i].id) == goal.id)
	{
	  // this is a goalpoint
	  order.waypt[i].is_goal = true;
	  if (ElementID(order.waypt[i].id) == goal2.id) // last goalpoint?
	    order.waypt[i].is_stop = true; // tell navigator to stop there
	}
      
      ROS_DEBUG_STREAM("waypt[" << i << "] = "
                       << ElementID(order.waypt[i].id).name().str
                       << " (" << order.waypt[i].mapxy.x
                       << ", " << order.waypt[i].mapxy.y
                       << ") E" << order.waypt[i].is_entry
                       << ", G" << order.waypt[i].is_goal
                       << ", P" << order.waypt[i].is_spot
                       << ", S" << order.waypt[i].is_stop
                       << ", X" << order.waypt[i].is_exit
                       << ", Z" << order.waypt[i].is_perimeter);
    }
  
  for (uint i=0; i<zones.size(); i++)
    if (zones[i].zone_id==order.waypt[0].id.seg &&
	zones[i].zone_id==order.waypt[1].id.seg)
      order.max_speed=fmin(speedlimit,zones[i].speed_limit);

  ROS_DEBUG_STREAM("Max speed = " << order.max_speed << " m/s");
  ROS_DEBUG_STREAM("Min speed = " << order.min_speed << " m/s");

  order.replan_num=replan_num;
  order.next_uturn=-1;

  for (unsigned i = 0; i < art_msgs::Order::N_WAYPTS-1; ++i)
    if (order.waypt[i].id.seg == order.waypt[i+1].id.seg
	&& order.waypt[i].id.lane != order.waypt[i+1].id.lane &&
	!order.waypt[i].is_lane_change)
      {
	order.next_uturn=i;
	break;
      }

  ROS_DEBUG_STREAM("Next uturn = " << order.next_uturn);
  ROS_DEBUG_STREAM("Replan num = " << order.replan_num);
  
  return order;
}

// replan route, return true if successful
bool Commander::replan_route()
{
  replan_num++;

  // get current way-point node
  WayPointNode *current;
  
  current= graph->get_node_by_id(navstate->last_waypt);
  if (current == NULL)
    {
      ROS_WARN_STREAM("last_waypt "
                      << ElementID(navstate->last_waypt).name().str
                      << " is not in the RNDF graph");
      return false;
    }
  
  // We've already checked for this in FSM::current_event(), if
  // still true its the end of the run.
  if (current->index==goal.index)
    return true;

  // call A* from current to goal
  WayPointEdgeList edges =
    GraphSearch::astar_search(*graph, current->index, goal.index, speedlimit);
    
  // Edges will be empty if we are planning inside a zone
  if (edges.empty()) // no route?
    {
      if (!blockages->empty())
	{
	  ROS_ERROR("No path found. Removing blockage and trying again");
	  blockages->pop_oldest();
	  replan_num--;
	  return replan_route();
	}
      else 
	{ 
	  ROS_ERROR("No path found to next checkpoint");
	  ROS_ERROR_STREAM(" Attempted to find a path between "
                           << current->id.name().str << " and "
                           << goal.id.name().str);
	  return false;
	}
    }
  
  Path new_route;

  new_route.new_path(current->index,goal.index,edges);
  
  
  if (goal2.index != goal.index) {
    
    edges = GraphSearch::astar_search(*graph, goal.index, goal2.index,
				      speedlimit);
    
    if (edges.empty())		// no route?
      {
	if (!blockages->empty())
	  {
	    ROS_ERROR("No path found. Removing blockage and trying again");
	    blockages->pop_oldest();
	    replan_num--;
	    return replan_route();
	  }
	else 
	  { 
	    ROS_ERROR_STREAM("No path found to next checkpoint");
	    ROS_ERROR_STREAM("Attempted to find a path between "
                             << current->id.name().str << " and "
                             << goal.id.name().str);
	    return false;
	  }
      }
    
  
    new_route.append_path(goal.index, goal2.index, edges);
  }

  *route=new_route;


  // optionally print the entire route path
  if (verbose)
    route->print(*graph);
  
  return true;
}

// set immediate checkpoint goals from mission
//
// exit:
//	goal = WayPointNode of next checkpoint
//	goal2 = following checkpoint if any, immediate goal otherwise
//
void Commander::set_checkpoint_goals(void)
{
  ElementID goal_id = mission->current_checkpoint_elementid();
  WayPointNode *goal_ptr = graph->get_node_by_id(goal_id);
  if (goal_ptr)
    {
      goal = *goal_ptr;
      if (mission->checkpoint_elementid.size() > 1)
	goal2 = *graph->get_node_by_id(mission->next_checkpoint_elementid());
      else
	goal2 = goal;
      
      ROS_DEBUG_STREAM("goal checkpoints: "
                       << goal.id.name().str << ", "
                       << goal2.id.name().str);
    }
  else
    {
      ROS_WARN_STREAM("bad goal checkpoint: "
                      << goal_id.name().str
                      << ", no such way-point node.");
    }
}
