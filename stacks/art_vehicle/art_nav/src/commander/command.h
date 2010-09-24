/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2009, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 *
 *  Author: Patrick Beeson, Jack O'Quin
 */
#ifndef __command_h__
#define __command_h__

#include <ros/ros.h>

#include <art_map/zones.h>

#include <art_nav/NavBehavior.h>
#include <art_nav/Mission.h>
#include <art_msgs/NavigatorCommand.h>
#include <art_msgs/NavigatorState.h>

#include "Blockage.h"
#include "Path.h"
#include "Event.h"

class CmdrFSM;

class Commander
{
  friend class CmdrFSM;			// state machine is a friend

 public:

  Commander(int verbosity, double limit, Graph* _graph, Mission* _mission,
	    const ZonePerimeterList& _zones);
  ~Commander();
  art_msgs::Order command(const art_msgs::NavigatorState &cur_navstate);

 private:
  int verbose;

  CmdrFSM* fsm;

  Graph* graph;
  Mission* mission;

  Blockages* blockages;

  ElementID current_way;
  const art_msgs::NavigatorState *navstate; // current Navigator state
  art_msgs::Order order;

  WayPointNode goal;			// next checkpoint goal
  WayPointNode goal2;			// following checkpoint

  ZonePerimeterList zones;

  Path* route;

  float speedlimit;
  int replan_num;

  // private methods:

  // return most urgent current event
  CmdrEvent current_event(void);

  // get next checkpoint, return true if any remain
  bool next_checkpoint(void);

  // prepare next Navigator order
  art_msgs::Order prepare_order(art_msgs::Behavior::_value_type behavior);

  // replan route, return true if successful
  bool replan_route();

  // set immediate checkpoint goals from mission
  void set_checkpoint_goals(void);
};


#endif
