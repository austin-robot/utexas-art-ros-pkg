/*
 *  Navigator class interface
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <assert.h>
//#include <art/DARPA_rules.h>

#include "navigator_internal.h"
#include "course.h"

/** @todo add ROS-style obstacle detection */
//#include "obstacle.h"

// subordinate controller classes
#include "estop.h"

Navigator::Navigator()
{
  order.behavior.value = art_nav::Behavior::Pause; // initial order

  // set navigator state flags to false, current polygon none
  navdata.cur_poly = -1;
  navdata.lane_blocked = false;
  navdata.stopped = false;
  navdata.road_blocked = false;
  navdata.reverse = false;
  navdata.have_zones = false;

  odometry = new nav_msgs::Odometry();
  // allocate helper classes
  pops = new PolyOps();
  course = new Course(this, verbose);
  //obstacle = new Obstacle(this, verbose);

  // allocate controller classes
  estop = new Estop(this, verbose);
};

Navigator::~Navigator()
{
  // controller classes
  delete estop;

  // free helper classes
  delete course; 
  delete pops;
  delete odometry;
  //delete obstacle;
};

// main navigator entry point -- called once every driver cycle
//
//  Each order contains a behavior which determines the navigator
//  state for this cycle.  All return a pilot_command_t with the control
//  output for this cycle.
//
pilot_command_t Navigator::navigate(void)
{
  pilot_command_t pcmd;			// pilot command to return

  // report whether odometry reports vehicle currently stopped
  navdata.stopped = (fabsf(odometry->twist.twist.linear.x)
                     < Epsilon::speed);

  // run top-level (E-stop) state machine controller
  estop->control(pcmd);

  // copy last commander order to navigator state message -- it may
  // have been updated due to way-points passed
  navdata.last_order = order;

  return pcmd;
}

// configure .cfg variables -- called by driver constructor
void Navigator::configure()
{
  ROS_INFO("Navigator configuration options:");

  // configure controller methods
  estop->configure();
  course->configure();
  //obstacle->configure();
}

