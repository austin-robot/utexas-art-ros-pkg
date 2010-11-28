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

#include "obstacle.h"

// subordinate controller classes
#include "estop.h"

/** Main ART navigator class.

    The Navigator class instantiates some infrastructure and the
    top-level (Estop) controller. Then, on each cycle it runs the
    Estop controller, which indirectly invokes other controllers when
    appropriate.

   @todo Add ROS-style obstacle detection.
*/
Navigator::Navigator(nav_msgs::Odometry *odom_msg)
{
  odometry = odom_msg;

  order.behavior.value = art_msgs::Behavior::Pause; // initial order

  // set navigator state flags to false, current polygon none
  navdata.cur_poly = -1;
  navdata.lane_blocked = false;
  navdata.stopped = false;
  navdata.road_blocked = false;
  navdata.reverse = false;
  navdata.have_zones = false;
  art_msgs::MapID null_waypt = ElementID().toMapID();
  navdata.last_waypt = null_waypt;
  navdata.replan_waypt = null_waypt;

  // allocate helper classes
  pops = new PolyOps();
  course = new Course(this, verbose);
  obstacle = new Obstacle(this, verbose);

  // allocate controller classes
  estop = new Estop(this, verbose);
};

Navigator::~Navigator()
{
  // controller classes
  delete estop;

  // free helper classes
  delete obstacle;
  delete course; 
  delete pops;
};

/** Main navigator entry point -- called once every driver cycle
 *
 *  The order contains a behavior which affects the navigator state
 *  for this cycle.
 *
 *  @return a pilot_command_t with control output for this cycle.
 */
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

/** Configure parameters */
void Navigator::configure()
{
  course->configure();
}

