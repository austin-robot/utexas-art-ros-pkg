/* -*- mode: C++ -*-
 *
 *  Navigator class interface
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __NAVIGATOR_INTERNAL_H__
#define __NAVIGATOR_INTERNAL_H__

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <art/epsilon.h>
#include <art/error.h>
#include <art/infinity.h>

#include <art_msgs/ArtHertz.h>
#include <art_map/euclidean_distance.h>
#include <art_map/PolyOps.h>

#include <art_msgs/Behavior.h>
#include <art_msgs/NavigatorCommand.h>
#include <art_msgs/NavigatorState.h>
#include <art_msgs/ObservationArray.h>
#include <nav_msgs/Odometry.h>

#include <art_nav/NavBehavior.h>

#include "art_nav/NavigatorConfig.h"
typedef art_nav::NavigatorConfig Config;

// Provide short names for some messages so they can more easily be
// packaged differently.
using art_msgs::Behavior;
using art_msgs::NavigatorCommand;
using art_msgs::NavigatorState;
using art_msgs::Observation;
using art_msgs::ObservationArray;
using nav_msgs::Odometry;
using art_msgs::Order;

typedef struct
{
  float velocity;
  float yawRate;
} pilot_command_t;

// forward class declarations
class Estop;
class Course;
class Obstacle;

class Navigator
{
public:

  // helper classes
  PolyOps* pops;			// polygon operations class
  Course* course;			// course planning class
  Obstacle* obstacle;			// obstacle class

  // subordinate controllers
  Estop	*estop;

  // public data used by controllers
  art_msgs::Order order;               // current commander order
  art_msgs::NavigatorState navdata;    // current navigator state data
  nav_msgs::Odometry estimate;         // estimated control position
  nav_msgs::Odometry *odometry;

  // public methods
  Navigator(nav_msgs::Odometry *odom_msg);
  ~Navigator();

  // configure parameters
  void configure();

  Config config_;                       // configuration parameters

  // decrease pilot command velocity, obeying min_speed
  void reduce_speed_with_min(pilot_command_t &pcmd, float new_speed)
  {
    pcmd.velocity = fminf(pcmd.velocity,
			  fmaxf(order.min_speed, new_speed));
  }

  // main navigator entry point -- called once every cycle
  pilot_command_t navigate(void);

  // trace controller state
  void trace_controller(const char *name, pilot_command_t &pcmd)
  {
    if (verbose >= 4)
      ART_MSG(7, "%s: pcmd = (%.3f, %.3f) ",
	      name, pcmd.velocity, pcmd.yawRate);
  }

private:
  int verbose;				// log message verbosity
};

#endif // __NAVIGATOR_INTERNAL_H__
