/*
 * Navigator class interface
 *
 *  Copyright (C) 2007 Austin Robot Technology
 *  All Rights Reserved. Licensed Software.
 *
 *  This is unpublished proprietary source code of Austin Robot
 *  Technology, Inc.  The copyright notice above does not evidence any
 *  actual or intended publication of such source code.
 *
 *  PROPRIETARY INFORMATION, PROPERTY OF AUSTIN ROBOT TECHNOLOGY
 *
 *  If this is ever released publicly, the requirements of the GPL
 *  will apply, due to Player header and library dependencies.
 *
 *  $Id$
 *
 * Authors: Ryan Madigan, Jack O'Quin
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

#include <art_common/ArtHertz.h>
#include <art_map/euclidean_distance.h>
#include <art_map/PolyOps.h>

#include <art_nav/NavigatorCommand.h>
#include <art_nav/NavigatorState.h>
#include <art_nav/Behavior.h>
#include <art_nav/NavBehavior.h>

//TODO create art_observers package with this interface
#include <art_nav/odometry.h>

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
  art_nav::Order order;			// current commander order
  art_nav::NavigatorState navdata;      // current navigator state data
  nav_msgs::Odometry estimate;          // estimated control position
  
  Odometry* odometry;

  // public methods
  Navigator();
  ~Navigator();

  // configure parameters
  void configure();

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
