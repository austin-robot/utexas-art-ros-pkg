/*
 *  Navigator stop line controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <art_msgs/ArtVehicle.h>
using art_msgs::ArtVehicle;

#include <art/DARPA_rules.h>
#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "stop_line.h"

/**
 *  This is based on the unpublished "Control Tutorial" draft dated
 *  January 26, 2004 by Dr. Benjamin Kuypers, section 5: "The Stopping
 *  Controller".  He recommends a constant deceleration instead of the
 *  simpler exponential decay.  The dynamical system is:
 * 
 * 	x_dot = -k * sqrt(x)
 * 
 *  Solving analytically with initial condition x(0) = D and v(0) = V
 *  yields these equations of motion:
 * 
 * 	x(t) = (sqrt(D) - V*t/(2*sqrt(D)))**2	(parabolic drop)
 * 	v(t) = dx/dt = (V**2/2*D)*t + V		(linear velocity)
 * 	a(t) = dv/dt = V**2/(2*D) = A		(constant deceleration)
 * 
 *  Note that the initial velocity V is negative in these equations,
 *  because it represents motion from positive x to zero.  The system
 *  stops in finite time T = -2*D/V, with x(T) = 0, and v(T) = 0.
 * 
 *  For example, when D = 10m from stop line and V = -5m/s, the
 *  vehicle stops in 4 seconds at a constant 1.25m/s/s deceleration.
 */

StopLine::StopLine(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  reset_me();
};

StopLine::~StopLine() {};

// configuration method
void StopLine::configure()
{
  min_stop_distance = config_->min_stop_distance;
  stop_creep_speed = config_->stop_creep_speed;
  max_creep_distance = config_->max_creep_distance;
  stop_deceleration = config_->stop_deceleration;
  stop_distance = config_->stop_distance;
  stop_latency = config_->stop_latency;

#if 0
  ros::NodeHandle nh("~");

  nh.param("min_stop_distance", min_stop_distance, 5.0);
  ROS_INFO("minimum distance to begin stopping is %.3f m", min_stop_distance);

  nh.param("stop_creep_speed", stop_creep_speed, 1.0);
  ROS_INFO("speed while creeping forward is %.3f m/s", stop_creep_speed);

  nh.param("max_creep_distance", max_creep_distance,
           (double) ArtVehicle::length);
  ROS_INFO("distance in which creep applies is %.3f m", max_creep_distance);

  nh.param("stop_deceleration", stop_deceleration, 0.2);
  ROS_INFO("desired stopping deceleration is %.3f m/s/s", stop_deceleration);

  // Distance from front bumper to stop.  Give it a small overshoot to
  // aim for point just beyond the actual stop line.  When the stop
  // polygon is reached, the controller will request full brake, while
  // the car is going slowly.
  nh.param("stop_distance", stop_distance,
           DARPA_rules::stop_line_to_bumper+1.0);
  ROS_INFO("desired stopping distance is %.3f m", stop_distance);

  // stop_latency compensates for latency in the braking system
  nh.param("stop_latency", stop_latency, 1.5);
  ROS_INFO("stopping latency is %.3f sec", stop_latency);
#endif
};

/** Set speed for steady deceleration to stop way-point
 *
 * @pre
 *	course->stop_waypt is goal way-point of stop
 * @param pcmd contains desired heading and speed, assuming it is not
 *	       yet time to stop, updated on exit
 * @param topspeed velocity limit
 * @post
 *	resets course->stop_waypt.id if reached.
 * @return
 *	OK if in process of stopping;
 *	Finished if stop way-point reached.
 */
Controller::result_t StopLine::control(pilot_command_t &pcmd,
				       float topspeed)
{
  result_t result = OK;

  // distance from front bumper to stop way-point
  float wayptdist =
    (Euclidean::DistanceToWaypt(MapPose(estimate->pose.pose),
                                course->stop_waypt)
     - ArtVehicle::front_bumper_px + stop_distance);
  // stop_latency compensates for latency in the braking system.
  float abs_speed = fabsf(estimate->twist.twist.linear.x);
  float latencydist = abs_speed * stop_latency;
  float D = wayptdist - latencydist;

  // According to the model, deceleration should be constant, but in
  // the real world control latency will cause it to vary, so apply
  // feedback and recompute the model every cycle.
  if (abs_speed < Epsilon::speed)
    abs_speed = 0.0;

  float V = -abs_speed;
  float A = V*V/(2.0*D);
  ElementID stop_id = course->stop_waypt.id;
  
  // see if it is time to begin stopping
  if ((D <= min_stop_distance || A >= stop_deceleration)
      && !stopping)
    {
      stopping = true;
      initial_speed = fmaxf(topspeed,abs_speed);
      ART_MSG(8, "begin stopping for waypoint %s",
	      stop_id.name().str);
    }

  // Once stopping is initiated, keep doing it until reset(), no
  // matter how V changes.
  if (stopping)
    {

      // check whether front bumper is within stop way-point polygon
      Polar front_bumper(0.0, ArtVehicle::front_bumper_px);

      if (D <= stop_distance ||
	  pops->pointInPoly(front_bumper, MapPose(estimate->pose.pose),
                            course->stop_poly))
	{
	  pcmd.velocity = 0.0;		// halt immediately.

	  // report Finished when stopped within stop polygon
	  if (abs_speed < Epsilon::speed)
	    {
	      // Consider this way-point "reached".
	      course->new_waypoint_reached(stop_id);
	      course->stop_waypt.id = ElementID(); // reset way-point
	      result = Finished;
	    }
	}
      else if (!creeping && abs_speed >= Epsilon::speed)
	{
	  // Not there yet.  Do not let the requested speed increase
	  // above initial speed when stopping began, even when the
	  // car was accelerating.
	  pcmd.velocity = fminf(pcmd.velocity, abs_speed - A);
	  pcmd.velocity = fminf(pcmd.velocity, initial_speed);
	  if (pcmd.velocity < 0.0)
	    pcmd.velocity = 0.0;	// do not change direction
	}
      else if (D <= max_creep_distance)
	{
	  creeping=true;
	  // stopped too soon, keep creeping forward
	  pcmd.velocity = fminf(pcmd.velocity, stop_creep_speed);
	}

      if (verbose >= 2)
	{
	  ART_MSG(8, "stop %.3f m away for waypoint %s (%.3f,%.3f),"
		  " %.3f m latency",
		  wayptdist-stop_distance, stop_id.name().str,
		  course->stop_waypt.map.x, course->stop_waypt.map.y,
		  latencydist);
	  ART_MSG(8, "current, desired speed %.3f m/s, %.3f m/s,"
		  " decel %.3f m/s/s",
		  abs_speed, pcmd.velocity, A);
	}
    }

  trace("stop_line controller", pcmd, result);
  return result;
};

// reset controller and any subordinates
void StopLine::reset(void)
{
  trace_reset("StopLine");
  reset_me();
};

// reset this controller only
void StopLine::reset_me(void)
{
  stopping = false;
  creeping = false;
  initial_speed = 0.0;
};
