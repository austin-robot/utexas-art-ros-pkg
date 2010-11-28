/*
 *  Navigator stop controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <art_msgs/ArtVehicle.h>
using art_msgs::ArtVehicle;

#include "navigator_internal.h"
#include "Controller.h"
#include "stop.h"

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
 *
 *  @todo Reconcile this with the StopLine controller, which has an
 *        incompatible interface.
 */

Stop::Stop(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  reset();
};

Stop::~Stop() {};

Controller::result_t Stop::control(pilot_command_t &pcmd)
{
  ART_ERROR("Stop::control() requires a float distance parameter,");
  ART_ERROR("                regular control() interface not supported.");
  return NotApplicable;
}

/** Set speed for steady deceleration for stop distance
 *
 * @param pcmd contains desired heading and speed, assuming it is not
 *	       yet time to stop, updated on exit
 * @param distance to stop location along current path
 * @param threshold close enough distance
 * @param topspeed velocity limit
 * @return
 *	OK if in process of stopping;
 *	Finished if stop way-point reached.
 */
Controller::result_t Stop::control(pilot_command_t &pcmd,
				   float distance,
                                   float threshold,
				   float topspeed)
{
  result_t result = OK;

  // stop_latency compensates for latency in the braking system.
  float abs_speed = fabsf(estimate->twist.twist.linear.x);
  float latencydist = abs_speed * config_->stop_latency;
  float D = distance - latencydist;

  // According to the model, deceleration should be constant, but in
  // the real world latency will cause it to vary, so apply feedback
  // and recompute the model every cycle.
  if (abs_speed < Epsilon::speed)
    abs_speed = 0.0;

  float V = -abs_speed;
  float A = V*V/(2.0*D);

  // see if it is time to begin stopping
  if ((D <= config_->min_stop_distance || A >= config_->stop_deceleration)
      && !stopping)
    {
      stopping = true;
      initial_speed = fmaxf(topspeed,abs_speed);
      if (verbose)
	ART_MSG(2, "begin stopping while %.3fm distant", distance);
    }

  // Once stopping is initiated, keep doing it until reset(), no
  // matter how V changes.
  if (stopping)
    {

      if (D <= threshold)
	{
	  pcmd.velocity = 0.0;		// halt immediately.

	  // report Finished when stopped within stop polygon
	  if (abs_speed < Epsilon::speed)
	    result = Finished;
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
      else if (D <= config_->max_creep_distance)
	{
	  if (!creeping && verbose)
	    ART_MSG(2, "begin creeping while %.3fm distant", distance);
	  creeping = true;
	  // stopped too soon, keep creeping forward
	  pcmd.velocity = fminf(pcmd.velocity, config_->stop_creep_speed);
	}

      if (verbose >= 2)
	{
	  ART_MSG(5, "current, desired speed %.3f m/s, %.3f, decel %.3f m/s/s",
		  abs_speed, pcmd.velocity, A);
	  ART_MSG(5, "distance %.3f m, threshold %.3f, latency %.3f",
		  distance, threshold, latencydist);
	}
    }

  trace("stop controller", pcmd, result);
  return result;
};

// reset controller
void Stop::reset(void)
{
  stopping = false;
  creeping = false;
  initial_speed = 0.0;
};
