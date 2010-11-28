/*
 *  Navigator stop line safety area controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "stop_area.h"

#include <art/DARPA_rules.h>

StopArea::StopArea(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  reset_me();
};

StopArea::~StopArea() {};

// Slow down if stop line safety area reached.
//
// exit:
//	course->stop_waypt set
// result:
//	OK if within stop line safety area;
//	NotApplicable, otherwise.
//
Controller::result_t StopArea::control(pilot_command_t &pcmd)
{
  using art_msgs::ArtVehicle;
  float wayptdist = (course->stop_waypt_distance(true)
		     - ArtVehicle::front_bumper_px);

  if (wayptdist >= DARPA_rules::stop_line_safety_area)
    return NotApplicable;

  // reduce speed within stop line safety area
  nav->reduce_speed_with_min(pcmd, config_->stop_approach_speed);

  if (!in_safety_area)
    {
      in_safety_area = true;
      if (verbose)
	ART_MSG(2, "entering stop line safety area, distance %.3fm",
		wayptdist);
    }

  trace("stop_area controller", pcmd, OK);
  return OK;				// in stop line safety area
};

// reset controller and all subordinates
void StopArea::reset(void)
{
  trace_reset("StopArea");
  reset_me();
};

// reset controller state
void StopArea::reset_me(void)
{
  in_safety_area = false;
};
