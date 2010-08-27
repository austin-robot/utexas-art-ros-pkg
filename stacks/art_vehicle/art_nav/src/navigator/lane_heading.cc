/*
 *  Navigator lane heading controller
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
#include "lane_heading.h"


LaneHeading::LaneHeading(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
}

LaneHeading::~LaneHeading()
{
}

void LaneHeading::configure()
{
}

// set desired lane heading for this cycle
Controller::result_t LaneHeading::control(pilot_command_t &pcmd)
{

  // set heading to desired course
  course->desired_heading(pcmd);

  trace("lane_heading controller", pcmd);

  return OK;				// always successful
}

// reset all subordinate controllers
void LaneHeading::reset(void)
{
  trace_reset("LaneHeading");
}
