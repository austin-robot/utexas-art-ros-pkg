/*
 *  Navigator lane edge controller
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
#include "lane_edge.h"
#include "safety.h"

LaneEdge::LaneEdge(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  safety = new Safety(navptr, _verbose);
}

LaneEdge::~LaneEdge()
{
  delete safety;
}

void LaneEdge::configure()
{
  safety->configure(cf, section);
}

// steer towards the edge of the lane
//
// entry:
//	offset_ratio = lane offset ratio: where 1.0 is the left edge
//		and -1.0 is the right edge.
// exit:
//	pcmd adjusted to head for desired offset in the lane with
//	speed adjusted by the safety controller
// returns:
//	safety controller result if this path taken
//	
Controller::result_t LaneEdge::control(pilot_command_t &pcmd,
				       float offset_ratio)
{
  result_t result = OK;

  // set heading with desired offset
  course->desired_heading(pcmd, offset_ratio);

  // always run safety check on the output
  result = safety->control(pcmd);

  trace("lane_edge controller", pcmd, result);
  return result;
}

// reset all subordinate controllers
void LaneEdge::reset(void)
{
  trace_reset("LaneEdge");
  safety->reset();
}
