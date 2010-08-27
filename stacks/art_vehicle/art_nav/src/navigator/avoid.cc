/*
 *  Navigator avoid obstacles in path controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include "navigator_internal.h"
#include "Controller.h"
#include "avoid.h"

#include "lane_edge.h"
#include "safety.h"

Avoid::Avoid(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  lane_edge = new LaneEdge(navptr, _verbose);
  safety = new Safety(navptr, _verbose);
  reset_me();
};

Avoid::~Avoid()
{
  delete lane_edge;
  delete safety;
};

void Avoid::configure()
{
  // left and right lane offsets for lane_edge controller
  left_offset_ratio = cf->ReadFloat(section, "left_offset_ratio", 0.8);
  ART_MSG(2, "\toffset ratio to left of lane is %.3f", left_offset_ratio);
  right_offset_ratio = cf->ReadFloat(section, "right_offset_ratio", -0.8);
  ART_MSG(2, "\toffset ratio to right of lane is %.3f", right_offset_ratio);

  lane_edge->configure(cf, section);
  safety->configure(cf, section);
}

// avoid obstacles in travel lane
//
// entry:
//	pcmd is planned output from primary lane following controller
//	incmd is a copy of the original primary controller input
// exit:
//	pcmd updated based on safety considerations
// result:
//	from safety controller
//
Controller::result_t Avoid::control(pilot_command_t &pcmd,
				     pilot_command_t incmd)
{
  // make copies of primary controller output
  pilot_command_t right_cmd = pcmd;
  pilot_command_t left_cmd = pcmd;
  result_t result = safety->control(pcmd);
  if (result != OK)			// safety modified pcmd?
    {
      // Try some avoidance maneuvers.  Each reruns the safety
      // controller on its output before returning.
      result_t edge_right = lane_edge->control(right_cmd, right_offset_ratio);
      result_t edge_left = lane_edge->control(left_cmd, left_offset_ratio);

      if (std::min(edge_right, edge_left) < result)
	{
	  if (edge_left < edge_right)	// left side is clearer?
	    {
	      if (verbose >= 2)
		ART_MSG(5, "avoiding to left (%s < %s)",
			result_name(edge_left), result_name(edge_right));
	      pcmd = left_cmd;
	      result = edge_left;
	    }
	  else				// right side at least as good?
	    {
	      if (verbose >= 2)
		ART_MSG(5, "avoiding to right (%s >= %s)",
			result_name(edge_left), result_name(edge_right));
	      pcmd = right_cmd;
	      result = edge_right;
	    }
	}
    }

  trace("avoid controller", pcmd, result);
  return result;
};

// reset all subordinate controllers
void Avoid::reset(void)
{
  trace_reset("Avoid");
  reset_me();
  lane_edge->reset();
  safety->reset();
}

// reset this controller only
void Avoid::reset_me(void)
{
}
