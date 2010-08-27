/* -*- mode: C++ -*-
 *
 *  Navigator edge right controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __LANE_EDGE_HH__
#define __LANE_EDGE_HH__

class Safety;

class LaneEdge: public Controller
{
public:

  LaneEdge(Navigator *navptr, int _verbose);
  ~LaneEdge();
  void configure();
  result_t control(pilot_command_t &pcmd, float offset_ratio);
  void reset(void);

private:

  Safety *safety;
};

#endif // __LANE_EDGE_HH__
