//
// Navigator edge right controller
//
//  Copyright (C) 2007 Austin Robot Technology
//  All Rights Reserved. Licensed Software.
//
//  This is unpublished proprietary source code of Austin Robot
//  Technology, Inc.  The copyright notice above does not evidence any
//  actual or intended publication of such source code.
//
//  PROPRIETARY INFORMATION, PROPERTY OF AUSTIN ROBOT TECHNOLOGY
//
//  $Id$
//
//  Author: Jack O'Quin
//

#ifndef __LANE_EDGE_HH__
#define __LANE_EDGE_HH__

class Safety;

class LaneEdge: public Controller
{
public:

  LaneEdge(Navigator *navptr, int _verbose);
  ~LaneEdge();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd, float offset_ratio);
  void reset(void);

private:

  Safety *safety;
};

#endif // __LANE_EDGE_HH__
