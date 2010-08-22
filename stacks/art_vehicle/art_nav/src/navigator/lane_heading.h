//
// Navigator lane heading controller
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

#ifndef __LANE_HEADING_HH__
#define __LANE_HEADING_HH__


class LaneHeading: public Controller
{
public:

  LaneHeading(Navigator *navptr, int _verbose);
  ~LaneHeading();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd);
  void reset(void);

};

#endif // __LANE_HEADING_HH__
