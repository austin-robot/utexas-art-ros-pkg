//
// Navigator avoid obstacles in path controller
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

#ifndef __AVOID_HH__
#define __AVOID_HH__

class LaneEdge;
class Safety;

class Avoid: public Controller
{
public:

  Avoid(Navigator *navptr, int _verbose);
  ~Avoid();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd, pilot_command_t incmd);
  void reset(void);

private:

  // .cfg variables
  float left_offset_ratio;
  float right_offset_ratio;

  LaneEdge *lane_edge;
  Safety *safety;

  void reset_me(void);
};

#endif // __AVOID_HH__
