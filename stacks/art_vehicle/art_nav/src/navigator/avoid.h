/* -*- mode: C++ -*-
 *
 *  Navigator avoid obstacles in path controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __AVOID_HH__
#define __AVOID_HH__

class LaneEdge;
class Safety;

class Avoid: public Controller
{
public:

  Avoid(Navigator *navptr, int _verbose);
  ~Avoid();
  void configure();
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
