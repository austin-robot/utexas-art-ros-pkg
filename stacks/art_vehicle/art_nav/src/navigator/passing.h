//
// Navigator passing controller
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

#ifndef __PASSING_HH__
#define __PASSING_HH__

class Avoid;
class Halt;
class LaneHeading;
class FollowSafely;
class SlowForCurves;

class Passing: public Controller
{
public:

  Passing(Navigator *navptr, int _verbose);
  ~Passing();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:
  bool in_passing_lane;

  Avoid		*avoid;
  Halt		*halt;
  FollowSafely	*follow_safely;
  LaneHeading	*lane_heading;
  SlowForCurves *slow_for_curves;

  // .cfg variables
  float passing_distance;
  float close_stopping_distance;
  float passing_distance_ahead;
  float passing_distance_behind;
  float passing_speed;

  bool done_passing(void);

  // reset this controller only
  void reset_me(void);
};

#endif // __PASSING_HH__
