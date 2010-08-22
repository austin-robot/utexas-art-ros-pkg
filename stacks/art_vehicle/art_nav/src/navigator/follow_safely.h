//
// Navigator safe following distance controller
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

#ifndef __FOLLOW_SAFELY_HH__
#define __FOLLOW_SAFELY_HH__

class FollowSafely: public Controller
{
public:

  FollowSafely(Navigator *navptr, int _verbose);
  ~FollowSafely();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd);

private:
  // .cfg variables
  float close_stopping_distance;
  float desired_following_time;
  float max_following_time;
  float min_following_time;

  // adjust speed to maintain a safe following time
  void adjust_speed(pilot_command_t &pcmd, float obs_dist);
};

#endif // __FOLLOW_SAFELY_HH__
