/* -*- mode: C++ -*-
 *
 *  Navigator safe following distance controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __FOLLOW_SAFELY_HH__
#define __FOLLOW_SAFELY_HH__

class FollowSafely: public Controller
{
public:

  FollowSafely(Navigator *navptr, int _verbose);
  ~FollowSafely();
  result_t control(pilot_command_t &pcmd);

private:

  // adjust speed to maintain a safe following time
  void adjust_speed(pilot_command_t &pcmd, float obs_dist);
};

#endif // __FOLLOW_SAFELY_HH__
