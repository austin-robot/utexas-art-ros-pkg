/* -*- mode: C++ -*-
 *
 *  Navigator passing controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __PASSING_HH__
#define __PASSING_HH__

//class Avoid;
class Halt;
class FollowSafely;
class SlowForCurves;

class Passing: public Controller
{
public:

  Passing(Navigator *navptr, int _verbose);
  ~Passing();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:
  bool in_passing_lane;

  //Avoid       *avoid;
  Halt		*halt;
  FollowSafely	*follow_safely;
  SlowForCurves *slow_for_curves;

  bool done_passing(void);

  // reset this controller only
  void reset_me(void);
};

#endif // __PASSING_HH__
