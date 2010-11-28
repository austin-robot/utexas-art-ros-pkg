/* -*- mode: C++ -*-
 *
 *  Navigator "slow down for curves" controller
 *  Copyright (C) 2007, 2010, Mickey Ristroph
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __SLOW_FOR_CURVES_HH__
#define __SLOW_FOR_CURVES_HH__

class SlowForCurves: public Controller
{
public:

  SlowForCurves(Navigator *navptr, int _verbose):
    Controller(navptr, _verbose), current_limiting_id(0) {}
  ~SlowForCurves() {}
  result_t control(pilot_command_t &pcmd);
  void reset(void);


private:
  
  // state
  int current_limiting_id;
  
  float max_safe_speed(const std::vector<poly>& polygons,
		       const int& start_index,
		       const int& stop_index,
		       const float& max);
};

#endif // __SLOW_FOR_CURVES_HH__
