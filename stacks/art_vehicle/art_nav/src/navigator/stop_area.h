/* -*- mode: C++ -*-
 *
 *  Navigator stop line safety area controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */


#ifndef __STOP_AREA_HH__
#define __STOP_AREA_HH__

class StopArea: public Controller
{
public:

  StopArea(Navigator *navptr, int _verbose);
  ~StopArea();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:

  // controller state
  bool in_safety_area;

  void reset_me(void);
};

#endif // __STOP_AREA_HH__
