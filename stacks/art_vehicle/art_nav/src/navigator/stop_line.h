/* -*- mode: C++ -*-
 *
 *  Navigator stop line controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */


#ifndef __STOP_LINE_HH__
#define __STOP_LINE_HH__

class StopLine: public Controller
{
public:

  StopLine(Navigator *navptr, int _verbose);
  ~StopLine();
  result_t control(pilot_command_t &pcmd, float topspeed=3.0);
  void reset(void);

private:

  // controller state
  bool stopping;                        // stopping initiated
  bool creeping;                        // creeping up to line
  double initial_speed;                 // initial speed while stopping
  double max_creep_distance;            // applicable distance for creep

  void reset_me(void);
};

#endif // __STOP_LINE_HH__
