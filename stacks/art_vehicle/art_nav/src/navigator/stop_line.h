/* -*- mode: C++ -*-
 *
 *  Navigator stop line controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
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
  void configure();
  result_t control(pilot_command_t &pcmd, float topspeed=3.0);
  void reset(void);

private:
  // .cfg variables
  float min_stop_distance;		// minimum distance to begin stopping
  float stop_creep_speed;		// speed while creeping forward
  float stop_deceleration;		// desired deceleration
  float stop_distance;			// desired stop distance
  float stop_latency;			// stop control latency in seconds

  // controller state
  bool stopping;			// stopping initiated
  bool creeping;                        // creeping up to line
  float initial_speed;			// initial speed while stopping
  float max_creep_distance;             // applicable distance for creep

  void reset_me(void);
};

#endif // __STOP_LINE_HH__
