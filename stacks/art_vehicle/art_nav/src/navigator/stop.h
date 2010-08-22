//
// Navigator stop line controller
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

#ifndef __STOP_LINE_HH__
#define __STOP_LINE_HH__

class Stop: public Controller
{
public:

  Stop(Navigator *navptr, int _verbose);
  ~Stop();
  void configure(ConfigFile* cf, int section);

  // regular control method -- not supported
  result_t control(pilot_command_t &pcmd);

  // overloaded control method
  result_t control(pilot_command_t &pcmd, float distance, float threshold,
		   float topspeed=3.0);
  void reset(void);

private:
  // .cfg variables
  float min_stop_distance;		// minimum distance to begin stopping
  float stop_creep_speed;		// speed while creeping forward
  float stop_deceleration;		// desired deceleration
  float stop_latency;			// stop control latency in seconds

  // controller state
  bool stopping;			// stopping initiated
  bool creeping;                        // creeping up to line
  float initial_speed;			// initial speed while stopping
  float max_creep_distance;             // applicable distance for creep
};

#endif // __STOP_LINE_HH__
