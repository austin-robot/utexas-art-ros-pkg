//
// Navigator run controller
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

#ifndef __RUN_HH__
#define __RUN_HH__

// forward declarations for subordinate classes
class Halt;
class Road;
class Safety;
class NavTimer;
class VoronoiZone;

class Run: public Controller
{
public:

  Run(Navigator *navptr, int _verbose);
  ~Run();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:

  // Go behavior state machine
  typedef enum
    {
      Continue,				// run road controller
      Escape,				// try to get unstuck
      Replan,				// wait for Commander to replan
    } state_t;

  state_t go_state;			// current Go behavior FSM state
  player_pose2d_t blockage_pose;
  float blockage_waypt_dist;
  int last_replan;
  NavTimer *escape_timer;

  // .cfg variables
  bool  escape;
  float escape_distance;
  double escape_timeout_secs;
  bool  extra_safety_check;
  float initialize_distance;
  float initialize_min_angle;
  float max_speed;

  // subordinate controllers
  Halt		*halt;
  Road		*road;
  Safety	*safety;
  VoronoiZone	*unstuck;

  // behavior-specific control methods
  Controller::result_t initialize(pilot_command_t &pcmd);
  Controller::result_t go(pilot_command_t &pcmd);

  void begin_escape(void);
  void set_go_state(state_t);
  ElementID starting_waypt(void);
};

#endif // __RUN_HH__
