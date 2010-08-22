//
// Navigator evade obstacle controller
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

#ifndef __EVADE_HH__
#define __EVADE_HH__

class Halt;
class LaneEdge;
class Safety;
class NavTimer;

class Evade: public Controller
{
public:

  Evade(Navigator *navptr, int _verbose);
  ~Evade();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:

  // .cfg variables
  float evade_delay;
  float evade_offset_ratio;
  float evasion_speed;

  // simple state machine
  typedef enum
    {
      Init,
      Leave,
      Wait,
      Return,
    } state_t;

  state_t state;			// current FSM state

  NavTimer *evade_timer;

  // subordinate controllers
  Halt *halt;
  LaneEdge *lane_edge;
  Safety *safety;

  Controller::result_t leave_lane_right(pilot_command_t &pcmd);
  void reset_me(void);
  void set_state(state_t newstate);
};

#endif // __EVADE_HH__
