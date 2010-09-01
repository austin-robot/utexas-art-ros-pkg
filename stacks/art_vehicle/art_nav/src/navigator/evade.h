/* -*- mode: C++ -*-
 *
 *  Finite state machine interface
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

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
  void configure();
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
