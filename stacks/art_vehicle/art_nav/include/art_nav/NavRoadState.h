//
// Navigator road finite state machine states
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

#ifndef __NAV_ROAD_STATE_H__
#define __NAV_ROAD_STATE_H__

#include <art_nav/FSMstate.h>

class NavRoadState: FSMstate
{
public:

  // Navigator road states
  typedef enum
    {
      Block,
      Evade,
      Follow,
      Init,
      Pass,
      Uturn,
      WaitCross,
      WaitLane,
      WaitPass,
      WaitStop,
      Zone,
      N_states
    } state_t;

  // return state name as a C string
  const char *Name(void) const
  {
    static const char *state_name[N_states] =
      {
	"Block",
	"Evade",
	"Follow",
	"Init",
	"Pass",
	"Uturn",
	"WaitCross",
	"WaitLane",
	"WaitPass",
	"WaitStop",
	"Zone",
      };
    return state_name[this->state];
  }

  NavRoadState()
  {
    this->state = Init;
  }

  NavRoadState(state_t &istate)
  {
    this->state = istate;
  }

  ~NavRoadState();

  state_t Value(void) const
  {
    return this->state;
  }

  void operator=(const NavRoadState::state_t &newstate)
  {
    this->state = newstate;
  }

  bool operator==(const NavRoadState &compare) const
  {
    return this->state == compare.state;
  }

  bool operator==(const state_t &compare) const
  {
    return this->state == compare;
  }

  bool operator!=(const NavRoadState &compare) const
  {
    return this->state != compare.state;
  }

  bool operator!=(const state_t &compare) const
  {
    return this->state != compare;
  }

private:
  state_t state;
};

#endif // __NAV_ROAD_STATE_H__
