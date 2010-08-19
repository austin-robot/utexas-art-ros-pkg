//
// Commander finite state machine interface
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

#ifndef __CMDR_STATE_H__
#define __CMDR_STATE_H__

class CmdrState
{
public:
  // Commander states
  typedef enum
    {
      Done,				// mission completed
      Init,				// initial state
      Road,				// travelling on road
      N_states				// total number of states
    } state_t;

  // return name of each state as a C string
  const char *Name(void)
  {
    static const char *state_name[N_states] =
      {
	"Done",
	"Init",
	"Road",
      };
    return state_name[state];
  }

  CmdrState()
  {
    this->state = Init;			// initial state
  }

  CmdrState(state_t val)
  {
    this->state = val;
  }

  state_t Value(void)
  {
    return this->state;
  }

  void operator=(state_t newval)
  {
    this->state = newval;
  }

  bool operator==(CmdrState newval)
  {
    return this->state == newval.state;
  }

  bool operator==(state_t newval)
  {
    return this->state == newval;
  }

  bool operator!=(CmdrState newval)
  {
    return this->state != newval.state;
  }

  bool operator!=(state_t newval)
  {
    return this->state != newval;
  }

private:
  state_t state;			// current state
};

#endif // __CMDR_STATE_H__
