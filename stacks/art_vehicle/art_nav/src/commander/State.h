/* -*- mode: C++ -*-
 *
 *  Commander finite state machine interface
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

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
