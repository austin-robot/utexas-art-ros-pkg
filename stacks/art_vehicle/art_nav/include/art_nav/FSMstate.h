/* -*- mode: C++ -*-
 *
 *  Base class for finite state machine states
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 *
 *  Author: Jack O'Quin
 */

#ifndef __FSM_STATE_H__
#define __FSM_STATE_H__

class FSMstate
{
public:

  // States in this base class are just for example.  State values are
  // overloaded for each real FSM definition.  Since state_t in the
  // subclasses differs from this, methods with state_t parameters
  // cannot be inherited from FSMstate.
  typedef enum
    {
      Start,				// starting state
      Final,				// final state
      N_states				// total number of states
    } state_t;

  // return name of each state as a C string
  const char *Name(void)
  {
    static const char *state_name[N_states] =
      {
	"Start",
	"Final",
      };
    return state_name[state];
  }

  FSMstate()
  {
    this->state = Start;
  }

  ~FSMstate();

  void operator=(const FSMstate &newval)
  {
    this->state = newval.state;
  }

  bool operator==(const FSMstate &compare)
  {
    return (int) this->state == (int) compare.state;
  }

  bool operator!=(const FSMstate &compare)
  {
    return (int) this->state != (int) compare.state;
  }

private:

  state_t state;			// current state
};

#endif // __FSM_STATE_H__
