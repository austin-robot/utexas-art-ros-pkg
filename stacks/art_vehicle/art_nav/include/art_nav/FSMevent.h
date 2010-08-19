//
// Base class for finite state machine events
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

#ifndef __FSM_EVENT_H__
#define __FSM_EVENT_H__

class FSMevent
{
public:
  // Events in this base class are just for example.  They will be
  // overloaded by each real FSM definition.  Since event_t in the
  // subclasses differs from this, methods with event_t parameters
  // cannot be inherited from FSMevent.
  typedef enum
    {
      Done,				// stop the FSM
      None,				// Null event
      N_events				// total number of events
    } event_t;

  // return name of each event as a C string
  const char *Name()
  {
    static const char *event_name[N_events] =
      {
	"Done",
	"None",
      };
    return event_name[event];
  }

  FSMevent()
  {
    this->event = None;
  }

  void operator=(const FSMevent &newval)
  {
    this->event = newval.event;
  }

  bool operator==(const FSMevent compare)
  {
    return this->event == compare.event;
  }

  bool operator!=(const FSMevent compare)
  {
    return this->event != compare.event;
  }

private:
  event_t event;
};

#endif // __FSM_EVENT_H__
