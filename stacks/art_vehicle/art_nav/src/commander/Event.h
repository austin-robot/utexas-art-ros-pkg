/* -*- mode: C++ -*-
 *
 * Commander finite state machine interface events
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __CMDR_EVENT_H__
#define __CMDR_EVENT_H__

#include <art_nav/FSMevent.h>

class CmdrEvent: FSMevent
{
public:
  typedef enum
    {
      Blocked,				// road blocked
      Done,				// mission completed
      EnterLane,			// enter travel lane
      Fail,				// mission failure
      None,				// no significant event
      Replan,
      Wait,
      N_events				// total number of events
    } event_t;

  // return name of each event as a C string
  const char *Name()
  {
    static const char *event_name[] =
      {
	"Blocked",
	"Done",
	"EnterLane",
	"Fail",
	"None",
	"Replan",
	"Wait",
      };
    return event_name[event];
  }

  CmdrEvent()
  {
    this->event = None;
  }

  CmdrEvent(event_t val)
  {
    this->event = val;
  }

  event_t Value(void)
  {
    return this->event;
  }

  void operator=(event_t newval)
  {
    this->event = newval;
  }

  void operator=(CmdrEvent newval)
  {
    this->event = newval.event;
  }

private:
  event_t event;
};

#endif // __CMDR_EVENT_H__
