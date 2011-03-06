/* -*- mode: C++ -*-
 *
 *  Navigator E-stop finite state machine events
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __NAV_ESTOP_EVENT_HH__
#define __NAV_ESTOP_EVENT_HH__

#include <art_nav/FSMevent.h>

class NavEstopEvent: FSMevent
{
public:

  // navigator E-stop control events
  typedef enum
    {
      Abort,				// E-stop disable
      Pause,				// E-stop pause
      Quit,				// run finished normally
      Run,				// E-stop run enabled
      Suspend,				// suspend autonomous operation
      None,				// no E-stop event this cycle
      N_events
    } event_t;

  // return event name as a C string
  const char *Name(void)
  {
    static const char *event_name[N_events] =
      {
	"Abort",
	"Pause",
	"Quit",
	"Run",
	"Suspend",
	"None",
      };
    return event_name[this->event];
  }

  NavEstopEvent()
  {
    this->event = Pause;
  }

  NavEstopEvent(event_t ievent)
  {
    this->event = ievent;
  }

  ~NavEstopEvent() {};

  event_t Value(void)
  {
    return this->event;
  }

  void operator=(const NavEstopEvent::event_t &newevent)
  {
    this->event = newevent;
  }

  bool operator==(const event_t &compare)
  {
    return this->event == compare;
  }

  bool operator!=(const event_t &compare)
  {
    return this->event != compare;
  }

private:
  event_t event;
};

#endif // __NAV_ESTOP_EVENT_HH__
