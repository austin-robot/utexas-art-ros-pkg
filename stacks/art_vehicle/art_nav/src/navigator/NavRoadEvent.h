/* -*- mode: C++ -*-
 *
 *  Navigator road finite state machine events
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __NAV_ROAD_EVENT_HH__
#define __NAV_ROAD_EVENT_HH__

#include <art_nav/FSMevent.h>

class NavRoadEvent: FSMevent
{
public:

  // navigator E-stop control events
  typedef enum
    {
      Block,				// road blocked
      ChangeLane,			// change lanes here
      Collision,			// collision imminent
      FollowLane,			// follow travel lane
      Merge,				// intersection merge
      None,				// no new event
      Pass,				// pass obstacle in lane
      Perimeter,			// passed zone perimeter
      StopLine,				// stopped at line
      Uturn,				// do U-turn
      WaitPass,				// wait to pass
      N_events
    } event_t;

  // return event name as a C string
  const char *Name(void)
  {
    static const char *event_name[N_events] =
      {
	"Block",
	"ChangeLane",
	"Collision",
	"FollowLane",
	"Merge",
	"None",
	"Pass",
	"Perimeter",
	"StopLine",
	"Uturn",
	"WaitPass",
      };
    return event_name[this->event];
  }

  NavRoadEvent()
  {
    this->event = None;
  }

  NavRoadEvent(event_t ievent)
  {
    this->event = ievent;
  }

  ~NavRoadEvent() {};

  event_t Value(void)
  {
    return this->event;
  }

  void operator=(const NavRoadEvent &newevent)
  {
    this->event = newevent.event;
  }

  void operator=(const NavRoadEvent::event_t &newevent)
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

#endif // __NAV_ROAD_EVENT_HH__
