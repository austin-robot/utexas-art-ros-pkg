/* -*- mode: C++ -*- 
 *
 *  Description:  Driver timer class
 *
 *  Copyright (C) 2009 Austin Robot Technology                    
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _D_TIMER_HH_
#define _D_TIMER_HH_

#include <sys/time.h>
#include <time.h>
#include <art_msgs/ArtHertz.h>

/** @brief driver timer class. */
class DriverTimer
{
 public:

  /** @brief Constructor */
  DriverTimer()
    {
      this->Cancel();
    };

  virtual ~DriverTimer() {};

  /** @brief Cancel timer. */
  virtual void Cancel(void)
  {
    timer_running = false;
  }

  /** @brief Return true if timer has expired.
   *
   *  Called once per cycle while timer is running.  Skipped cycles do
   *  not contribute to timer expiration.  That allows timers to pause
   *  while the vehicle is pausing.  It should not immediately begin
   *  passing after pausing behind a stopped vehicle, for example.
   */
  virtual bool Check(void)
  {
    if (!timer_running)
      return false;			// timer not set

    // decrement time remaining by the duration of one cycle
    time_remaining -= (1.0 / art_msgs::ArtHertz::STEERING);
    return (time_remaining <= 0.0);
  }

  /** @brief Restart timer.
   *
   *  Conditionally start timer unless running and not expired.
   */
  virtual void Restart(double duration)
  {
    if (timer_running && time_remaining > 0.0)
      return;

    Start(duration);
  }

  /** @brief Start timer. */
  virtual void Start(double duration)
  {
    timer_running = true;
    time_remaining = duration;
  }

 protected:

  double time_remaining;		//< time remaining until done
  bool timer_running;			//< true when timer running
};

#endif // _D_TIMER_HH_
