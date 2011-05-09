/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART pilot acceleration controller using older speed controllers.

     @author Jack O'Quin

 */

#ifndef _ACCEL_SPEED_H_
#define _ACCEL_SPEED_H_

#include "accel.h"
#include "speed.h"

namespace pilot
{

/** Pilot acceleration controller for older speed control classes */
class AccelSpeed: public AccelBase
{
 public:

  AccelSpeed(art_pilot::PilotConfig &config);
  virtual ~AccelSpeed();

  typedef boost::shared_ptr<device_interface::ServoDeviceBase> ServoPtr;

  /** Adjust acceleration to match target.
   *
   *  @param pstate current pilot state
   *  @param brake shared pointer to brake servo device interface
   *  @param throttle shared pointer to throttle servo device interface
   */
  virtual void adjust(art_msgs::PilotState &pstate,
                      ServoPtr brake, ServoPtr throttle);

  /** Reconfigure controller parameters. */
  virtual void reconfigure(art_pilot::PilotConfig &newconfig);

  /** Reset accel_speed controller. */
  virtual void reset(void);

private:

  boost::shared_ptr<SpeedControl> speed_; // speed control
};
  
}; // namespace pilot

#endif // _ACCEL_SPEED_H_
