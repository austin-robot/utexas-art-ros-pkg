/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART pilot acceleration controller abstract base class.

     @author Jack O'Quin

 */

#ifndef _ACCEL_H_
#define _ACCEL_H_

#include <ros/ros.h>
#include <art_msgs/PilotState.h>
#include <art_pilot/PilotConfig.h>
#include "device_interface.h"

namespace pilot
{

/** Pilot acceleration controller virtual base class */
class AccelBase
{
 public:

  /** Constructor.
   *
   *  @param[in,out] config pilot configuration parameters
   *                 (may be modified if parameter values invalid)
   */
  AccelBase(art_pilot::PilotConfig &config) {};

  /** Destructor (required for virtual base classes). */
  virtual ~AccelBase();

  /** Shared pointer to servo device instance. */
  typedef boost::shared_ptr<device_interface::ServoDeviceBase> ServoPtr;

  /** Adjust acceleration to match target.
   *
   *  @param pstate current pilot state
   *  @param brake shared pointer to brake servo device interface
   *  @param throttle shared pointer to throttle servo device interface
   */
  virtual void adjust(art_msgs::PilotState &pstate,
                      ServoPtr brake, ServoPtr throttle) = 0;

  /** Reconfigure controller parameters.
   *
   *  @param[in,out] newconfig latest pilot configuration parameters
   *                 (may be modified if parameter values invalid)
   */
  virtual void reconfigure(art_pilot::PilotConfig &newconfig) = 0;

  /** Reset acceleration controller. */
  virtual void reset(void) = 0;
};

/** Shared point to AccelBase instance. */
typedef boost::shared_ptr<AccelBase> AccelBasePtr;

/** Allocate a new acceleration controller.
 *
 *  @param[in,out] config latest pilot configuration parameters
 *                 (may be modified if parameter values invalid)
 *  @return boost shared pointer to acceleration controller object
 */
AccelBasePtr allocAccel(art_pilot::PilotConfig &config);

/** Clamp value to range.
 *
 *  @lower minimum value of range
 *  @value data to compare
 *  @upper maximum value of range
 *  @return @a value unless it is outside [@a lower, @a upper];
 *          or the nearest limit, otherwise.
 */
static inline float clamp(float lower, float value, float upper)
{
  return std::max(lower, std::min(value, upper));
}

/** Signum (sign) of a value.
 *
 *  @value data to compare
 *  @return 1.0 if @a value is positive, 0.0 if zero, -1.0 if negative.
 */
static inline float signum(float value)
{
  return (value > 0.0? 1.0:
          (value == 0.0? 0.0:
           -1.0));
}
  
}; // namespace pilot

#endif // _ACCEL_H_
