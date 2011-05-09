/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART pilot acceleration controller simple example.

     @author Jack O'Quin

 */

#ifndef _ACCEL_EXAMPLE_H_
#define _ACCEL_EXAMPLE_H_

#include "accel.h"

class Pid;                              // class reference for pointers

namespace pilot
{

/** Simple example acceleration controller. */
class AccelExample: public AccelBase
{
 public:

  AccelExample(art_pilot::PilotConfig &config);
  virtual ~AccelExample();

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

  /** Reset accel_example controller. */
  virtual void reset(void);

private:

  // When true, brake is the controlling device, otherwise throttle.
  bool braking_;

  boost::shared_ptr<Pid> brake_pid_;    // Brake_Pilot control PID
  boost::shared_ptr<Pid> throttle_pid_; // Throttle control PID
};
  
}; // namespace pilot

#endif // _ACCEL_EXAMPLE_H_
