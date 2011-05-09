/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     Experimental ART pilot acceleration controller.

     @author Jack O'Quin

 */

#ifndef _ACCEL_PLAN_H_
#define _ACCEL_PLAN_H_

#include "accel.h"

class Pid;                              // class reference for pointers

namespace pilot
{

/** Simple example acceleration controller. */
class AccelPlan: public AccelBase
{
 public:

  AccelPlan(art_pilot::PilotConfig &config);
  virtual ~AccelPlan();

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

  /** Reset controller. */
  virtual void reset(void);

private:

  /** Adjust velocity to match target.
   *
   *  @pre pstate.plan reflects planned velocity change
   *
   *  @param pstate current pilot state
   *  @param brake shared pointer to brake servo device interface
   *  @param throttle shared pointer to throttle servo device interface
   */
  void adjustVelocity(art_msgs::PilotState &pstate,
                      ServoPtr brake, ServoPtr throttle);

  float speed_;                   // absolute value of target velocity
  float accel_;                   // absolute value of acceleration
  ros::Time prev_cycle_;          // previous cycle time for this plan

  // When true, brake is the controlling device, otherwise throttle.
  bool braking_;

  boost::shared_ptr<Pid> brake_pid_;    // Brake_Pilot control PID
  boost::shared_ptr<Pid> throttle_pid_; // Throttle control PID
};
  
}; // namespace pilot

#endif // _ACCEL_PLAN_H_
