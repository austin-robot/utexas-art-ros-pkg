/*
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     Experimental ART pilot acceleration controller.

     This controller does brake and throttle PID control adjust speed
     to match the target goal velocity. It does not monitor
     acceleration directly, but gradually adjusts the commanded speed
     depending on the requested acceleration.

     @author Jack O'Quin

 */

#include <ros/ros.h>
#include <art/pid2.h>
#include <art_msgs/ArtHertz.h>
#include <art_msgs/Epsilon.h>

#include "accel_plan.h"

using art_msgs::Epsilon;

namespace pilot
{

AccelPlan::AccelPlan(art_pilot::PilotConfig &config):
  AccelBase(config),
  braking_(true),                       // begin with brake on
  brake_pid_(new Pid("brake", config.brake_kp, config.brake_ki,
                     config.brake_kd, 1.0, 0.0, 5000.0)),
  throttle_pid_(new Pid("throttle", config.throttle_kp, config.throttle_ki,
                        config.throttle_kd, 0.4, 0.0, 5000.0))
{
  reset();
};

AccelPlan::~AccelPlan() {};

void AccelPlan::adjust(art_msgs::PilotState &pstate,
                       ServoPtr brake, ServoPtr throttle)
{
  // check whether target has changed since previous cycle
  float goal_speed = fabs(pstate.target.speed); 
  float goal_accel = fabs(pstate.target.acceleration);
  if (goal_speed != speed_ || goal_accel !=accel_) // target changed?
    {
      // save new targets for later comparisons
      speed_ = goal_speed;
      accel_ = goal_accel;

      // make a new acceleration plan
      pstate.plan.acceleration = accel_;
      if (accel_ == 0.0)                // no acceleration limit?
        {
          // plan directly to target speed
          pstate.plan.speed = speed_;
        }
      else
        {
          // make plan starting with current speed
          pstate.plan.speed = fabs(pstate.current.speed);
        }
    }

  // compute time since previous cycle
  float dt;                             // delta T
  if (prev_cycle_ == ros::Time())       // first cycle since reset?
    {
      // assume nominal cycle time
      dt = 1.0 / art_msgs::ArtHertz::PILOT;
    }
  else
    {
      // use actual time since previous cycle
      dt = (pstate.header.stamp - prev_cycle_).toSec();
    }
  
  // update plan in pilot state message
  if (accel_ != 0.0)                    // have acceleration limit?
    {
      // gradually change planned speed
      float error = speed_ - pstate.plan.speed;
      float dv = accel_ * dt;           // desired delta V for this cycle

      // limit error term to absolute value of requested acceleration
      if (fabs(error) <= dv)
        {
          pstate.plan.speed = speed_;
        }
      else
        {
          pstate.plan.speed += dv * signum(error);
        }
    }

  // request brake or throttle update to achieve planned velocity
  adjustVelocity(pstate, brake, throttle);

  // remember time of this cycle
  prev_cycle_ = pstate.header.stamp;
}

void AccelPlan::adjustVelocity(art_msgs::PilotState &pstate,
                               ServoPtr brake, ServoPtr throttle)
{
  float brake_request;
  float throttle_request;
  float abs_speed = fabs(pstate.current.speed);
  float error = fabs(pstate.plan.speed) - abs_speed;

  if (braking_)
    {
      // controlling with brake:
      brake_request = brake_pid_->Update(error, abs_speed);
      throttle_request = 0.0;
      
      // If requesting brake off, switch to throttle control.

      // Must check reported brake position, too.  Otherwise there
      // will be considerable overlap, applying throttle while the
      // brake is still on.  That can cause mechanical damage to the
      // transmission.
      if ((brake->value() < Epsilon::brake_position)
          && (brake_request < Epsilon::brake_position))
        {
          brake_request = 0.0;          // brake off
          braking_ = false;             // using throttle now
          throttle_pid_->Clear();       // reset PID controller
        }
    }
  else
    {
      // controlling with throttle:
      throttle_request = throttle_pid_->Update(error, abs_speed);
      brake_request = 0.0;

      // If requesting throttle off, switch to brake control.

      // Since throttle responds much faster than brake, it will reach
      // idle before the brake really starts engaging.  So, not
      // checking throttle->value() here is an option, which reduces
      // latency when slowing down.
      if (throttle_request < Epsilon::throttle_position)
        {
          throttle_request = 0.0;       // throttle off
          braking_ = true;              // using brake now
          brake_pid_->Clear();          // reset PID controller
        }
    }

  brake_request = clamp(0.0, brake_request, 1.0);
  if (fabsf(brake_request - brake->value()) > Epsilon::brake_position)
    {
      brake->publish(brake_request, pstate.header.stamp);
    }

  throttle_request = clamp(0.0, throttle_request, 1.0);
  if (fabsf(throttle_request - throttle->value()) > Epsilon::throttle_position)
    {
      throttle->publish(throttle_request, pstate.header.stamp);
    }
}

/** allocate appropriate speed control subclass for this configuration */
void AccelPlan::reconfigure(art_pilot::PilotConfig &newconfig)
{
  brake_pid_->Configure(newconfig.brake_kp,
                        newconfig.brake_ki,
                        newconfig.brake_kd);
  throttle_pid_->Configure(newconfig.throttle_kp,
                           newconfig.throttle_ki,
                           newconfig.throttle_kd);
}

/** reset controller */
void AccelPlan::reset(void)
{
  // clear any existing acceleration plan
  prev_cycle_ == ros::Time();
  accel_ = 0.0;
  speed_ = 0.0;

  // clear any Ki build-up in PID controllers
  brake_pid_->Clear();
  throttle_pid_->Clear();
}

};
