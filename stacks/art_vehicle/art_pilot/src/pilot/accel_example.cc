/*
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART pilot acceleration controller example.

     This simple example just does brake and throttle PID control
     adjust speed to match the target goal velocity. It does not
     handle acceleration directly.

     @author Jack O'Quin

 */

#include <ros/ros.h>
#include <art/pid2.h>
#include <art_msgs/Epsilon.h>

#include "accel_example.h"

using art_msgs::Epsilon;

namespace pilot
{

AccelExample::AccelExample(art_pilot::PilotConfig &config):
  AccelBase(config),
  braking_(true),                       // begin with brake on
  brake_pid_(new Pid("brake", config.brake_kp, config.brake_ki,
                     config.brake_kd, 1.0, 0.0, 5000.0)),
  throttle_pid_(new Pid("throttle", config.throttle_kp, config.throttle_ki,
                        config.throttle_kd, 0.4, 0.0, 5000.0))
{
  reset();
};

AccelExample::~AccelExample() {};

void AccelExample::adjust(art_msgs::PilotState &pstate,
                        ServoPtr brake, ServoPtr throttle)
{
  float brake_request;
  float throttle_request;
  float abs_speed = fabs(pstate.current.speed);
  float error = fabs(pstate.target.speed) - abs_speed;

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
void AccelExample::reconfigure(art_pilot::PilotConfig &newconfig)
{
  brake_pid_->Configure(newconfig.brake_kp,
                        newconfig.brake_ki,
                        newconfig.brake_kd);
  throttle_pid_->Configure(newconfig.throttle_kp,
                           newconfig.throttle_ki,
                           newconfig.throttle_kd);
}

/** reset corresponding speed controller */
void AccelExample::reset(void)
{
  brake_pid_->Clear();
  throttle_pid_->Clear();
}

};
