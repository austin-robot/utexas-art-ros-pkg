/*
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART pilot acceleration controller using older speed controllers.

     @author Jack O'Quin

 */

#include <ros/ros.h>
#include "accel_speed.h"

#include "speed.h"
#include "learned_controller.h"

namespace pilot
{

AccelSpeed::AccelSpeed(art_pilot::PilotConfig &config):
  AccelBase(config)
{
  switch(config.acceleration_controller)
    {
    case art_pilot::Pilot_Speed_Learned:
      {
        ROS_INFO("using RL learned controller for speed control");
        speed_.reset(new LearnedSpeedControl());
        break;
      }
    case art_pilot::Pilot_Speed_Matrix:
      {
        ROS_INFO("using acceleration matrix for speed control");
        speed_.reset(new SpeedControlMatrix());
        break;
      }
    case art_pilot::Pilot_Speed_PID:
      {
        ROS_INFO("using brake and throttle PID for speed control");
        speed_.reset(new SpeedControlPID());
        break;
      }
    }

  // pass configuration parameters to the relevant speed controller
  speed_->configure(config);
};

AccelSpeed::~AccelSpeed() {};

void AccelSpeed::adjust(art_msgs::PilotState &pstate,
                        ServoPtr brake, ServoPtr throttle)
{
  // initialize interface to old SpeedControl class
  //
  // abs_speed: absolute value of current velocity in m/s
  // error: difference between that and our immediate goal
  //
  float abs_speed = fabs(pstate.current.speed);
  float error = fabs(pstate.target.speed) - abs_speed;
  float brake_request = brake->last_request();
  float throttle_request = throttle->last_request();
  speed_->set_brake_position(brake->value());
  speed_->set_throttle_position(throttle->value());

  // Adjust brake and throttle settings.
  speed_->adjust(abs_speed, error,
                 &brake_request, &throttle_request);

  brake_request = clamp(0.0, brake_request, 1.0);
  if (fabsf(brake_request - brake->value()) > EPSILON_BRAKE)
    {
      brake->publish(brake_request, pstate.header.stamp);
    }

  throttle_request = clamp(0.0, throttle_request, 1.0);
  if (fabsf(throttle_request - throttle->value()) > EPSILON_THROTTLE)
    {
      throttle->publish(throttle_request, pstate.header.stamp);
    }
}

/** allocate appropriate speed control subclass for this configuration */
void AccelSpeed::reconfigure(art_pilot::PilotConfig &newconfig)
{
  // pass new configuration to underlying speed control class
  speed_->configure(newconfig);
}

/** reset corresponding speed controller */
void AccelSpeed::reset(void)
{
  speed_->reset();
}

};
