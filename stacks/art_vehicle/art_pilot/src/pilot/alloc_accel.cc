/*
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     Allocate an acceleration controller based on the current pilot
     configuration.

     @author Jack O'Quin
 */

#include "accel.h"
#include "accel_example.h"
#include "accel_plan.h"
#include "accel_speed.h"

namespace pilot
{

// also define an empty abstract base class destructor for the linker
AccelBase::~AccelBase() {}

/** allocate an acceleration controller instance
 *
 *  @param config current Pilot configuration parameters
 *  @return boost shared pointer to a newly allocated controller
 */
boost::shared_ptr<AccelBase>
  allocAccel(art_pilot::PilotConfig &config)
{
  boost::shared_ptr<AccelBase> controller;

  switch(config.acceleration_controller)
    {
    default:
      {
        ROS_ERROR_STREAM("Unknown acceleration controller: "
                         << config.acceleration_controller
                         << " (using AccelExample)");
        config.acceleration_controller = art_pilot::Pilot_Accel_Plan;
        // no break: fall into next case...
      }
    case art_pilot::Pilot_Accel_Example:
      {
        ROS_INFO("using example acceleration controller");
        controller.reset(new AccelExample(config));
        break;
      }
    case art_pilot::Pilot_Accel_Plan:
      {
        ROS_INFO("using planned acceleration controller");
        controller.reset(new AccelPlan(config));
        break;
      }
    case art_pilot::Pilot_Speed_Learned:
    case art_pilot::Pilot_Speed_Matrix:
    case art_pilot::Pilot_Speed_PID:
      {
        // An older, speed-based controller. It will figure out which.
        controller.reset(new AccelSpeed(config));
        break;
      }
    }

  return controller;
}

}; // namespace pilot
