/* -*- mode: C++ -*-
 *
 *  Description:  common steering constants and calculations
 *
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _STEERING_H
#define _STEERING_H

#include <angles/angles.h>
#include <art_msgs/ArtVehicle.h>

/** @brief 
 *
 * Common steering constants and calculations.
 */

namespace Steering
{

  /** Constants: */
  const float steer_speed_min = 3.8;	/* minimum speed for calculation */
					/* (experimentally verified) */

  /** Determine steering angle (in degrees) for a given speed and yawrate.
   *  
   *   This computation is based on a simplified "bicycle model" of
   *   vehicle steering geometery.  We ignore the slightly different
   *   angles of the two front wheels, abstracting them into a single
   *   wheel at the midline of the vehicle (like a bicycle or tricycle).
   *  
   *   Consider a radius from the center of the vehicle's turning circle
   *   to the midpoint of the rear axle.  The vehicle's wheelbase is at
   *   a right angle to this radius, since the rear wheels do not pivot.
   *   The hypotenuse of this right triangle is the slightly longer
   *   distance from the midpoint of the front axle back to the center
   *   of the circle.  As long as the wheels do not slip, the acute
   *   angle between the radius and the hypotenuse will be equal to the
   *   angle of the front wheel relative to the midline of the vehicle.
   *  
   *   Thus, the tangent of the steering angle is the wheelbase (w)
   *   divided by the radius (r).  We estimate r from the desired
   *   yawspeed (y), assuming a constant velocity (v):
   *  
   *  	y = (2*pi radians) / (2*pi*r/v seconds)
   *      y = v/r radians/second
   *      r = v/y
   */
  static inline float steering_angle(float v, float y)
  {
    float steer_radians = atan2f(art_msgs::ArtVehicle::wheelbase * y,v);
    float steer_degrees = angles::to_degrees(steer_radians);

    steer_degrees = fminf(steer_degrees,
                          art_msgs::ArtVehicle::max_steer_degrees);
    steer_degrees = fmaxf(steer_degrees,
                          -art_msgs::ArtVehicle::max_steer_degrees);

    return steer_degrees;
  }

  /** Determine yaw rate (in radians/second) for a given speed and
   *  steering angle (in degrees).  Inverse of steering_angle().
   */
  static inline double angle_to_yaw(double v, float angle)
  {
    return v * (tanf(angles::from_degrees(angle))
                / art_msgs::ArtVehicle::wheelbase);
  }

  const double maximum_yaw =
    angle_to_yaw(steer_speed_min, art_msgs::ArtVehicle::max_steer_degrees);
}

#endif // _STEERING_H
