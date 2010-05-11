/*
 *  Description:  ART vehicle dimensions.
 *
 *  Copyright (C) 2009 Austin Robot Technology                    
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _VEHICLE_HH_
#define _VEHICLE_HH_

/**  @file
   
     @brief ART vehicle dimensions.
   
     This class encapsulates constants for the dimensions of the ART
     autonomous vehicle.  All dimensions are in meters.
   
     @author Jack O'Quin
 */

#include <math.h>
#include <angles/angles.h>

namespace ArtVehicle
{
  const std::string frame_id = "vehicle"; // ROS frame ID

  const float length = 4.8f;		//< overall length
  const float width = 2.12f;		//< overall width
  const float halflength = length/2;
  const float halfwidth = width/2;
  const float wheelbase = 2.33918;	//< wheelbase in meters

  // egocentric coordinates relative to vehicle origin at center of
  // rear axle
  const float front_bumper_px = 3.5f;	// (approximately)
  const float rear_bumper_px = front_bumper_px - length;
  const float front_left_wheel_px = wheelbase;
  const float front_left_wheel_py = halfwidth;
  const float front_right_wheel_px = wheelbase;
  const float front_right_wheel_py = -halfwidth;
  const float rear_left_wheel_px = 0.0f;
  const float rear_left_wheel_py = halfwidth;
  const float rear_right_wheel_px = 0.0f;
  const float rear_right_wheel_py = -halfwidth;

  // player geometry, egocentric pose of robot base (the px really
  // does need to be positive for some reason)
  const float geom_px = front_bumper_px - halflength;
  const float geom_py = 0.0;
  const float geom_pa = 0.0;

  const float velodyne_px = 0.393; // (approximately)
  const float velodyne_py=0.278;   // (approximately)
  const float velodyne_pz=2.2;   // 2.25 was first approximate
				 // measurement, but 2.2 as after a
				 // plane fit to the data (when
				 // getting roll and pitch)
  const float velodyne_yaw=-0.0343;// (approximately)
  const float velodyne_pitch=0.016353735091186868; // (calculated)
  const float velodyne_roll=0.0062133721370998124; // (calculated)

  const float front_SICK_px = 3.178;
  const float front_SICK_py= 0.0;		// (approximately)
  const float front_SICK_pz = 0.941;
  const float front_SICK_roll = 0.0;		// (approximately)
  const float front_SICK_pitch = 0.0;		// (approximately)
  const float front_SICK_yaw = 0.027;	        // (approximately)

  const float rear_SICK_px = -1.140;
  const float rear_SICK_py = 0.0;               // (approximately)
  const float rear_SICK_pz = 0.943;
  const float rear_SICK_roll = 0.0;		// (approximately)
  const float rear_SICK_pitch = 0.0;		// (approximately)
  const float rear_SICK_yaw = M_PI;	        // (approximately)

  const float camera_front_right_px=velodyne_px+0.127; // (approx)
  const float camera_front_right_py=velodyne_py-0.089; // (approx)
  const float camera_front_right_pz=velodyne_pz-0.216;   // (approximately)
  const float camera_front_right_yaw=-0.4974;// (-28.5 deg approx)
  const float camera_front_right_pitch=0.0; // (assumed)
  const float camera_front_right_roll=0.0; // (assumed)

  // Compute vehicle turning radius.  This is the distance from the
  // center of curvature to the vehicle origin in the middle of the
  // rear axle.  The <art/steering.h> comments describe the steering
  // geometry model.  Since max_steer_degrees is considerably less
  // than 90 degrees, there is no problem taking its tangent.
  const float max_steer_degrees = 29.0;	//< maximum steering angle (degrees)
  const float max_steer_radians = angles::from_degrees(max_steer_degrees);
  const float turn_radius = wheelbase / tanf(max_steer_radians);

  const float front_outer_wheel_turn_radius =
    sqrtf(powf(wheelbase,2)+
	  powf(turn_radius+halfwidth,2));
  
  const float front_inner_wheel_turn_radius =
    sqrtf(powf(wheelbase,2)+
	  powf(turn_radius-halfwidth,2));

  const float rear_outer_wheel_turn_radius =
    turn_radius+halfwidth;

  const float rear_inner_wheel_turn_radius =
    turn_radius-halfwidth;

  const float front_outer_bumper_turn_radius =
    sqrtf(powf(front_bumper_px,2)+
	  powf(turn_radius+halfwidth,2));
  
  const float front_inner_bumper_turn_radius =
    sqrtf(powf(front_bumper_px,2)+
	  powf(turn_radius-halfwidth,2));

  const float rear_outer_bumper_turn_radius =
    sqrtf(powf(rear_bumper_px,2)+
	  powf(turn_radius+halfwidth,2));

  const float rear_inner_bumper_turn_radius =
    sqrtf(powf(rear_bumper_px,2)+
	  powf(turn_radius-halfwidth,2));

};

#endif
