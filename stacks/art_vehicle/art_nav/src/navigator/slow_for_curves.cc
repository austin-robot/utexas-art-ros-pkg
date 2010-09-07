/*
 *  Navigator "slow down for curves" controller
 *
 *  Copyright (C) 2007, Mickey Ristroph
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <float.h>

#include <art/epsilon.h>
#include <art_servo/steering.h>
#include <art_map/coordinates.h>

#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "slow_for_curves.h"

void SlowForCurves::configure()
{
  ros::NodeHandle nh("~");
  using art_common::ArtVehicle;

  // TODO: lookahead_distance should probably be time in seconds.
  nh.param("lookahead_distance", lookahead_distance, 100.0);
  nh.param("max_deceleration", max_deceleration, 0.4);
  nh.param("min_speed_when_slowing_for_curves", 
           min_speed_when_slowing_for_curves, 3.0);
  nh.param("max_yaw_rate", max_yaw_rate, 
           Steering::angle_to_yaw(min_speed_when_slowing_for_curves,
                                  ArtVehicle::max_steer_degrees)); 
  nh.param("min_curve_length", min_curve_length, 1.5);

  ROS_INFO("turn lookahead distance is %.3f meters", lookahead_distance);
  ROS_INFO("turn max yaw rate is %.3f radians/second", max_yaw_rate);
  ROS_INFO("turn max deceleration is %.3f meters/second^2", max_deceleration);
  ROS_INFO("turn min speed when slowing for curves is %.3f meters/second",
           min_speed_when_slowing_for_curves);
  ROS_INFO("turn min curve length is %.3f meters", min_curve_length);
}

Controller::result_t SlowForCurves::control(pilot_command_t &pcmd)
{
  
  if (pcmd.velocity < min_speed_when_slowing_for_curves)
    {
      if (verbose>=3)
	ART_MSG(3,"Already going slow: %.3f",pcmd.velocity);
      return OK;
    }
  

  // These indices are checked in max_safe_speed
  int start_index = pops->getClosestPoly(course->plan,
                                         MapPose(estimate->pose.pose));

  int stop_index = pops->index_of_downstream_poly(course->plan,
						  start_index,
						  lookahead_distance);
	
  float max_speed = max_safe_speed(course->plan,
				   start_index,
				   stop_index,
				   pcmd.velocity);

  // Never slow down below min_speed... if the commanded velocity is larger.
  // This prevents a bogusly sharp turn bring the car to a complete stop.
  max_speed = fmaxf(min_speed_when_slowing_for_curves, max_speed);
	
  // XXX: Scale the yawRate. Is this the right thing to do? Test.
  // PFB: Not needed because heading slowdown will be taken care of
  // later/

  //pcmd.yawRate = pcmd.yawRate * (max_speed/pcmd.velocity);

  pcmd.velocity =  max_speed;
	
  trace("slow_for_curves controller", pcmd);

  return OK;				// always successful
}

float SlowForCurves::max_safe_speed(const std::vector<poly>& polygons,
				    const int& start_index,
				    const int& stop_index,
				    const float& max) {
  //Check for bogus input
  if(start_index < 0 || start_index >= (int)polygons.size() ||
     stop_index < 0 || stop_index >= (int)polygons.size() ||
     start_index >= stop_index)
    {
      ART_MSG(3,"bogus input: start_index %d and stop_index %d",start_index, stop_index);
      return -1;
    }

  // MGR: I am not totally sure the logic in the function is what we
  // want. It is an attempt to find the highest speed we could be
  // travelling right now, and still be able to slow down enough
  // to satisfy the speed constraints for every curve ahead of us.

  // The dheading and corrosponding distance may be mismatched.

  // TODO: Try using to indexs into the array, and keeping them X meters
  // apart. Call the other functions on that distance and that change in
  // heading.

  if (verbose >= 4)
    ART_MSG(8, "slow_for_curves: searching over polygons %d(%s) to %d(%s)",
	    polygons.at(start_index).poly_id,
	    polygons.at(start_index).start_way.name().str,
	    polygons.at(stop_index).poly_id,
	    polygons.at(stop_index).start_way.name().str);

  float distance = 0;
  float max_speed = 100;

  char limiting_string[512] = "";
  int limiting_id = 0;

  for(int begin = start_index; begin < stop_index; begin++) {
    distance += (polygons.at(begin).length +
		 polygons.at(begin+1).length)/2.0;

    float length = 0;
    int end = begin;
    while(end < stop_index && length < min_curve_length) {
      length += (polygons.at(end).length +
		 polygons.at(end+1).length)/2.0;
      end++;
    }
		
    float dheading =
      Coordinates::normalize(polygons[end].heading - polygons[begin].heading);
		
    float max_then =
      fmaxf(min_speed_when_slowing_for_curves,
	    course->max_speed_for_change_in_heading(dheading, length, 
						    max_speed, max_yaw_rate));
		
    float max_now =
      course->max_speed_for_slow_down(max_then, distance, 
				      max_speed, max_deceleration);
 
    if (max_now < max_speed) {
      limiting_id = polygons.at(begin).poly_id;
      sprintf(limiting_string, "Polygons: %d(%s)->%d(%s), (dheading: %.3f, length: %.3f, max_then: %.3f)   Max speed now is %.3f for curve in %.3f meters.",
	      polygons.at(begin).poly_id,
	      polygons.at(begin).start_way.name().str,
	      polygons.at(end).poly_id,
	      polygons.at(end).start_way.name().str,
	      dheading,
	      length,
	      max_then,
	      max_now,
	      distance);
    }

    max_speed = fminf(max_speed, max_now);
    if (Epsilon::equal(max_speed,0.0))
      return 0.0;
  }
	
  if(verbose >= 4 && current_limiting_id != limiting_id) {
    ART_MSG(2, "new limiting_factor: %s", limiting_string);
    current_limiting_id = limiting_id;
  }

  max_speed = fminf(max_speed, max);

  return max_speed;
}


void SlowForCurves::reset(void) {
  trace_reset("SlowForCurves");
  current_limiting_id = 0;
}
