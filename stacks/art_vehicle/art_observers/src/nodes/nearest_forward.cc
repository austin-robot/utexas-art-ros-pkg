/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology, Michael Quinlan
 *  Copyright (C) 2011 UT-Austin & Austin Robot Technology
 *  License: Modified BSD Software License 
 */

/**  @file

     Nearest forward observer implementation.

     @author Michael Quinlan, Jack O'Quin

 */

#include <art_observers/nearest_forward.h>

namespace observers
{

NearestForward::NearestForward(Oid_t id, const std::string &name):
  Observer(id, name)
{
  distance_filter_.configure();
  velocity_filter_.configure();
}

NearestForward::~NearestForward() 
{
}

art_msgs::Observation
  NearestForward::update(int origin_poly_id,
                       art_msgs::ArtLanes &lane_quads,
                       art_msgs::ArtLanes &obstacle_quads) 
{
  float distance = 80.0;
  if (obstacle_quads.polygons.size()!=0)
    {
      // Get distance along road from robot to nearest obstacle
      int target_id = obstacle_quads.polygons[0].poly_id;
      distance = 0;
      for (size_t i=0; i<lane_quads.polygons.size(); i++)
	{
	  distance+=lane_quads.polygons[i].length;
	  if (lane_quads.polygons[i].poly_id == target_id)
	    {
	      break;
	    }
	}
    }
  // Filter the distance
  float filt_distance;
  distance_filter_.update(distance, filt_distance);
  
  // Calculate velocity of object (including filter)
  float prev_distance = observation_.distance;
  ros::Time current_update(ros::Time::now());
  double time_change = (current_update - prev_update_).toSec();
  float velocity = (filt_distance - prev_distance) / (time_change);
  float filt_velocity;
  velocity_filter_.update(velocity,filt_velocity);
  prev_update_ = current_update; // Reset prev_update time

  // Time to intersection (-1 if obstacle is moving away)
  double time = -1;
  if (filt_velocity < 0)      // Object getting closer, will intersect
    {
      if (filt_velocity > -0.1)	    // avoid dividing by a tiny number
	{
	  filt_velocity = 0.1;
	}
      time = fabs(filt_distance / filt_velocity);
    }

  // Am I clear, I.e. I won't hit anything
  bool clear=false;
  if (time == -1)
    {
      clear = true;
    }

  // Do I really know enough to publish data ?
  bool applicable=false;
  if (velocity_filter_.isFull())
    applicable = true;
    
  // return the observation
  observation_.distance = filt_distance;
  observation_.velocity = filt_velocity;
  observation_.time = time;
  observation_.clear = clear;
  observation_.applicable = applicable;
                   
  return observation_;
}

}; // namespace observers
