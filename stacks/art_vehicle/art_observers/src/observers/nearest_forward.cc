/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology, Michael Quinlan
 *  Copyright (C) 2011 UT-Austin & Austin Robot Technology
 *  License: Modified BSD Software License 
 */

/**  @file

     Nearest forward observer implementation.

     @author Michael Quinlan, Jack O'Quin

 */

#include <art_observers/QuadrilateralOps.h>
#include <art_observers/nearest_forward.h>

namespace observers
{

NearestForward::NearestForward(art_observers::ObserversConfig &config):
  Observer(config,
	   art_msgs::Observation::Nearest_forward,
	   std::string("Nearest_forward"))
{
  distance_filter_.configure();
  velocity_filter_.configure();
}

NearestForward::~NearestForward() 
{
}

art_msgs::Observation
  NearestForward::update(const art_msgs::ArtQuadrilateral &robot_quad,
			 const art_msgs::ArtLanes &local_map,
			 const art_msgs::ArtLanes &obstacles)
{
  // get quadrilaterals ahead in the current lane
  art_msgs::ArtLanes lane_quads =
    quad_ops::filterLanes(robot_quad, local_map,
                          *quad_ops::compare_forward_seg_lane);

  // get obstacles ahead in the current lane
  art_msgs::ArtLanes lane_obstacles =
    quad_ops::filterLanes(robot_quad, obstacles,
                          *quad_ops::compare_forward_seg_lane);

  float distance = config_.max_range;
  if (lane_obstacles.polygons.size()!=0)
    {
      // Get distance along road from robot to nearest obstacle
      int target_id = lane_obstacles.polygons[0].poly_id;
      distance = 0;
      for (size_t i = 0; i < lane_quads.polygons.size(); i++)
	{
	  distance += lane_quads.polygons[i].length;
	  if (lane_quads.polygons[i].poly_id == target_id)
	    break;
	}
    }

  // Filter the distance by averaging over time
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
