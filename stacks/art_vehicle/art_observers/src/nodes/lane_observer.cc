/*
 *  Copyright (C) 2010 UT-Austin &  Austin Robot Technology, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */

/**  @file
 
     ART lane observers class implementation.

     @author Michael Quinlan

 */

#include <art_observers/lane_observer.h>

LaneObserver::LaneObserver(Oid_t id, const std::string &name):
  Observer(id, name)
{
  distance_filter_.configure();
  velocity_filter_.configure();
  
  prev_update_ = ros::Time::now();
  current_update_ = ros::Time::now();
}

LaneObserver::~LaneObserver() 
{
}

art_msgs::Observation
  LaneObserver::update(int origin_poly_id,
                       art_msgs::ArtLanes &lane_quads,
                       art_msgs::ArtLanes &obstacle_quads) 
{
  current_update_ = ros::Time::now();
  if (current_update_ == prev_update_) return observation_;

  float distance=80.0;
  if (obstacle_quads.polygons.size()!=0) {
    // Get length of road from robot to nearest obstacle
    int target_id = obstacle_quads.polygons[0].poly_id;
    distance=0;
    for (size_t i=0; i<lane_quads.polygons.size(); i++) {
      distance+=lane_quads.polygons[i].length;
      if (lane_quads.polygons[i].poly_id == target_id) {
        break;
      }
    }
  }
  // Filter the distance
  float filt_distance;
  distance_filter_.update(distance, filt_distance);
  
  // Calculate velocity of object (including filter)
  float prev_distance = observation_.distance;
  double time_change = (current_update_ - prev_update_).toSec();
  float velocity = (filt_distance - prev_distance) / (time_change);
  float filt_velocity;
  velocity_filter_.update(velocity,filt_velocity);
  prev_update_ = current_update_; // Reset prev_update time

  //Time to intersection
  double time = -1; // Default if obstacle is moving away (i.e no intersection)
  if (filt_velocity < 0) {  // Object is getting closer, we will intersect
    if (filt_velocity > -0.1) // lets not diving by a tiny number
      filt_velocity = 0.1;
    time = fabs(filt_distance / filt_velocity);
  }

  // Am I clear, I.e. I won't hit anything
  bool clear=false;
  if (time == -1) {
    clear = true;
  }

  // Do i really know enough to publish data ?
  bool applicable=false;
  if (velocity_filter_.isFull())
    applicable = true;
    
  // Publish a nearest front observeration
  observation_.distance = filt_distance;
  observation_.velocity = filt_velocity;
  observation_.time = time;
  observation_.clear = clear;
  observation_.applicable = applicable;
                   
  return observation_;
}

