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

//==============================================================================================================

art_msgs::Observation
LaneObserver::updateAdj(int adj_poly_id,
                       art_msgs::ArtLanes &adj_lane_quads,
                       art_msgs::ArtLanes &adj_obstacle_quads) 
{ 
  if(adj_poly_id == -1) {
    observation_.distance = 80.0;
    observation_.velocity = 0.0;
    observation_.time = 0.0;
    observation_.clear = false;
    observation_.applicable = false;
    return observation_;
  }
  if (adj_obstacle_quads.polygons.size()!=0) {
    int index_adj;
    for (unsigned i = 0; i < adj_lane_quads.polygons.size() ; i++)
    {
      if (adj_lane_quads.polygons[i].poly_id == adj_poly_id) 
      { 
        index_adj = (int) i;
        break;
      } 
    }
    art_msgs::ArtLanes front_adj_lane;
    art_msgs::ArtLanes rear_adj_lane;
    if (adj_lane_quads.polygons[index_adj+1].poly_id > adj_lane_quads.polygons[index_adj].poly_id)
    {
       front_adj_lane.polygons.resize(adj_lane_quads.polygons.size() - index_adj + 1); 
       for (unsigned i = index_adj; i < adj_lane_quads.polygons.size(); i++) 
       {
          front_adj_lane.polygons[i-index_adj] = adj_lane_quads.polygons[i];
       }
       rear_adj_lane.polygons.resize(index_adj + 1);
       for (int i = 0; i < index_adj; i++)
       {
          rear_adj_lane.polygons[i] = adj_lane_quads.polygons[i];
       }
    }
    else
    { 
       front_adj_lane.polygons.resize(index_adj + 1);
       for (int i = 0; i < index_adj; i++)
       {
          front_adj_lane.polygons[i] = adj_lane_quads.polygons[i];
       }
       rear_adj_lane.polygons.resize(adj_lane_quads.polygons.size() - index_adj + 1); 
       for (unsigned i = index_adj; i < adj_lane_quads.polygons.size(); i++) 
       {
          rear_adj_lane.polygons[i] = adj_lane_quads.polygons[i];
       }
    }
    art_msgs::ArtLanes front_adj_obstacles;
    art_msgs::ArtLanes rear_adj_obstacles; 
    front_adj_obstacles.polygons.resize (front_adj_lane.polygons.size()); 
    rear_adj_obstacles.polygons.resize (rear_adj_lane.polygons.size()); 
    int front_counter_ = 0; 
    int rear_counter_ = 0;     
    for (unsigned i = 0; i < adj_obstacle_quads.polygons.size(); i++) 
    { 
      if(adj_obstacle_quads.polygons[i].poly_id > adj_lane_quads.polygons[index_adj].poly_id)
        {
          front_counter_++;
	  front_adj_obstacles.polygons[i] = adj_obstacle_quads.polygons[i];
        }
      else 
        {
          rear_counter_++;
	  rear_adj_obstacles.polygons[i] = adj_obstacle_quads.polygons[i];
        }
    }
    front_adj_obstacles.polygons.resize (front_counter_);
    rear_adj_obstacles.polygons.resize (rear_counter_);
    art_msgs::Observation front_obs = updateAdjHelper(index_adj, front_adj_lane, front_adj_obstacles);
    art_msgs::Observation rear_obs = updateAdjHelper(index_adj, rear_adj_lane, rear_adj_obstacles);  
    if (rear_obs.time > front_obs.time)     // determines which adjacent observation (front or rear) is more relevant to publish
    {                                        // if front_obs.time = rear_obs.time, then the front_obs will be published
      observation_ =  rear_obs;
    } else {
      observation_ =  front_obs; 
    }
  }
  return observation_;
}

art_msgs::Observation
  LaneObserver::updateAdjHelper(int adj_poly_id, 
                             art_msgs::ArtLanes &adj_lane_quads,
                             art_msgs::ArtLanes &adj_obstacle_quads)
{
  current_update_ = ros::Time::now();

  float distance=80.0;
  if (adj_obstacle_quads.polygons.size()!=0) {
    // Get length of road from robot to nearest obstacle
    int target_id = adj_obstacle_quads.polygons[0].poly_id;
    distance=0;
    for (size_t i=0; i<adj_lane_quads.polygons.size(); i++) {
      distance+= adj_lane_quads.polygons[i].length;
      if (adj_lane_quads.polygons[i].poly_id == target_id) {
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
  if (time == -1 || time >= 10) {   // obstacle is moving away OR is at least 10 seconds away
    clear = true;
  }

  // Do i really know enough to publish data ?
  bool applicable=false;
  if (velocity_filter_.isFull())
    applicable = true;
    
  // Publish observeration
  observation_.distance = filt_distance;
  observation_.velocity = filt_velocity;
  observation_.time = time;
  observation_.clear = clear;
  observation_.applicable = applicable;
                   
  return observation_;
}

// Updates: cleaned up updateAdj and revised a bit of updateAdjHelper (see comments); code compiles with only a warning (lane_observer.cc:172: warning: control reaches end of non-void function) not sure what this warning signifies; still need to ensure that rear observer returns a negative distance; segmentation fault when trying to run with StageSim

