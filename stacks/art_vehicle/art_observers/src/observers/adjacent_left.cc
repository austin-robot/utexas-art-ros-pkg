/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology, Michael Quinlan
 *  Copyright (C) 2011 UT-Austin & Austin Robot Technology
 *  License: Modified BSD Software License 
 */

/**  @file

     Nearest forward observer implementation.

     @author Michael Quinlan, Jack O'Quin, Corbyn Salisbury

 */

#include <art_observers/QuadrilateralOps.h>
#include <art_observers/adjacent_left.h>

namespace observers
{

AdjacentLeft::AdjacentLeft(art_observers::ObserversConfig &config):
  Observer(config,
	   art_msgs::Observation::Adjacent_left,
	   std::string("Adjacent Left"))
{
  distance_filter_.configure();
  velocity_filter_.configure();
}

AdjacentLeft::~AdjacentLeft()
{
}

art_msgs::Observation
  AdjacentLeft::update(const art_msgs::ArtQuadrilateral &robot_quad,
			 const art_msgs::ArtLanes &local_map,
			 const art_msgs::ArtLanes &obstacles,
			 MapPose pose_)
{
  observation_.distance = 0;
  observation_.velocity = 0;
  observation_.time = 0;
  observation_.clear = false;
  observation_.applicable = false;


  art_msgs::ArtLanes adj_lane_quads = quad_ops::filterAdjacentLanes
					(pose_, local_map, 1);

  art_msgs::ArtLanes adj_lane_obstacles = getObstaclesInLane(obstacles, adj_lane_quads);
  
  // Finding closest poly in left lane
  PolyOps polyOps_left;
  std::vector<poly> adj_polys_left;
  int adjacent_poly_id;
  polyOps_left.GetPolys(adj_lane_quads, adj_polys_left);
  adjacent_poly_id = polyOps_left.getClosestPoly(adj_polys_left, robot_quad.midpoint.x, robot_quad.midpoint.y);

  // Get index of adjacent polygon in left lane
  int index_adj;
  for (unsigned i = 0; i < adj_lane_quads.polygons.size() ; i++){
    if (adj_lane_quads.polygons[i].poly_id == adjacent_poly_id) { 
      index_adj = (int) i;
      break;
    } 
  }

  art_msgs::ArtLanes front_adj_lane;
  art_msgs::ArtLanes rear_adj_lane;
  // Which way are we going down the lane?
  if (adj_lane_quads.polygons[index_adj+1].poly_id > adj_lane_quads.polygons[index_adj].poly_id){
    front_adj_lane.polygons.resize(adj_lane_quads.polygons.size() - index_adj + 1); 
    for (unsigned i = index_adj; i < adj_lane_quads.polygons.size(); i++) {
      front_adj_lane.polygons[i-index_adj] = adj_lane_quads.polygons[i];
    }

    rear_adj_lane.polygons.resize(index_adj + 1);
    for (int i = 0; i < index_adj; i++){
       rear_adj_lane.polygons[i] = adj_lane_quads.polygons[i];
    }
  }
  else { 
    front_adj_lane.polygons.resize(index_adj + 1);
    for (int i = 0; i < index_adj; i++){
       front_adj_lane.polygons[i] = adj_lane_quads.polygons[i];
    }

    rear_adj_lane.polygons.resize(adj_lane_quads.polygons.size() - index_adj + 1); 
    for (unsigned i = index_adj; i < adj_lane_quads.polygons.size(); i++) {
       rear_adj_lane.polygons[i] = adj_lane_quads.polygons[i];
    }
  }

  art_msgs::ArtLanes front_adj_obstacles;
  art_msgs::ArtLanes rear_adj_obstacles; 
  front_adj_obstacles.polygons.resize (front_adj_lane.polygons.size()); 
  rear_adj_obstacles.polygons.resize (rear_adj_lane.polygons.size()); 
  int front_counter_ = 0; 
  int rear_counter_ = 0;     
  for (unsigned i = 0; i < adj_lane_obstacles.polygons.size(); i++) { 
    if(adj_lane_obstacles.polygons[i].poly_id > adj_lane_quads.polygons[index_adj].poly_id){
	front_adj_obstacles.polygons[front_counter_] = adj_lane_obstacles.polygons[i];
	front_counter_++;
    } else {
	rear_adj_obstacles.polygons[rear_counter_] = adj_lane_obstacles.polygons[i];
        rear_counter_++;
    }
  }
  front_adj_obstacles.polygons.resize (front_counter_);
  rear_adj_obstacles.polygons.resize (rear_counter_);
  art_msgs::Observation front_obs = updateAdjHelper(index_adj, front_adj_lane, front_adj_obstacles);
  art_msgs::Observation rear_obs = updateAdjHelper(index_adj, rear_adj_lane, rear_adj_obstacles);  
  if (rear_obs.time > front_obs.time) {    // determines which adjacent observation (front or rear) is more relevant to publish if front_obs.time = rear_obs.time, then the front_obs will be published
    observation_ =  rear_obs;
  } else {
    observation_ =  front_obs; 
  }

  return observation_;
}


art_msgs::Observation
  AdjacentLeft::updateAdjHelper(int adj_poly_id, 
                             art_msgs::ArtLanes &adj_lane_quads,
                             art_msgs::ArtLanes &adj_obstacle_quads)
{
  ros::Time current_update_(ros::Time::now());

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
}; // namespace observers
