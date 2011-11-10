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

  // Check to see if there is a left lane to check
  if(adj_polys_left.size() == 0) {
     return observation_;
  }



  float distance=80.0;
  if (adj_lane_obstacles.polygons.size()!=0) {
    // Get length of road from robot to nearest obstacle
    int target_id = adj_lane_obstacles.polygons[0].poly_id;
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
  ros::Time current_update(ros::Time::now());
  double time_change = (current_update - prev_update_).toSec();
  float velocity = (filt_distance - prev_distance) / (time_change);
  float filt_velocity;
  velocity_filter_.update(velocity,filt_velocity);
  prev_update_ = current_update; // Reset prev_update time

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
  
  return observation_;
}

}; // namespace observers
