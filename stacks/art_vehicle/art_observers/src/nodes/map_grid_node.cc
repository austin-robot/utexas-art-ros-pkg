/*
 *  Copyright (C) 2010 UT-Austin &  Austin Robot Technology, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */

/** \file

  This node creates a pseudo occupancy grid ontop of the maplanes data struture

*/

#include <ros/ros.h>
#include <art_obstacles/map_grid.h>
#include <art_msgs/Observation.h>
#include <visualization_msgs/MarkerArray.h>

#define NODE "map_grid"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  MapGrid* mGrid = new MapGrid(&node);
  // Listen to obstacle cloud
  
  ros::Subscriber velodyne_obstacles =  node.subscribe("fused_obstacles", 1,
                                                       // ros::Subscriber velodyne_obstacles =  node.subscribe("velodyne/obstacles", 1,
                                                       &MapGrid::processObstacles, mGrid,
                                                       ros::TransportHints().tcpNoDelay(true));
  // Listen to local road map
  ros::Subscriber road_map = node.subscribe("roadmap_local", 1,
                                            &MapGrid::processLocalMap, mGrid,
                                            ros::TransportHints().tcpNoDelay(true));

  ros::spin();                          // handle incoming data

  return 0;
}
