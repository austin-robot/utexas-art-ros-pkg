/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id: accel.h 1539 2011-05-09 04:09:20Z jack.oquin $
 */

/**  @file
 
     ART observer map grid interface.

     @author Michael Quinlan

 */

#ifndef _MAP_GRID_H_
#define _MAP_GRID_H_

#include <vector>
#include <tr1/unordered_set>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <art_msgs/ArtLanes.h>
#include <art_msgs/Observation.h>

#include <art_observers/QuadrilateralOps.h>
#include <art_observers/lane_observer.h>

class MapGrid 
{
public:

  MapGrid(ros::NodeHandle* node);
  ~MapGrid();

  void processObstacles(const sensor_msgs::PointCloud &msg);
  void processLocalMap(const art_msgs::ArtLanes &msg);

private:
  void transformPointCloud(const sensor_msgs::PointCloud &msg);
  void filterPointsInLocalMap();
  bool isPointInAPolygon(float x, float y);

  void runObservers();
  void calcRobotPolygon();

  void publishObstacleVisualization();

  LaneObserver nearest_front_observer_;
  LaneObserver nearest_rear_observer_;

  sensor_msgs::PointCloud transformed_obstacles_;
  art_msgs::ArtLanes local_map_;

  std::tr1::unordered_set<int> added_quads_;

  art_msgs::ArtLanes obs_quads_;
  std::vector<art_msgs::ArtQuadrilateral>::iterator obs_it_;

  art_msgs::ArtQuadrilateral robot_polygon_;

  tf::TransformListener* tf_listener_;
  visualization_msgs::MarkerArray marks_msg_;

  ros::Publisher nearest_front_publisher_;
  ros::Publisher nearest_rear_publisher_;

  ros::Publisher visualization_publisher_;

  ros::NodeHandle* node_;
};

#endif // _MAP_GRID_H_
