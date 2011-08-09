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
     @author Jack O'Quin

 */

#ifndef _MAP_GRID_H_
#define _MAP_GRID_H_

#include <vector>
#include <tr1/unordered_set>

#include <ros/ros.h>
#include <art_msgs/ArtLanes.h>
#include <art_msgs/ObservationArray.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <art_observers/lane_observer.h>
#include <art_observers/QuadrilateralOps.h>

class MapGrid 
{
public:

  MapGrid(ros::NodeHandle &node);
  ~MapGrid();
  void spin();

private:

  void calcRobotPolygon();
  void filterPointsInLocalMap();
  bool isPointInAPolygon(float x, float y);
  void processLocalMap(const art_msgs::ArtLanes &msg);
  void processObstacles(const sensor_msgs::PointCloud &msg);
  void publishObstacleVisualization();
  void runObservers();
  void transformPointCloud(const sensor_msgs::PointCloud &msg);

  ros::NodeHandle node_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  ros::Subscriber obstacle_sub_;
  ros::Subscriber road_map_sub_;
  ros::Publisher observations_pub_;
  ros::Publisher viz_pub_;

  LaneObserver nearest_front_observer_;
  LaneObserver nearest_rear_observer_;

  sensor_msgs::PointCloud obstacles_;
  art_msgs::ArtLanes local_map_;

  std::tr1::unordered_set<int> added_quads_;
  art_msgs::ArtLanes obs_quads_;
  std::vector<art_msgs::ArtQuadrilateral>::iterator obs_it_;
  art_msgs::ArtQuadrilateral robot_polygon_;

  /// Only used within MapGrid::publishObstacleVisualization(), a
  /// class variable only to avoid memory allocation on every cycle.
  visualization_msgs::MarkerArray marks_msg_;
};

#endif // _MAP_GRID_H_
