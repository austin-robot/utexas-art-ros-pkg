/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART observer map grid observations interface.

     @author Michael Quinlan
     @author Jack O'Quin

 */

#ifndef _LANE_OBSERVATIONS_H_
#define _LANE_OBSERVATIONS_H_

#include <vector>
#include <tr1/unordered_set>

#include <ros/ros.h>

#include <art_msgs/ArtLanes.h>
#include <art_msgs/ObservationArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <art_observers/QuadrilateralOps.h>

#include <art_observers/nearest_forward.h>

class LaneObservations 
{
public:

  LaneObservations(ros::NodeHandle &node);
  ~LaneObservations();
  void spin();

private:

  void addObserver(Observer &obs)
  {
    observers_.push_back(&obs);
    observations_.obs.push_back(art_msgs::Observation());
  }

  void calcRobotPolygon();
  void filterPointsInLocalMap();
  bool isPointInAPolygon(float x, float y);
  void processLocalMap(const art_msgs::ArtLanes &msg);
  void processObstacles(const sensor_msgs::PointCloud &msg);
  void processPose(const nav_msgs::Odometry &odom);
  void publishObstacleVisualization();
  void runObservers();
  void transformPointCloud(const sensor_msgs::PointCloud &msg);
  int getClosestPoly(const std::vector<poly>& polys, float x, float y);
  int getClosestPoly(const std::vector<poly>& polys, MapXY pt)
  {
    return getClosestPoly(polys, pt.x, pt.y);
  }
  int getClosestPoly(const std::vector<poly>& polys,
                     const MapPose &pose)
  {
    return getClosestPoly(polys, pose.map.x, pose.map.y);
  }

  ros::NodeHandle node_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;

  observers::NearestForward nearest_forward_observer_;
#if 0
  LaneObserver nearest_rear_observer_;
  LaneObserver adjacent_left_observer_;
  LaneObserver adjacent_right_observer_;
#endif

  ros::Subscriber obstacle_sub_;
  ros::Subscriber road_map_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher observations_pub_;
  ros::Publisher viz_pub_;

  sensor_msgs::PointCloud obstacles_;
  art_msgs::ArtLanes local_map_;
  MapPose pose_;

  // vector of observers, in order of the observations they publish
  std::vector<Observer *> observers_;
  art_msgs::ObservationArray observations_;

  std::tr1::unordered_set<int> added_quads_;
  art_msgs::ArtLanes obs_quads_;
  std::vector<art_msgs::ArtQuadrilateral>::iterator obs_it_;
  art_msgs::ArtQuadrilateral robot_polygon_;

  /// Only used within LaneObservations::publishObstacleVisualization(), a
  /// class variable only to avoid memory allocation on every cycle.
  visualization_msgs::MarkerArray marks_msg_;
};

#endif // _LANE_OBSERVATIONS_H_
