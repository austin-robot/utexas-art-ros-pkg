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
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <art_observers/nearest_forward.h>
#include <art_observers/nearest_backward.h>

#include <art_observers/ObserversConfig.h>
typedef art_observers::ObserversConfig Config;

class LaneObservations 
{
public:

  LaneObservations(ros::NodeHandle &node,
		   ros::NodeHandle &priv_nh);
  ~LaneObservations();
  void spin();

private:

  void addObserver(observers::Observer &obs)
  {
    observers_.push_back(&obs);
    observations_.obs.push_back(art_msgs::Observation());
  }

  void calcRobotPolygon();
  void filterPointsInLocalMap();
  bool isPointInAPolygon(float x, float y);
  void processLocalMap(const art_msgs::ArtLanes &msg);
  void processObstacles(const sensor_msgs::PointCloud &msg);
  void publishObstacleVisualization();
  void runObservers();
  void transformPointCloud(const sensor_msgs::PointCloud &msg);

  ros::NodeHandle node_;
  ros::NodeHandle priv_nh_;

  Config config_;			///< configuration parameters

  // observer instances
  observers::NearestForward nearest_forward_observer_;
  observers::NearestBackward nearest_backward_observer_;

  boost::shared_ptr<tf::TransformListener> tf_listener_;

  ros::Subscriber obstacle_sub_;
  ros::Subscriber road_map_sub_;
  ros::Publisher observations_pub_;
  ros::Publisher viz_pub_;

  sensor_msgs::PointCloud obstacles_;
  art_msgs::ArtLanes local_map_;

  // vector of observers, in order of the observations they publish
  std::vector<observers::Observer *> observers_;
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
