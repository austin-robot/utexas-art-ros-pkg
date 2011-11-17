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
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

#include <art_observers/nearest_forward.h>
#include <art_observers/nearest_backward.h>
#include <art_observers/adjacent_left.h>
#include <art_observers/adjacent_right.h>

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

  /** @brief short name for PCL PointCloud */
  typedef pcl::PointCloud<pcl::PointXYZ> PtCloud;

  /** @brief Add an entry to the observer vector.
   *
   *  @param obs observer to add
   */
  void addObserver(observers::Observer &obs)
  {
    observers_.push_back(&obs);
    observations_.obs.push_back(art_msgs::Observation());
  }

  void calcRobotPolygon();
  void filterPointsInLocalMap();
  bool isPointInAPolygon(float x, float y);
  void processLocalMap(const art_msgs::ArtLanes::ConstPtr &msg);
  void processObstacles(void);
  void processPointCloud(const sensor_msgs::PointCloud::ConstPtr &msg);
  void processPointCloud2(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void processPose(const nav_msgs::Odometry &odom);
  void publishObstacleVisualization();
  void runObservers();
  void transformPointCloud(const PtCloud &msg);

  ros::NodeHandle node_;		///< node handle
  ros::NodeHandle priv_nh_;		///< private node handle

  Config config_;			///< configuration parameters

  // observer instances
  observers::NearestForward nearest_forward_observer_;
  observers::NearestBackward nearest_backward_observer_;
  observers::AdjacentLeft adjacent_left_observer_;
  observers::AdjacentRight adjacent_right_observer_;

  boost::shared_ptr<tf::TransformListener> tf_listener_;

  // ROS topic subscriptions and publishers
  ros::Subscriber pc_sub_;		///< deprecated PointCloud input
  ros::Subscriber pc2_sub_;		///< PointCloud2 input
  ros::Subscriber road_map_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher observations_pub_;
  ros::Publisher viz_pub_;

  PtCloud obstacles_;			///< current obstacle data
  art_msgs::ArtLanes local_map_;	///< local road map

  /// vector of observers, in order of the observations they publish
  std::vector<observers::Observer *> observers_;

  /// current observations from the observers
  art_msgs::ObservationArray observations_;

  std::tr1::unordered_set<int> added_quads_; ///< set of obstacle quads
  art_msgs::ArtLanes obs_quads_;	///< vector of obstacle quads
  std::vector<art_msgs::ArtQuadrilateral>::iterator obs_it_;
  art_msgs::ArtQuadrilateral robot_polygon_; ///< robot's current polygon
  MapPose pose_; // pose of Map

  /// Only used within LaneObservations::publishObstacleVisualization(), a
  /// class variable only to avoid memory allocation on every cycle.
  visualization_msgs::MarkerArray marks_msg_;
};

#endif // _LANE_OBSERVATIONS_H_
