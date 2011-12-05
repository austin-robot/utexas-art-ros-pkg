/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     Hand-written configuration class for observers.

     This is compatible with a dynamic reconfigure interface, so one
     could easily be provided, if needed.

     @author Jack O'Quin

 */

#ifndef _OBSERVERS_CONFIG_H_
#define _OBSERVERS_CONFIG_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace art_observers
{

  // Hand-written configuration class for observers.
  class ObserversConfig
  {
  public:
    /// default constructor
    ObserversConfig():
      map_frame_id(std::string("/map")),
      robot_frame_id(std::string("vehicle"))
    {};
    ObserversConfig(const ObserversConfig &that)
    {
      *this = that;
    };
    ObserversConfig(ros::NodeHandle priv_nh)
    {
      // get configuration parameters
      priv_nh.param("map_frame_id", map_frame_id, std::string("/map"));
      priv_nh.param("robot_frame_id", robot_frame_id, std::string("vehicle"));

      // apply tf_prefix to robot frame ID, if needed
      std::string tf_prefix = tf::getPrefixParam(priv_nh);
      robot_frame_id = tf::resolve(tf_prefix, robot_frame_id);

      ROS_INFO_STREAM("map frame: " << map_frame_id
		      << ", robot frame: " << robot_frame_id);
    };

    std::string map_frame_id;		///< frame ID of map
    std::string robot_frame_id;		///< frame ID of robot
  };

}; // namespace art_observers

#endif // _OBSERVERS_CONFIG_H_
