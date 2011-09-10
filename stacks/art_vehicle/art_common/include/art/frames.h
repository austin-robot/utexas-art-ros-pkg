/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2009 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file
 
     \brief ART vehicle frames of reference for ROS transforms.

     \author Jack O'Quin

     @todo figure out a less klunky way to do this

 */

#ifndef _FRAMES_H_
#define _FRAMES_H_

#include <string>
#include <ros/ros.h>
#include <tf/tf.h>

namespace ArtFrames
{
  /** Earth frame of reference */
  const std::string earth =     "earth";

  /** odometry frame of reference */
  const std::string odom =      "odom";

  /** vehicle-relative frame */
  const std::string vehicle =   "vehicle";

  /** laser frames of reference */
  const std::string velodyne =  "velodyne";
  const std::string front_sick = "front_sick";
  const std::string rear_sick = "rear_sick";

  /** camera device and optical frames of reference */
  const std::string center_front_camera = "center_front_camera";
  const std::string center_front_camera_optical = "center_front_camera_optical";
  const std::string left_front_camera = "left_front_camera";
  const std::string left_front_camera_optical = "left_front_camera_optical";
  const std::string right_front_camera = "right_front_camera";
  const std::string right_front_camera_optical = "right_front_camera_optical";

  class VehicleRelative
  {
  public:
    VehicleRelative() {};

    /** \brief configure vehicle-relative transform prefix parameter */
    void getPrefixParam(void)
    {
      ros::NodeHandle private_nh("~");
      std::string prefix_param;
      if (private_nh.searchParam("tf_prefix", prefix_param))
        {
          private_nh.getParam(prefix_param, prefix_);
          ROS_INFO_STREAM("vehicle-relative transform prefix: " << prefix_);
        }
    }

    /** \brief get vehicle-relative transform frame ID
     *
     *  \param relframe frame ID relative to vehicle
     *  \returns corresponding global frame ID
     */
    inline std::string getFrame(std::string relframe)
    {
      return tf::resolve(prefix_, relframe);
    }

  private:
    std::string prefix_;                /**< vehicle frame prefix */
  };
};

#endif // _FRAMES_H_
