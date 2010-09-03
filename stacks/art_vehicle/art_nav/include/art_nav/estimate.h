/* -*- mode: C++ -*-
 *
 *  ART odometry estimator interface
 *
 *  Copyright (C) 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _ESTIMATE_H_
#define _ESTIMATE_H_

#include <nav_msgs/Odometry.h>

/** @file
 *
 *  @brief ART odometry estimate function prototypes. 
 */
namespace Estimate
{
  void control_pose(const nav_msgs::Odometry &odom,
                    ros::Time est_time,
                    nav_msgs::Odometry &est);

  void front_bumper_pose(const nav_msgs::Odometry &odom,
                         nav_msgs::Odometry &est);

  void front_axle_pose(const nav_msgs::Odometry &odom, 
                       nav_msgs::Odometry &est);
}

#endif // _ESTIMATE_H_
