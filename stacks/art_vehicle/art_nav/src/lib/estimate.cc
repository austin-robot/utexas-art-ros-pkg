/*
 *  ART odometry estimator
 *
 *  Copyright (C) 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <math.h>
#include <art_common/ArtVehicle.h>
#include <art/epsilon.h>
#include <art_map/euclidean_distance.h>
#include <art_map/coordinates.h>

#include <art_nav/estimate.h>

/** @file
 *
 *  @brief ART odometry estimate functions. 
 */
namespace Estimate
{
  /** Estimate control pose from earlier odometry.
   *
   * @param[in] odom odometry on which to base estimate, with time stamp
   * @param est_time time for which estimated odometry desired
   * @param est estimated odometry for time in @a est.header.stamp
   */
  void control_pose(const nav_msgs::Odometry &odom,
                    ros::Time est_time,
                    nav_msgs::Odometry &est)
  {
#if 1 // just copy last odometry for now

    // copy most recent odom
    est = odom;

#else // figure out how to interpolate quaternions

    // assume constant velocity and yaw rate
    est.twist = odom.twist;

    MapPose odom_pose = MapPose(odom.pose.pose);
    
    // TODO: extrapolate speed and yaw rate using change from last
    // cycle.  Use average of current and estimated derivatives to
    // estimate control position.  When the car is accelerating, look
    // farther ahead.
    
    // how far has the car travelled and its heading changed?
    ros::Duration time_diff = est.header.stamp - odom.header.stamp;
    float est_dist = odom.pose.pose.position.x * time_diff.toSec();
    double odom_yawrate = odom.twist.twist.angular.z;
    MapPose est_pose;
    est_pose.yaw = Coordinates::normalize(odom_pose.yaw
                                          + odom_yawrate * time_diff);
    if (fabs(odom_yawrate) < Epsilon::yaw)
      {
	// estimate straight line path at current velocity and heading
	est_pose.map.x = odom_pose.map.x + est_dist * cos(odom.pos.pa);
	est_pose.map.y = odom_pose.map.y + est_dist * sin(odom.pos.pa);
        ROS_DEBUG("estimated path distance = %.3f", est_dist);
      }
    else					// turning
      {
	// estimate circular path -- from _Probabilistic Robotics_,
	// Thrun, Burgard and Fox, ISBN 0-262-20162-3, 2005; section
	// 5.3.3, exact motion model, pp. 125-127.
	float est_radius = est_dist / odom_yawrate;
	est_pose.map.x = (odom_pose.map.x
                          - est_radius * sin(odom_pose.yaw)
                          + est_radius * sin(est_pose.yaw));
	est_pose.map.y = (odom_pose.map.y
                          + est_radius * cos(odom_pose.yaw)
                          - est_radius * cos(est_pose.yaw));
        ROS_DEBUG("estimated path distance = %.3f, radius = %.3f",
		  est_dist, est_radius);
      }

    ROS_DEBUG("estimated control pose = (%.3f, %.3f, %.3f)",
	      est.pose.pose.position.x,
	      est.pose.pose.position.y,
              est_pose.yaw);
#endif

    // return estimate time stamp in its header
    est.header.stamp = est_time;
  }

  void front_bumper_pose(const nav_msgs::Odometry &odom, 
                         nav_msgs::Odometry &est)
  {
    Polar bump_rel(0,art_common::ArtVehicle::front_bumper_px);
    MapXY bump_abs =
      Coordinates::Polar_to_MapXY(bump_rel, MapPose(odom.pose.pose));
    est = odom;
    est.pose.pose.position.x = bump_abs.x;
    est.pose.pose.position.y = bump_abs.y;
  }

  void front_axle_pose(const nav_msgs::Odometry &odom,
                       nav_msgs::Odometry &est)
  {
    Polar axle_rel(0,art_common::ArtVehicle::wheelbase);
    MapXY axle_abs = Coordinates::Polar_to_MapXY(axle_rel,
                                                 MapPose(odom.pose.pose));
    est = odom;
    est.pose.pose.position.x = axle_abs.x;
    est.pose.pose.position.y = axle_abs.y;
  }

} // namespace Estimate
