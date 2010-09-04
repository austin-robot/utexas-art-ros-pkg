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
   * This is inherently a two-dimensional calculation, assuming
   * constant covariance, velocity and yaw rate.
   *
   * @param[in] odom odometry on which to base estimate, with time stamp
   * @param est_time time for which estimated odometry desired
   * @param est estimated odometry for time in @a est.header.stamp
   * 
   * @todo Extrapolate speed and yaw rate using change from last
   * cycle.  Use average of current and estimated derivatives to
   * estimate control position.  When the car is accelerating, look
   * farther ahead.
   */
  void control_pose(const nav_msgs::Odometry &odom,
                    ros::Time est_time,
                    nav_msgs::Odometry &est)
  {
    // copy entire odom message, set estimate time stamp
    // (with covariance, z dimension, velocity and yaw rate)
    est = odom;
    est.header.stamp = est_time;

    if (odom.header.stamp == ros::Time() || est_time == ros::Time())
      {
        ROS_WARN_STREAM("invalid estimate time stamp, odom: "
                        << odom.header.stamp
                        << ", est_time: " << est_time);
        return;                         // return unmodified odom
      }

    double dt = est_time.toSec() - odom.header.stamp.toSec();
    if (dt < 0.0 || dt > 1.0)
      {
        ROS_WARN("bogus delta time for estimate: %.3f", dt);
        return;                         // return unmodified odom
      }

#if 1  // seems to be working now...

    // Refine pose estimate based on last reported velocity and yaw
    // rate: how far has the car travelled and its heading changed?

    double odom_yaw = tf::getYaw(odom.pose.pose.orientation);
    double odom_yawrate = odom.twist.twist.angular.z;
    double est_dist = odom.twist.twist.linear.x * dt;
    double est_yaw = Coordinates::normalize(odom_yaw
                                            + odom_yawrate * dt);
    if (fabs(odom_yawrate) < Epsilon::yaw)
      {
	// estimate straight line path at current velocity and heading
	est.pose.pose.position.x = (odom.pose.pose.position.x
                                    + est_dist * cos(odom_yaw));
	est.pose.pose.position.y = (odom.pose.pose.position.y
                                    + est_dist * sin(odom_yaw));

        ROS_DEBUG("estimated straight path distance = %.3f", est_dist);
      }
    else					// turning
      {
	// Car is turning. Estimate circular path [see: _Probabilistic
	// Robotics_, Thrun, Burgard and Fox, ISBN 0-262-20162-3,
	// 2005; section 5.3.3, exact motion model, pp. 125-127].
	double est_radius = est_dist / odom_yawrate;
	est.pose.pose.position.x = (odom.pose.pose.position.x
                                    - est_radius * sin(odom_yaw)
                                    + est_radius * sin(est_yaw));
	est.pose.pose.position.y = (odom.pose.pose.position.y
                                    + est_radius * cos(odom_yaw)
                                    - est_radius * cos(est_yaw));

        ROS_DEBUG("estimated path distance = %.3f, radius = %.3f",
		  est_dist, est_radius);
      }

    ROS_DEBUG("estimated control pose = (%.3f, %.3f, %.3f)",
	      est.pose.pose.position.x,
	      est.pose.pose.position.y,
              est_yaw);

    // @todo Preserve roll and pitch in the estimated quaternion.
    // This implementation sets them to zero, which does not matter
    // for the current navigator logic.
    est.pose.pose.orientation = tf::createQuaternionMsgFromYaw(est_yaw);

#if 0  // for detailed debugging only, extremely verbose:
    ROS_INFO_STREAM("last reported Odometry: " << odom);
    ROS_INFO_STREAM("estimated Odometry: " << est);
#endif // for detailed debugging only
#endif
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
