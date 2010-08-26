#ifndef _ESTIMATE_H_
#define _ESTIMATE_H_

#include <math.h>

#include <nav_msgs/Odometry.h>

#include <art/epsilon.h>
#include <art_common/ArtVehicle.h>
#include <art_map/euclidean_distance.h>
#include <art_map/coordinates.h>

/** @brief global ART estimate definitions.  These constants represent
 *  trivial differences in distance, speed, angle, etc.
 */
namespace Estimate
{
  inline void control_pose(const nav_msgs::Odometry &odom, 
			   double odom_time, 
			   double curr_time, 
			   nav_msgs::Odometry &est,
			   int verbose=3)
  {

    // assume constant velocity and yaw rate
    est.twist = odom.twist;
    
    // TODO: extrapolate speed and yaw rate using change from last
    // cycle.  Use average of current and estimated derivatives to
    // estimate control position.  When the car is accelerating, look
    // farther ahead.
    
    // how far has the car travelled and its heading changed?
#if 0 // figure out how to interpolate quaternions
    double time_diff = curr_time - odom_time;
    float est_dist = odom.pose.pose.position.x * time_diff;
    est.pose.orientation = Coordinates::normalize(odom.pos.pa
					+ odom.vel.pa * time_diff);
    if (fabs(odom.vel.pa) < Epsilon::yaw)
      {
	// estimate straight line path at current velocity and heading
	est.pos.px = odom.pos.px + est_dist * cos(odom.pos.pa);
	est.pos.py = odom.pos.py + est_dist * sin(odom.pos.pa);
        ROS_DEBUG("estimated path distance = %.3f", est_dist);
      }
    else					// turning
      {
	// estimate circular path -- from _Probabilistic Robotics_,
	// Thrun, Burgard and Fox, ISBN 0-262-20162-3, 2005; section
	// 5.3.3, exact motion model, pp. 125-127.
	float est_radius = est_dist / odom.vel.pa;
	est.pos.px = (odom.pos.px
		       - est_radius * sin(odom.pos.pa)
		       + est_radius * sin(est.pos.pa));
	est.pos.py = (odom.pos.py
		       + est_radius * cos(odom.pos.pa)
		       - est_radius * cos(est.pos.pa));
	if (verbose >= 3)
	  ART_MSG(5, "estimated path distance = %.3f, radius = %.3f",
		  est_dist, est_radius);
      }

    if (verbose >= 2)
      ART_MSG(5, "estimated control pose = (%.3f, %.3f, %.3f)",
	      est.pos.px, est.pos.py, est.pos.pa);
#endif
  }

  inline void front_bumper_pose(const nav_msgs::Odometry &odom, 
                                nav_msgs::Odometry &est)
  {
    Polar bump_rel(0,art_common::ArtVehicle::front_bumper_px);
    MapXY bump_abs =
      Coordinates::Polar_to_MapXY(bump_rel, MapPose(odom.pose.pose));
    est = odom;
    est.pose.pose.position.x = bump_abs.x;
    est.pose.pose.position.y = bump_abs.y;
  }

  inline void front_axle_pose(const nav_msgs::Odometry &odom, 
			      nav_msgs::Odometry &est)
  {
    Polar axle_rel(0,art_common::ArtVehicle::wheelbase);
    MapXY axle_abs = Coordinates::Polar_to_MapXY(axle_rel,
                                                 MapPose(odom.pose.pose));
    est = odom;
    est.pose.pose.position.x = axle_abs.x;
    est.pose.pose.position.y = axle_abs.y;
  }


}

#endif
