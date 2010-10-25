/*
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file
 
     Model speed and turn rate of the ART autonomous vehicle.

     \author Jack O'Quin

 */

#include <angles/angles.h>

#include <art/conversions.h>
#include <art/frames.h>
#include <art/UTM.h>

#include <art_msgs/BrakeState.h>
#include <art_msgs/Shifter.h>
#include <art_msgs/SteeringState.h>
#include <art_msgs/ThrottleState.h>
#include <art/steering.h>

#include <art_msgs/GpsInfo.h>

#include "vehicle_model.h"

void ArtVehicleModel::setup(void)
{
  int qDepth = 1;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  odom_pub_ =
    node_.advertise<nav_msgs::Odometry>(ns_prefix_ + "odom", qDepth);
  ground_truth_pub_ =
    node_.advertise<nav_msgs::Odometry>(ns_prefix_ + "ground_truth", qDepth);
  imu_pub_ =
    node_.advertise<sensor_msgs::Imu>(ns_prefix_ + "imu", qDepth);
  gps_pub_ =
    node_.advertise<art_msgs::GpsInfo>(ns_prefix_ + "gps", qDepth);
  
  // servo state topics
  brake_sub_ =
    node_.subscribe(ns_prefix_ + "brake/state", qDepth,
                    &ArtVehicleModel::brakeReceived, this, noDelay);
  shifter_sub_ =
    node_.subscribe(ns_prefix_ + "shifter/state", qDepth,
                    &ArtVehicleModel::shifterReceived, this, noDelay);
  steering_sub_ =
    node_.subscribe(ns_prefix_ + "steering/state", qDepth,
                    &ArtVehicleModel::steeringReceived, this, noDelay);
  throttle_sub_ =
    node_.subscribe(ns_prefix_ + "throttle/state", qDepth,
                    &ArtVehicleModel::throttleReceived, this, noDelay);

  // set default GPS origin, from SwRI site visit in San Antonio
  ros::NodeHandle private_nh("~");
  private_nh.param("latitude",  origin_lat_,   29.446018);
  private_nh.param("longitude", origin_long_, -98.607024);
  private_nh.param("elevation", origin_elev_, 100.0);
  ROS_INFO("map GPS origin: latitude %.6f, longitude %.6f, elevation %.1f m",
           origin_lat_, origin_long_, origin_elev_);

  // Convert latitude and longitude of map origin to UTM.
  UTM::LLtoUTM(origin_lat_, origin_long_,
               origin_northing_, origin_easting_, origin_zone_);
  ROS_INFO("Map origin UTM: %s, %.2f, %.2f",
           origin_zone_, origin_easting_, origin_northing_);

  // Round UTM origin of map to nearest UTM grid intersection.
  double grid_x = (rint(origin_easting_/UTM::grid_size) * UTM::grid_size);
  double grid_y = (rint(origin_northing_/UTM::grid_size) * UTM::grid_size);
  ROS_INFO("UTM grid origin: (%.f, %.f)", grid_x , grid_y);

  // Report stage-generated odometry relative to that location.
  map_origin_x_ = origin_easting_ - grid_x;
  map_origin_y_ = origin_northing_ - grid_y;
  ROS_INFO("MapXY origin: (%.f, %.f)", map_origin_x_ , map_origin_y_);
}

// Servo device interfaces.
//
// IMPORTANT: These callbacks run in a separate thread, so all class
// data updates must be done while holding the msg_lock_.
//
void
ArtVehicleModel::brakeReceived(const art_msgs::BrakeState::ConstPtr &msg)
{
  //ROS_DEBUG("brake state received: position %.3f", msg->position);
  boost::mutex::scoped_lock lock(msg_lock_);
  brake_position_ = msg->position;
}

void
ArtVehicleModel::shifterReceived(const art_msgs::Shifter::ConstPtr &msg)
{
  ROS_DEBUG("shifter state received: gear %u", msg->gear);
  boost::mutex::scoped_lock lock(msg_lock_);
  shifter_gear_ = msg->gear;
}

void 
ArtVehicleModel::steeringReceived(const art_msgs::SteeringState::ConstPtr &msg)
{
  //ROS_DEBUG("steering state received: %.1f (degrees)", msg->angle);
  boost::mutex::scoped_lock lock(msg_lock_);
  steering_angle_ = msg->angle;
}

void
ArtVehicleModel::throttleReceived(const art_msgs::ThrottleState::ConstPtr &msg)
{
  //ROS_DEBUG("throttle state received: position %.3f", msg->position);
  boost::mutex::scoped_lock lock(msg_lock_);
  throttle_position_ = msg->position;
}

/** Model vehicle acceleration
 *
 *  @pre last_update_time_ = time of previous update.
 *  @param odomVel[out] -> the Odometry message Twist component
 *  @param imuMsg[out] -> the IMU message
 *  @param sim_time[in] current simulation time
 *
 *  @note This simple model does not account for sideways slippage, so
 *        odomVel->linear.y is always zero.  Similarly, roll, pitch
 *        and odomVel->linear.z are always zero, because Stage is a 2D
 *        simulation.
 *
 *  @todo introduce some small random fluctuations
 */
void ArtVehicleModel::ModelAcceleration(geometry_msgs::Twist *odomVel,
                                        sensor_msgs::Imu *imuMsg,
                                        ros::Time sim_time)
{
  // MUST serialize with updates from incoming messages
  boost::mutex::scoped_lock lock(msg_lock_);

  double speed = fabs(odomVel->linear.x);

  // assume full brake or throttle produces 1g (9.81 m/s/s)
  // TODO: tune these coefficients using actual vehicle measurements
  static const double g = 9.81;         // acceleration due to gravity
  static const double throttle_accel = g;
  static const double brake_decel = g;
  static const double rolling_resistance = 0.01 * g;
  static const double drag_coeff = 0.01;

  // the vehicle idles at 7 MPH (3.1 m/s) with no brake or throttle
  static const double idle_accel = (rolling_resistance
                                    + drag_coeff * 3.1 * 3.1);

  double wind_resistance = drag_coeff * speed * speed;
  double accel = (idle_accel
                  + throttle_position_ * throttle_accel
                  - brake_position_ * brake_decel
                  - rolling_resistance
                  - wind_resistance);

  // compute seconds since last update (probably zero first time)
  double deltaT = ros::Duration(sim_time - last_update_time_).toSec();
  speed += accel * deltaT;              // adjust speed
  imuMsg->linear_acceleration.x = accel;

  // Brake and throttle (by themselves) never cause reverse motion.
  // Only shifting into Reverse can do that.
  if (speed < 0.0)
    {
      speed = 0.0;
      imuMsg->linear_acceleration.x = 0.0;
    }

  // Set velocity sign based on gear.
  odomVel->linear.x = speed;            // forward movement
  if (shifter_gear_ ==  art_msgs::Shifter::Reverse)
    odomVel->linear.x = -speed;         // reverse movement

  // set yaw rate (radians/second) from velocity and steering angle
  odomVel->angular.z = Steering::angle_to_yaw(odomVel->linear.x,
                                              steering_angle_);
  imuMsg->angular_velocity.z = odomVel->angular.z;

  // set simulated vehicle velocity using the "car" steering model,
  // which uses steering angle in radians instead of yaw rate.
  double angleRadians = angles::from_degrees(steering_angle_);
  ROS_DEBUG("Stage SetSpeed(%.3f, %.3f, %.3f)",
            odomVel->linear.x, odomVel->linear.y, angleRadians);
  stgp_->SetSpeed(odomVel->linear.x, odomVel->linear.y, angleRadians);
}

// update vehicle dynamics model
void ArtVehicleModel::update(ros::Time sim_time)
{
  sensor_msgs::Imu imu_msg;

  // model vehicle acceleration from servo actuators
  ModelAcceleration(&odomMsg_.twist.twist, &imu_msg, sim_time);

  // Get latest position data from Stage
  // Translate into ROS message format and publish
  odomMsg_.pose.pose.position.x = stgp_->est_pose.x + map_origin_x_;
  odomMsg_.pose.pose.position.y = stgp_->est_pose.y + map_origin_y_;
  odomMsg_.pose.pose.position.z = origin_elev_;
  odomMsg_.pose.pose.orientation =
    tf::createQuaternionMsgFromYaw(stgp_->est_pose.a);

  odomMsg_.header.stamp = sim_time;
  odomMsg_.header.frame_id = tf_prefix_ + ArtFrames::earth;
  odomMsg_.child_frame_id = tf_prefix_ + ArtFrames::vehicle;
  odom_pub_.publish(odomMsg_);

  // publish simulated IMU data
  imu_msg.header.stamp = sim_time;
  imu_msg.header.frame_id = tf_prefix_ + ArtFrames::vehicle;
  imu_msg.orientation = odomMsg_.pose.pose.orientation;
  imu_pub_.publish(imu_msg);

  // broadcast /earth transform relative to sea level
  tf::Quaternion vehicleQ;
  tf::quaternionMsgToTF(odomMsg_.pose.pose.orientation, vehicleQ);
  tf::Transform txEarth(vehicleQ, 
                       tf::Point(odomMsg_.pose.pose.position.x,
                                 odomMsg_.pose.pose.position.y,
                                 odomMsg_.pose.pose.position.z));
  tf_->sendTransform(tf::StampedTransform(txEarth, sim_time,
                                          tf_prefix_ + ArtFrames::earth,
                                          tf_prefix_ + ArtFrames::vehicle));

  // Also publish /odom frame with same elevation as /vehicle and same
  // orientation as /earth
  tf::Transform txOdom(tf::Quaternion(0.0, 0.0, 0.0, 1.0), 
                       tf::Point(0.0, 0.0, -odomMsg_.pose.pose.position.z));
  tf_->sendTransform(tf::StampedTransform(txOdom, sim_time,
                                          tf_prefix_ + ArtFrames::odom,
                                          tf_prefix_ + ArtFrames::earth));

  // Also publish the ground truth pose and velocity, correcting for
  // Stage's screwed-up coord system.
  Stg::Pose gpose = stgp_->GetGlobalPose();
  Stg::Velocity gvel = stgp_->GetGlobalVelocity();
  tf::Quaternion gposeQ;
  gposeQ.setRPY(0.0, 0.0, gpose.a-M_PI/2.0);
  tf::Transform gt(gposeQ, tf::Point(gpose.y, -gpose.x, 0.0));
  tf::Quaternion gvelQ;
  gvelQ.setRPY(0.0, 0.0, gvel.a-M_PI/2.0);
  tf::Transform gv(gvelQ, tf::Point(gvel.y, -gvel.x, 0.0));

  groundTruthMsg_.pose.pose.position.x     = gt.getOrigin().x();
  groundTruthMsg_.pose.pose.position.y     = gt.getOrigin().y();
  groundTruthMsg_.pose.pose.position.z     = gt.getOrigin().z();
  groundTruthMsg_.pose.pose.orientation.x  = gt.getRotation().x();
  groundTruthMsg_.pose.pose.orientation.y  = gt.getRotation().y();
  groundTruthMsg_.pose.pose.orientation.z  = gt.getRotation().z();
  groundTruthMsg_.pose.pose.orientation.w  = gt.getRotation().w();
  groundTruthMsg_.twist.twist.linear.x     = gv.getOrigin().x();
  groundTruthMsg_.twist.twist.linear.y     = gv.getOrigin().y();
  groundTruthMsg_.twist.twist.angular.z    = gvel.a;
  groundTruthMsg_.header.stamp = sim_time;
  groundTruthMsg_.header.frame_id = tf_prefix_ + ArtFrames::odom;
  groundTruthMsg_.child_frame_id = tf_prefix_ + ArtFrames::vehicle;
  ground_truth_pub_.publish(groundTruthMsg_);

  publishGPS(sim_time);

  last_update_time_ = sim_time;
}

void ArtVehicleModel::publishGPS(ros::Time sim_time)
{
  art_msgs::GpsInfo gpsi;

  gpsi.header.stamp = sim_time;
  gpsi.header.frame_id = tf_prefix_ + ArtFrames::odom;

  // relocate pose relative to map origin
  gpsi.utm_e = (odomMsg_.pose.pose.position.x
                - map_origin_x_ + origin_easting_);
  gpsi.utm_n = (odomMsg_.pose.pose.position.y
                - map_origin_y_ + origin_northing_);

  UTM::UTMtoLL(gpsi.utm_n, gpsi.utm_e, origin_zone_,
               gpsi.latitude, gpsi.longitude);

  gpsi.zone = origin_zone_;
  gpsi.altitude   = odomMsg_.pose.pose.position.z;
  gpsi.quality = art_msgs::GpsInfo::DGPS_FIX;
  gpsi.num_sats = 9;

  gps_pub_.publish(gpsi);
}
