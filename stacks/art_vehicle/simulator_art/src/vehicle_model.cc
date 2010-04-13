/*
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file
 
     Model speed and turn rate of the ART autonomous vehicle.

     \todo figure out GPS publication
 
     \author Jack O'Quin

 */
#undef GPS                              // not implemented

#include <angles/angles.h>

#include <art/conversions.h>

#include <art_servo/BrakeState.h>
#include <art_servo/Shifter.h>
#include <art_servo/SteeringState.h>
#include <art_servo/ThrottleState.h>

#include <art_servo/steering.h>
#include <art/frames.h>
#include <art/hertz.h>
#include <art/vehicle.hh>
#include <art/epsilon.h>

#ifdef GPS
#include <art/applanix_info.h>
#include <art/UTM.h>
#endif

#include "vehicle_model.h"

void ArtVehicleModel::setup(void)
{
  int qDepth = 1;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  odom_pub_ =
    node_.advertise<nav_msgs::Odometry>(ns_prefix_ + "odom", qDepth);
  ground_truth_pub_ =
    node_.advertise<nav_msgs::Odometry>(ns_prefix_ + "ground_truth", qDepth);
  
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
}

// Servo device interfaces.
//
// IMPORTANT: These callbacks run in a separate thread, so all class
// data updates must be done while holding the msg_lock_.
//
void
ArtVehicleModel::brakeReceived(const art_servo::BrakeState::ConstPtr &msg)
{
  //ROS_DEBUG("brake state received: position %.3f", msg->position);
  boost::mutex::scoped_lock lock(msg_lock_);
  brake_position_ = msg->position;
}

void
ArtVehicleModel::shifterReceived(const art_servo::Shifter::ConstPtr &msg)
{
  ROS_DEBUG("shifter state received: gear %u", msg->gear);
  boost::mutex::scoped_lock lock(msg_lock_);
  shifter_gear_ = msg->gear;
}

void 
ArtVehicleModel::steeringReceived(const art_servo::SteeringState::ConstPtr &msg)
{
  //ROS_DEBUG("steering state received: %.1f (degrees)", msg->angle);
  boost::mutex::scoped_lock lock(msg_lock_);
  steering_angle_ = msg->angle;
}

void
ArtVehicleModel::throttleReceived(const art_servo::ThrottleState::ConstPtr &msg)
{
  //ROS_DEBUG("throttle state received: position %.3f", msg->position);
  boost::mutex::scoped_lock lock(msg_lock_);
  throttle_position_ = msg->position;
}

/** Model vehicle acceleration
 *
 *  On entry: last_update_time_ = time of previous update.
 *  Updates: odomVel, the Odometry message Twist component
 *
 *  Note: This simple model does not account for sideways slippage, so
 *        odomVel->linear.y is always zero.  Similarly, roll, pitch
 *        and odomVel->linear.z are always zero, because Stage is a 2D
 *        simulation.
 *
 *  TODO: introduce some small random fluctuations
 */
void ArtVehicleModel::ModelAcceleration(geometry_msgs::Twist *odomVel,
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

  // Brake and throttle (by themselves) never cause reverse motion.
  // Only shifting into Reverse can do that.
  if (speed < 0.0)
    speed = 0.0;

  // Set velocity sign based on gear.
  odomVel->linear.x = speed;            // forward movement
  if (shifter_gear_ ==  art_servo::Shifter::Reverse)
    odomVel->linear.x = -speed;         // reverse movement

  // set yaw rate (radians/second) from velocity and steering angle
  odomVel->angular.z = Steering::angle_to_yaw(odomVel->linear.x,
                                              steering_angle_);

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
  // model vehicle acceleration from servo actuators
  ModelAcceleration(&odomMsg_.twist.twist, sim_time);

  // Get latest position data from Stage
  // Translate into ROS message format and publish
  odomMsg_.pose.pose.position.x = stgp_->est_pose.x;
  odomMsg_.pose.pose.position.y = stgp_->est_pose.y;
  odomMsg_.pose.pose.orientation =
    tf::createQuaternionMsgFromYaw(stgp_->est_pose.a);

  odomMsg_.header.stamp = sim_time;
  odomMsg_.header.frame_id = tf_prefix_ + ArtFrames::odom;
  odomMsg_.child_frame_id = tf_prefix_ + ArtFrames::vehicle;
  odom_pub_.publish(odomMsg_);

  // broadcast odometry transform
  tf::Quaternion odomQ;
  tf::quaternionMsgToTF(odomMsg_.pose.pose.orientation, odomQ);
  tf::Transform txOdom(odomQ, 
                       tf::Point(odomMsg_.pose.pose.position.x,
                                 odomMsg_.pose.pose.position.y, 0.0));
  tf_->sendTransform(tf::StampedTransform(txOdom, sim_time,
                                          tf_prefix_ + ArtFrames::odom,
                                          tf_prefix_ + ArtFrames::vehicle));

  // Also publish the ground truth pose and velocity, correcting for
  // Stage's screwed-up coord system.
  Stg::stg_pose_t gpose = stgp_->GetGlobalPose();
  Stg::stg_velocity_t gvel = stgp_->GetGlobalVelocity();
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

#ifdef GPS
  updateGPS();
    
  Publish(gps_addr,
	  PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_DATA_STATE,
	  (void*)&opaque, sizeof(opaque.data_count)+opaque.data_count);
#endif

  last_update_time_ = sim_time;
}

#if GPS
void ArtVehicleModel::updateGPS()
{
  double UTMe = 0;
  double UTMn = 0;
  double car_lat = 0;
  double car_long = 0;
  char zone[255];
  LLtoUTM(origin_lat, origin_long, UTMn, UTMe, zone);
  UTMe += cur_pos.pos.px;
  UTMn += cur_pos.pos.py;
  //std::cerr << zone << std::endl;
  UTMtoLL(UTMn, UTMe, zone, car_lat, car_long);
  //std::cerr << "car_x: " << cur_pos.pos.px << std::endl;
  //std::cerr << "car_y: " << cur_pos.pos.py << std::endl;
  odom_gps.gps_latitude=car_lat;
  odom_gps.gps_longitude=car_long;
  /*
  odom_gps.gps_latitude=origin_lat;
  odom_gps.gps_longitude=origin_long;
  */
}
#endif
