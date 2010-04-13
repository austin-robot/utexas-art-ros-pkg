/* -*- mode: C++ -*-
 *
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

#ifndef _VEHICLE_MODEL_H_
#define _VEHICLE_MODEL_H_ 1

#include <string>
#include <boost/thread/mutex.hpp>

// libstage
#include <stage.hh>

// ROS interfaces
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <art_servo/BrakeState.h>
#include <art_servo/Shifter.h>
#include <art_servo/SteeringState.h>
#include <art_servo/ThrottleState.h>

// Corresponding ROS relative names
#define BRAKE_STATE    "brake/state"
#define SHIFTER_STATE  "shifter/state"
#define STEERING_STATE "steering/state"
#define THROTTLE_STATE "throttle/state"

class ArtVehicleModel
{
public:
    
  // Constructor
  ArtVehicleModel(Stg::StgModelPosition *stgPos,
                  tf::TransformBroadcaster *tfBroad,
                  std::string ns_prefix)
  {
    stgp_ = stgPos;                     // Stage position model
    tf_ = tfBroad;                      // ROS transform broadcaster
    ns_prefix_ = ns_prefix;             // namespace prefix
    tf_prefix_ = ns_prefix + "/";       // transform ID prefix

    // servo control status
    brake_position_ = 1.0;
    shifter_gear_ = art_servo::Shifter::Drive;
    steering_angle_ = 0.0;
    throttle_position_ = 0.0;
  }
  ~ArtVehicleModel() {};

  void update(ros::Time sim_time);      // update vehicle model
  void setup(void);                     // set up ROS topics
	
private:

  void ModelAcceleration(geometry_msgs::Twist *odomVel, ros::Time sim_time);

  // Stage interfaces
  Stg::StgModelPosition *stgp_;

  // ROS interfaces
  ros::NodeHandle node_;                // simulation node handle
  tf::TransformBroadcaster *tf_;        // ROS transform broadcaster
  std::string ns_prefix_;               // vehicle namespace
  std::string tf_prefix_;               // transform ID prefix

  nav_msgs::Odometry odomMsg_;
  ros::Publisher odom_pub_;
  nav_msgs::Odometry groundTruthMsg_;
  ros::Publisher ground_truth_pub_;
  ros::Time last_update_time_;

  // servo device interfaces
  ros::Subscriber brake_sub_;
  ros::Subscriber shifter_sub_;
  ros::Subscriber steering_sub_;
  ros::Subscriber throttle_sub_;

  // servo message callbacks
  void brakeReceived(const art_servo::BrakeState::ConstPtr &msg);
  void shifterReceived(const art_servo::Shifter::ConstPtr &msg);
  void steeringReceived(const art_servo::SteeringState::ConstPtr &msg);
  void throttleReceived(const art_servo::ThrottleState::ConstPtr &msg);

  // servo control status
  //
  // The mutex serializes access to these fields, which are set by
  // message callbacks running in a separate thread.
  boost::mutex msg_lock_;
  float brake_position_;
  uint8_t shifter_gear_;
  float steering_angle_;
  float throttle_position_;

  void publishGPS(ros::Time sim_time);

  double origin_lat_;
  double origin_long_;
  double origin_easting_;
  double origin_northing_;
  char   origin_zone_[20];
  double map_origin_x_;
  double map_origin_y_;

  ros::Publisher gps_pub_;
};

#endif // _VEHICLE_MODEL_H_
