/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART autonomous vehicle device interface implementations for pilot node.

     This is an implementation header.  It is only included by
     pilot.cc, to instantiate the derived classes.  Any other modules
     using these interfaces should include device_interface.h,
     instead.

     @note There is a lot of repeated code here, mostly due to the
     diversity of message types.  Templates could reduce some of the
     repetition, but at the cost of extra complexity. That additional
     indirection would make the code harder to read and understand.

     @author Jack O'Quin

 */

#ifndef __DEVICE_IMPL_H_
#define __DEVICE_IMPL_H_

#include <ros/ros.h>

// device messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <art_msgs/BrakeCommand.h>
#include <art_msgs/BrakeState.h>
#include <art_msgs/Shifter.h>
#include <art_msgs/SteeringCommand.h>
#include <art_msgs/SteeringState.h>
#include <art_msgs/ThrottleCommand.h>
#include <art_msgs/ThrottleState.h>

#include "device_interface.h"

namespace device_interface
{

/** Inertial Management Unit interface from Applanix node */
class DeviceImu: public DeviceBase
{
 public:

  DeviceImu(ros::NodeHandle node):
    DeviceBase(node)
  {
    sub_ = node.subscribe("imu", 1, &DeviceImu::process, this,
                          ros::TransportHints().tcpNoDelay(true));
  }

  virtual DeviceState state(ros::Time recently)
  {
    if (msg_.header.stamp > recently)
      return art_msgs::DriverState::RUNNING;
    else
      return art_msgs::DriverState::CLOSED;
  }

  float value()
  {
    return msg_.linear_acceleration.x;  // current acceleration
  }

private:

  void process(const sensor_msgs::Imu::ConstPtr &msgIn)
  {
    msg_ = *msgIn;
  }

  sensor_msgs::Imu msg_;                // last message received
};

/** Odometry interface from Applanix node */
class DeviceOdom: public DeviceBase
{
 public:

  DeviceOdom(ros::NodeHandle node):
    DeviceBase(node)
  {
    sub_ = node.subscribe("odom", 1, &DeviceOdom::process, this,
                          ros::TransportHints().tcpNoDelay(true));
  }

  virtual DeviceState state(ros::Time recently)
  {
    if (msg_.header.stamp > recently)
      return art_msgs::DriverState::RUNNING;
    else
      return art_msgs::DriverState::CLOSED;
  }

  float value()
  {
    return msg_.twist.twist.linear.x;   // current velocity
  }

private:

  void process(const nav_msgs::Odometry::ConstPtr &msgIn)
  {
    msg_ = *msgIn;
  }

  nav_msgs::Odometry msg_;              // last message received
};


/** Brake servo interface class */
class DeviceBrake: public ServoDeviceBase
{
 public:

  DeviceBrake(ros::NodeHandle node):
    ServoDeviceBase(node)
  {
    sub_ = node.subscribe("brake/state", 1,
                          &DeviceBrake::process, this,
                          ros::TransportHints().tcpNoDelay(true));
    pub_ = node.advertise<art_msgs::BrakeCommand>("brake/cmd", 1);
    cmd_.request = art_msgs::BrakeCommand::Absolute;
    cmd_.position = 1.0;
  }

  virtual float last_request()
  {
    return cmd_.position;               // last position requested
  }

  virtual void publish(float new_position, ros::Time cycle_time)
  {
    cmd_.header.stamp = cycle_time;
    cmd_.position = new_position;
    pub_.publish(cmd_);
  }

  virtual DeviceState state(ros::Time recently)
  {
    if (msg_.header.stamp > recently)
      return art_msgs::DriverState::RUNNING;
    else
      return art_msgs::DriverState::CLOSED;
  }

  virtual float value()
  {
    return msg_.position;               // current brake position
  }

private:

  void process(const art_msgs::BrakeState::ConstPtr &msgIn)
  {
    msg_ = *msgIn;
  }

  art_msgs::BrakeCommand cmd_;          // last command sent
  art_msgs::BrakeState msg_;            // last message received
};

/** Shifter interface class
 *
 *  Since this device uses gear numbers and not floats, it cannot
 *  implement the full ServoDeviceBase interface, so it uses
 *  DeviceBase, with some similar additional members.  The publish()
 *  method takes a gear number, and the value() method returns one.
 */
class DeviceShifter: public DeviceBase
{
 public:

  typedef art_msgs::Shifter::_gear_type Gear;

  DeviceShifter(ros::NodeHandle node):
    DeviceBase(node),
    shift_duration_(1.0)
  {
    sub_ = node.subscribe("shifter/state", 1,
                          &DeviceShifter::process, this,
                          ros::TransportHints().tcpNoDelay(true));
    pub_ = node.advertise<art_msgs::Shifter>("shifter/cmd", 1);
    msg_.gear = art_msgs::Shifter::Drive;
  }

  /** return true if shifter relay is busy */
  bool busy(void)
  {
    return ((ros::Time::now() - shift_time_) < shift_duration_);
  }

  /** return true if shifting is done (all relays off) */
  bool is_reset(void)
  {
    return (msg_.relays == 0);
  }

  void publish(Gear new_position, ros::Time cycle_time)
  {
    shift_time_ = cycle_time;
    cmd_.header.stamp = cycle_time;
    cmd_.gear = new_position;
    pub_.publish(cmd_);
  }

  virtual DeviceState state(ros::Time recently)
  {
    if (msg_.header.stamp > recently)
      return art_msgs::DriverState::RUNNING;
    else
      return art_msgs::DriverState::CLOSED;
  }

  Gear value()
  {
    return msg_.gear;                   // current gear number
  }

private:

  void process(const art_msgs::Shifter::ConstPtr &msgIn)
  {
    msg_ = *msgIn;
  }

  art_msgs::Shifter cmd_;               // last command sent
  art_msgs::Shifter msg_;               // last state message received
  ros::Publisher pub_;                  // command message publisher
  ros::Time shift_time_;                // time last shift requested
  ros::Duration shift_duration_;        // duration to hold relay
};

/** Steering servo interface class */
class DeviceSteering: public ServoDeviceBase
{
 public:

  DeviceSteering(ros::NodeHandle node):
    ServoDeviceBase(node)
  {
    sub_ = node.subscribe("steering/state", 1,
                          &DeviceSteering::process, this,
                          ros::TransportHints().tcpNoDelay(true));
    pub_ = node.advertise<art_msgs::SteeringCommand>("steering/cmd", 1);
    cmd_.request = art_msgs::SteeringCommand::Degrees;
    cmd_.angle = 0.0;
  }

  virtual float last_request()
  {
    return cmd_.angle;                  // last angle requested
  }

  virtual void publish(float new_position, ros::Time cycle_time)
  {
    cmd_.header.stamp = cycle_time;
    cmd_.angle = new_position;
    pub_.publish(cmd_);
  }

  virtual DeviceState state(ros::Time recently)
  {
    if (msg_.header.stamp > recently)
      return msg_.driver.state;
    else
      return art_msgs::DriverState::CLOSED;
  }

  virtual float value()
  {
    return msg_.angle;                  // current steering angle
  }

private:

  void process(const art_msgs::SteeringState::ConstPtr &msgIn)
  {
    msg_ = *msgIn;
  }

  art_msgs::SteeringCommand cmd_;          // last command sent
  art_msgs::SteeringState msg_;            // last message received
};

/** Throttle servo interface class */
class DeviceThrottle: public ServoDeviceBase
{
 public:

  DeviceThrottle(ros::NodeHandle node):
    ServoDeviceBase(node)
  {
    sub_ = node.subscribe("throttle/state", 1,
                          &DeviceThrottle::process, this,
                          ros::TransportHints().tcpNoDelay(true));
    pub_ = node.advertise<art_msgs::ThrottleCommand>("throttle/cmd", 1);
    cmd_.request = art_msgs::ThrottleCommand::Absolute;
    cmd_.position = 0.0;
  }

  virtual float last_request()
  {
    return cmd_.position;               // last position requested
  }

  virtual void publish(float new_position, ros::Time cycle_time)
  {
    cmd_.header.stamp = cycle_time;
    cmd_.position = new_position;
    pub_.publish(cmd_);
  }

  virtual DeviceState state(ros::Time recently)
  {
    if (msg_.header.stamp > recently)
      return art_msgs::DriverState::RUNNING;
    else
      return art_msgs::DriverState::CLOSED;
  }

  virtual float value()
  {
    return msg_.position;               // current throttle position
  }

private:

  void process(const art_msgs::ThrottleState::ConstPtr &msgIn)
  {
    msg_ = *msgIn;
  }

  art_msgs::ThrottleCommand cmd_;       // last command sent
  art_msgs::ThrottleState msg_;         // last message received
};

}; // end device_interface namespace

#endif // __DEVICE_IMPL_H_
