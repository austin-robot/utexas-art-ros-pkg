/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file
 
     ART autonomous vehicle device driver abstract base interfaces.
 
        DeviceBase -- device interface abstract base class
        ServoDeviceBase -- abstract derived class for servo devices

     \author Jack O'Quin

 */

#ifndef __DEVICE_INTERFACE_H_
#define __DEVICE_INTERFACE_H_

#include <ros/ros.h>
#include <art_msgs/DriverState.h>

namespace device_interface
{

/** Device virtual base class */
class DeviceBase
{
 public:

  /** Constructor

      @param node handle for state topic
  */
  DeviceBase(ros::NodeHandle node):
    node_(node)
  {}

  typedef art_msgs::DriverState::_state_type DeviceState;
  virtual DeviceState state(ros::Time recently) = 0;

protected:
  ros::NodeHandle node_;                // node handle for topic
  ros::Subscriber sub_;                 // state message subscriber
};

/** Servo device virtual derived class */
class ServoDeviceBase: public DeviceBase
{
 public:

  /** Constructor

      @param node handle for state and command topics
  */
  ServoDeviceBase(ros::NodeHandle node):
    DeviceBase(node)
  {}

  virtual float last_request() = 0;
  virtual float value() = 0;

  /** Publish servo request.

      @param new_position new position requested
      @param cycle_time current pilot cycle time stamp
  */
  virtual void publish(float new_position, ros::Time cycle_time) = 0;

protected:
  ros::Publisher pub_;                  // command message publisher
};

}; // end device_interface namespace

#endif // __DEVICE_INTERFACE_H_
