/*
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file

     ROS driver for the ART brake servo controller.

     \author Jack O'Quin
 */

#include <assert.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "ros/ros.h"

#include <art/pid2.h>			// PID control 

#include <art_msgs/BrakeCommand.h>
#include <art_msgs/BrakeState.h>

#include <art_msgs/ArtHertz.h>


#include "devbrake.h"			// servo device interface

/** 
 @brief ART brake servo driver

This driver provides a brake node interface to the brake servo for the
ART robot vehicle.

Publishes

- brake/state topic

Commands

- brake/cmd topic

Sets the desired brake position.  Position 1.0 is fully on, 0.0 is
fully off.  There are both absolute and relative versions of this
command.

Parameters

- port (string)
  - tty port name for the brake servo
  - use "/dev/null" when simulating the device
  - default: "/dev/brake" (actual hardware port)

- apply_on_exit (bool)
  - unless false, apply full brake during shutdown.
  - default: false

- training (bool)
  - if true, log brake data, but do not send it any commands
  - default: false

Training mode collects data while a human driver operates the vehicle.

- diagnostic (bool)
  - if true, log extra diagnostic brake information not needed for
    normal operation
  - default: false

@todo describe initial calibration values
@todo use ROS diagnostic_updater package

@author Jack O'Quin
*/


#define NODE "brake"

namespace
{
  // .cfg variables:
  std::string port = "/dev/brake";      // tty port name
  bool	training = false;               // use training mode
  bool	diagnostic = false;             // enable diagnostic mode
  int   qDepth = 1;                     // ROS topic queue depths
  /// @todo make queue depth an option

  Pid	*pid;				// PID control
  devbrake *dev;                        // servo device interface
  float	brake_pos;                      // current brake position
  float	set_point;			// requested brake setting
}

// ROS topics used by this driver
ros::Subscriber brake_cmd;              // /brake/cmd ROS topic
ros::Publisher brake_state;             // /brake/state ROS topic

int GetParameters(void)
{
  // use private node handle to get parameters
  ros::NodeHandle mynh("~");

  mynh.getParam("port", port);
  ROS_INFO("brake port = %s", port.c_str());

  mynh.getParam("training", training);
  if (training)
    ROS_INFO("using training mode");

  mynh.getParam("diagnostic", diagnostic);
  if (diagnostic)
    ROS_INFO("using diagnostic mode");

  // allocate and initialize the devbrake interface
  dev = new devbrake(training);

  // Set brake parameters -- make sure the defaults won't strip the
  // servo hardware gears.  These values will be used for /dev/null,
  // in training mode, or in case of failure.  They may be updated by
  // devbrake when it detects calibration changes.

  // TODO: move these parameters into a subordinate class.

  if (!mynh.getParam("encoder_min", dev->encoder_min))
    dev->encoder_min = 0.0;
  if (!mynh.getParam("encoder_max", dev->encoder_max))
    dev->encoder_max = 50000.0;
  dev->encoder_range = dev->encoder_max - dev->encoder_min;
  ROS_INFO("configured encoder range [%.f, %.f]",
           dev->encoder_min, dev->encoder_max);

  if (!mynh.getParam("pot_off", dev->pot_off))
    dev->pot_off = 4.9;
  if (!mynh.getParam("pot_full", dev->pot_full))
    dev->pot_full = 0.49;
  dev->pot_range = dev->pot_full - dev->pot_off;
  ROS_INFO("configured potentiometer range [%.3f, %.3f]",
           dev->pot_off, dev->pot_full);

  if (!mynh.getParam("pressure_min", dev->pressure_min))
    dev->pressure_min = 0.85;
  if (!mynh.getParam("pressure_max", dev->pressure_max))
    dev->pressure_max = 4.5;
  dev->pressure_range = dev->pressure_max - dev->pressure_min;
  ROS_INFO("configured pressure range [%.3f, %.3f]",
           dev->pressure_min, dev->pressure_max);

  // allocate PID control and configure parameters
  pid = new Pid("pid", 0.25, 0.0, 0.7);
  pid->Configure(mynh);

  return 0;
}

// Set up the device.  Return 0 if things go well, and -1 otherwise.
int Setup()
{   
  int rc = dev->Open(port.c_str());
  if (rc != 0)
    {
      ROS_FATAL("device open failed: %d", rc);
      return -1;
    }

  ROS_INFO("device opened");

  // wherever dev->Open() left the brake becomes our initial set point
  set_point = brake_pos = dev->get_position();

  return 0;
}

// Shutdown the device
int Shutdown()
{
  dev->Close();
  ROS_INFO("device closed");
  return 0;
}

void ProcessCommand(const art_msgs::BrakeCommand::ConstPtr &cmd)
{
  uint32_t request = cmd->request;

  // ignore all brake command messages when in training mode
  if (training)
    {
      ROS_DEBUG("in training mode: brake cmd %u ignored", request);
      return;
    }

  ROS_DEBUG("brake request %u, position %.3f", request, cmd->position);

  switch (request)
    {
    case art_msgs::BrakeCommand::Absolute:
      set_point = limit_travel(cmd->position);
      break;
    case art_msgs::BrakeCommand::Relative:
      set_point = limit_travel(brake_pos + cmd->position);
      break;
    default:
      {
	ROS_WARN("invalid brake request %u (ignored)", request);
	return;
      }
    }
}

// Poll device for current status.  Publish results as brake status.
//
// If an I/O fails, the corresponding values remain unchanged and old
// data are published.  This is intentional.
//
float PollDevice(void)
{
  art_msgs::BrakeState bs;             // brake state message

  // read the primary hardware sensor status
  dev->get_state(&bs.position, &bs.potentiometer, &bs.encoder, &bs.pressure);

#if 0 // TODO: use ROS diagnostics package
  if (diagnostic)			// return extra diagnostic values?
    {
      dev->query_amps(&aio_data.voltages[BrakeAmps]);
      dev->query_volts(&aio_data.voltages[BrakeVolts]);
      aio_data.voltages_count = BrakeDataMax;
    }
#endif

  bs.header.stamp = ros::Time::now();
  brake_state.publish(bs);

  // return current position
  return bs.position;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  // topics to read and write
  brake_cmd = node.subscribe(NODE "/cmd", qDepth, ProcessCommand,
                             ros::TransportHints().tcpNoDelay(true));
  brake_state = node.advertise<art_msgs::BrakeState>(NODE "/state", qDepth);

  if (GetParameters() != 0)
    return 1;

  ros::Rate cycle(art_msgs::ArtHertz::BRAKE); // set driver cycle rate

  if (Setup() != 0)
    return 2;

  // Main loop; grab messages off our queue and republish them via ROS
  while(ros::ok())
    {
      brake_pos = PollDevice();         // get and publish device status

      ros::spinOnce();                  // handle incoming commands

      static float const epsilon = 0.001;
      float ctlout = pid->Update(set_point - brake_pos, brake_pos);
      if (fabs(ctlout) > epsilon)       // not quite close enough?
        dev->brake_relative(ctlout);

      cycle.sleep();                    // sleep until next cycle
    }

  Shutdown();

  return 0;
}
