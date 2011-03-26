/*
 *  Copyright (C) 2007, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

#include <math.h>

#include <ros/ros.h>

#include <art_msgs/ArtHertz.h>
#include <art_msgs/ThrottleCommand.h>
#include <art_msgs/ThrottleState.h>

#include "devthrottle.h"		// servo device interface

#define IOADR_MAX_INPUTS  8

/**  \file

     @brief ROS driver for the ART throttle servo controller.

This driver provides an interface to the throttle servo for the ART robot
vehicle.

Publishes

- \b throttle/state [art_msgs::ThrottleState] throttle status.

Subscribes

- \b throttle/cmd [art_msgs::ThrottleCommand] throttle commands.

Sets the desired throttle position.  At position 1.0 the throttle is
fully open, fully closed at 0.0 (idle).  There are both absolute and
relative requests.

Parameters

- port (string)
  - tty port name for the throttle servo
  - use "/dev/null" when simulating the device
  - default: "/dev/throttle" (actual hardware port)

- training (bool)
  - if true, log throttle data, but do not send it any commands
  - default: false

Training mode collects data while a human driver operates the vehicle.

- diagnostic (bool)
  - if true, log extra diagnostic throttle information not needed for
    normal operation
  - default: false

@author Jack O'Quin
*/


#define CLASS "Throttle"

class Throttle
{
public:

  Throttle();
  ~Throttle()
  {
    delete dev_;
  }
  void	Main();
  int	Setup(ros::NodeHandle node);
  int	Shutdown();

private:

  void GetCmd(const art_msgs::ThrottleCommand::ConstPtr &cmd);
  void PollDevice(void);

  // configuration parameters
  std::string port_;                    // tty port name
  bool	training_;			// use training mode
  bool	diagnostic_;			// enable diagnostic mode

  ros::Subscriber throttle_cmd_;        // throttle/cmd
  ros::Publisher  throttle_state_;      // throttle/state

  devthrottle *dev_;			// servo device interface
};

// constructor, use pull mode with replace
Throttle::Throttle()
{
  // use private node handle to get parameters
  ros::NodeHandle mynh("~");

  port_ = "/dev/throttle";
  mynh.getParam("port", port_);
  ROS_INFO_STREAM("steering port = " << port_);
  
  diagnostic_ = false;
  mynh.getParam("diagnostic", diagnostic_);
  if (diagnostic_)
    ROS_INFO("using diagnostic mode");

  training_ = false;
  mynh.getParam("training", training_);
  if (training_)
    ROS_INFO("using training mode");

  // allocate and initialize the devthrottle interface
  dev_ = new devthrottle(training_);
}

// Set up the device.  Return 0 if things go well, and -1 otherwise.
int Throttle::Setup(ros::NodeHandle node)
{   
  int rc = dev_->Open(port_.c_str());
  if (rc != 0)
    return -1;

  if (training_ == false)
    {
      rc = dev_->throttle_absolute(0.0); // set idle throttle
      if (rc != 0)
	{
          dev_->Close();
	  return -1;
	}
    }

  // subscribe to relevant ROS topics
  static int qDepth = 1;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  throttle_cmd_ =
    node.subscribe("throttle/cmd", qDepth, &Throttle::GetCmd, this, noDelay);
  throttle_state_ =
    node.advertise<art_msgs::ThrottleState>("throttle/state", qDepth);

  return 0;
}

// Shutdown the device
int Throttle::Shutdown()
{
  if (training_ == false)
    dev_->throttle_absolute(0.0);	// set idle throttle
  dev_->Close();
  return 0;
}

void Throttle::GetCmd(const art_msgs::ThrottleCommand::ConstPtr &cmd)
{
  // ignore all throttle command messages when in training mode
  if (training_)
    return;

  switch (cmd->request)
    {
    case art_msgs::ThrottleCommand::Absolute:
      dev_->throttle_absolute(cmd->position);
      break;
    case art_msgs::ThrottleCommand::Relative:
      dev_->throttle_relative(cmd->position);
      break;
    default:
      {
	ROS_WARN("invalid throttle request %u (ignored)", cmd->request);
      }
    }
}

// poll device for current status
//
// if an I/O fails, the corresponding voltages[i] remains unchanged
//
void Throttle::PollDevice(void)
{
  int rc = dev_->query_status();        // get controller status
  if (rc == 0)				// any news?
    {
      art_msgs::ThrottleState msg;

      msg.position = dev_->get_position();
      dev_->query_rpms(&msg.rpms);
      msg.estop = dev_->query_estop();

      if (diagnostic_)             // publish extra diagnostic data?
	{
	  dev_->query_pid(&msg.pwm, &msg.dstate, &msg.istate);
	}

      msg.header.stamp = ros::Time::now();
      throttle_state_.publish(msg);
    }
}

// Main function for device thread
void Throttle::Main() 
{
  ros::Rate cycle(art_msgs::ArtHertz::THROTTLE); // set driver cycle rate

  while(ros::ok())
    {
      PollDevice();
      ros::spinOnce();                  // handle incoming commands
      cycle.sleep();                    // sleep until next cycle
    }
}
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "throttle");
  ros::NodeHandle node;
  Throttle dvr;

  if (dvr.Setup(node) != 0)
    return 2;
  dvr.Main();
  dvr.Shutdown();

  return 0;
}
