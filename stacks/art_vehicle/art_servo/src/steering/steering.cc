/*
 *  Copyright (C) 2008, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

#define USE_VOLTAGE_POLYNOMIAL 1

#include <ros/ros.h>

#include <art_msgs/ArtHertz.h>
#include <art/polynomial.h>

#include <art_msgs/SteeringCommand.h>
#include <art_msgs/SteeringState.h>
#include <art_msgs/IOadrState.h>

#include "devsteer.h"			// servo device interface
#include "testwheel.h"			// steering wheel self-test

/**  \file

     @brief ROS driver for the ART steering servo controller.

This driver provides an interface to the steering servo for the ART
robot vehicle.

@see <art/steering.h> for steering angle conversions and constants.

Publishes

- @b steering/state [art_msgs::SteeringState]

Subscribes:

- @b ioadr/state [art_msgs::IOadrState]

  - if omitted, the steering position is assumed to start at 0.0
  (centered) and all requested steering positions are assumed to be
  reached accurately.

- @b steering/cmd [art_msgs::SteeringCommand]

Sets the desired steering angle (in degrees).  Position
max_steer_degrees is fully left, 0.0 is centered, -max_steer_degrees
is fully right.  There are both absolute and relative versions of this
command.

Parameters:

- @b ~/port (string)
  - serial port name for steering controller (usually "/dev/steering")
  - default: "/dev/null" (simulate steering mechanism)

- @b ~/diagnostic (bool)
  - if true, publish extra diagnostic information not needed for normal
    operation
  - default: false

- @b ~/calibration_period (integer)
  - number of cycles to spend calibrating starting wheel position
  - default: 19

@see <@ref devsteer.h> for steering servo device options

@author Jack O'Quin
*/

static float const epsilon = 0.01;	// "close enough" in degrees

#define CLASS "ArtSteer"

class ArtSteer
{
public:

  ArtSteer();
  ~ArtSteer();
  void	Main();
  int	Setup(ros::NodeHandle node);
  int	Shutdown();

private:

  // internal methods
  void	GetCmd(const art_msgs::SteeringCommand::ConstPtr &cmdIn);
  void	GetPos(const art_msgs::IOadrState::ConstPtr &ioIn);
  void	PublishStatus(void);
  void	calibrate_wheel_position(void);
  void	read_wheel_angle(void);

  // .cfg variables:
  bool	diagnostic_;			// enable diagnostic mode
  bool  simulate_;			// simulate sensor input
  int	calibration_periods_;		// number of sensor calibration cycles
  double sensor_timeout_;		// sensor timeout (sec)

  // ROS topic interfaces
  ros::Subscriber ioadr_state_;         // ioadr/state (position sensor)
  ros::Subscriber steering_cmd_;        // steering/cmd
  ros::Publisher  steering_state_;      // steering/state
  ros::Publisher  steering_diag_;	// steering/diag

  devsteer *dev_;			// servo device interface
  float	steering_angle_;                // current steering angle (degrees)
  float	steering_sensor_;		// current steering sensor reading
  double cur_sensor_time_;	        // current sensor data time (sec)
  double last_sensor_time_;	        // previous sensor data time (sec)
  float	set_point_;			// requested steering setting
  float	last_set_point_;                // previous requested steering setting

  // sensor calibration data
  int	calibration_cycle_;
  float	mean_voltage_;

  // wheel self-test class
  testwheel *tw_;

  // driver state variables -- the four valid states occur in this order:
  //
  //	angle_known_    wheel_calibrated_       wheel_tested_
  //	   false	      false		    false
  //	   true		      false		    false
  //	   true		      true		    false
  //	   true		      true		    true
  //
  bool	angle_known_;			// wheel angle known
  bool	wheel_calibrated_;		// wheel position calibrated
  bool	wheel_tested_;			// wheel motion tested

  // polynomials for converting between sensor voltage and angle
  Polynomial *apoly_;		        // angle to voltage conversion
#if defined(USE_VOLTAGE_POLYNOMIAL)
  Polynomial *vpoly_;		        // voltage to angle conversion
#else
  std::vector<double> c_;               // volts2degrees() coefficients
#endif

  // convert steering position sensor voltage to degrees
  float inline volts2degrees(float volts)
  {
#if defined(USE_VOLTAGE_POLYNOMIAL)
    return vpoly_->value(volts);        // use polynomial
#else
    // non-linear curve fit
    return c_[0] + c_[1]*volts + c_[2]*cos(c_[3]*volts+c_[4]);
#endif
  }

  // convert steering position sensor degrees to voltage
  float inline degrees2volts(float degrees)
  {
    return apoly_->value(degrees);
  }
};

ArtSteer::ArtSteer()
{
  // polynomial to convert steering angles to sensor voltages
  // (inverse of vpoly_, only used when simulating steering angles)
  apoly_ = new Polynomial("apoly");
  apoly_->add_coef(2.544);		// default constant coefficient
  apoly_->add_coef(-0.05456918);        // default coefficient of a
  apoly_->add_coef(0.00036568);		// default coefficient of a**2
  //apoly_->configure(cf, section);	// fill in .cfg values

#if defined(USE_VOLTAGE_POLYNOMIAL)
  // polynomial to convert sensor voltages to steering angles
  vpoly_ = new Polynomial("vpoly");
  vpoly_->add_coef(62.5943141);		// default constant coefficient
  vpoly_->add_coef(-30.0634102);        // default coefficient of v
  vpoly_->add_coef(2.14785486);		// default coefficient of v**2
  //vpoly_->configure(cf, section);	// fill in .cfg values
#else
  c_.clear();				// fill in volts2degrees()
  c_.push_back(52.25490152);		//  coefficients
  c_.push_back(-18.15450073);
  c_.push_back(6.2447116);
  c_.push_back(0.90281232);
  c_.push_back(0.65112384);
#endif

  // use private node handle to get parameters
  ros::NodeHandle mynh("~");
  
  diagnostic_ = false;
  mynh.getParam("diagnostic", diagnostic_);
  if (diagnostic_)
    ROS_INFO("using diagnostic mode");

  // simulate sensor input parameter
  mynh.param("simulate", simulate_, false);
  if (simulate_)
    ROS_INFO("simulating position sensor");

  calibration_periods_ = 19;
  mynh.getParam("calibration_periods", calibration_periods_);
  ROS_INFO("calibrate steering sensor for %d cycles.", calibration_periods_);

  /// @todo use this to detect when ioadr driver hung or not responding
  sensor_timeout_ = 2.0;
  mynh.getParam("sensor_timeout", sensor_timeout_);
  ROS_INFO("steering sensor timeout: %.3f seconds.", sensor_timeout_);

  // allocate and initialize the steering device interface
  dev_ = new devsteer();
  dev_->Configure();

  // allocate and configure the steering wheel self-test
  tw_ = new testwheel(dev_);
  tw_->Configure();
}

ArtSteer::~ArtSteer()
{
  delete tw_;
  delete dev_;
  delete apoly_;
#if defined(USE_VOLTAGE_POLYNOMIAL)
  delete vpoly_;
#endif
}

// Set up the device.  Return 0 if things go well, and -1 otherwise.
int ArtSteer::Setup(ros::NodeHandle node)
{
  // initialize state variables
  angle_known_ = false;
  wheel_calibrated_ = false;
  wheel_tested_ = false;

  last_sensor_time_ = cur_sensor_time_ = 0.0;
  last_set_point_ = set_point_ = 180.0;	// preposterous starting value
  steering_angle_ = 0.0;                // needed for simulation
  calibration_cycle_ = 0;

  int rc = dev_->Open();
  if (rc != 0)
    return -1;

  // subscribe to relevant ROS topics
  static int qDepth = 1;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  steering_cmd_ =
    node.subscribe("steering/cmd", qDepth, &ArtSteer::GetCmd, this, noDelay);
  steering_state_ =
    node.advertise<art_msgs::SteeringState>("steering/state", qDepth);
  steering_diag_ =
    node.advertise<art_msgs::SteeringDiagnostics>("steering/diag", qDepth);
  ioadr_state_ =
    node.subscribe("ioadr/state", qDepth, &ArtSteer::GetPos, this, noDelay);

  return 0;
}

// Shutdown the device
int ArtSteer::Shutdown()
{
  dev_->Close();
  return 0;
}


void ArtSteer::GetCmd(const art_msgs::SteeringCommand::ConstPtr &cmdIn)
{
  switch (cmdIn->request)
    {
    case art_msgs::SteeringCommand::Degrees:
      ROS_DEBUG(" %.3f degrees (absolute) steering request", cmdIn->angle);
      set_point_ = limit_travel(cmdIn->angle);
      break;
    case art_msgs::SteeringCommand::Relative:
      // Should this option be supported at all?  Initially it will
      // give bogus results.
      ROS_DEBUG(" %.3f degrees (relative) steering request", cmdIn->angle);
      set_point_ = limit_travel(set_point_ + cmdIn->angle);
      break;
    default:
      {
	ROS_WARN("invalid steering request %u (ignored)", cmdIn->request);
      }
    }
}

void ArtSteer::GetPos(const art_msgs::IOadrState::ConstPtr &ioIn)
{
  if (simulate_)			// not using real sensor?
    return;

  // save steering position sensor voltage, convert to degrees
  steering_sensor_ = ioIn->voltages[0];
  steering_angle_ = limit_travel(volts2degrees(steering_sensor_));
  cur_sensor_time_ = ioIn->header.stamp.toSec();
  angle_known_ = true;
}

// Calibrate current steering wheel position.
//
// Assumes the wheel will not move while in this state.  Unless
// someone forgot to turn on the steering breaker, that will be
// true. Otherwise, the wheel self-test should detect the error.
//
// This implementation minimizes the effects of sensor noise by
// accumulating a moving average of the first "calibration_periods_"
// readings.
//
void ArtSteer::calibrate_wheel_position(void)
{
  if (cur_sensor_time_ > last_sensor_time_) // new sensor data this cycle?
    {
      if (calibration_cycle_++ == 0)	// first time?
	  mean_voltage_ = 0.0;

      // accumulate moving average
      mean_voltage_ += ((steering_sensor_ - mean_voltage_)
		       / calibration_cycle_);
      ROS_DEBUG("cumulative steering sensor average: %.3f volts", mean_voltage_);

      if (calibration_cycle_ >= calibration_periods_)
	{
	  // finished with calibration
	  wheel_calibrated_ = true;
	  steering_angle_ = limit_travel(volts2degrees(mean_voltage_));
	  ROS_INFO("initial steering angle: %.2f degrees", steering_angle_);
	  dev_->set_initial_angle(steering_angle_);
	}
    }
}

// Get simulated wheel angle from driver (if real sensor not available).
void ArtSteer::read_wheel_angle(void)
{
  int rc = dev_->get_angle(steering_angle_);
  if (rc != 0)
    {
      return;
    }
  steering_sensor_ = degrees2volts(steering_angle_);
  cur_sensor_time_ = ros::Time::now().toSec();
  angle_known_ = true;
}

// publish current device status
void ArtSteer::PublishStatus(void)
{
  art_msgs::SteeringState msg;         // steering state message

  // report the angle and voltage from the steering position sensor
  msg.angle = steering_angle_;
  msg.sensor = steering_sensor_;

  // TODO: use ROS diagnostics package
  if (diagnostic_)			// return extra diagnostic values?
    {
      // attempt to read current encoder position
      dev_->get_encoder(msg.encoder);
      dev_->publish_diag(steering_diag_);
    }

  msg.header.stamp = ros::Time::now();
  steering_state_.publish(msg);
}

// Main function for device thread
void ArtSteer::Main() 
{
  ros::Rate cycle(art_msgs::ArtHertz::STEERING); // set driver cycle rate

  while(ros::ok())
    {
      ros::spinOnce();                  // handle incoming commands

      if (simulate_)			// not using real position sensor
        read_wheel_angle();

      // Processing depends on driver state.  These tests reverse the
      // actual sequence of states, focusing on normal operation.
      if (wheel_tested_)			// wheel is working OK?
	{
	  if (fabs(set_point_ - last_set_point_) > epsilon)
	    {
	      // command steering position to match desired set point.
	      int rc = dev_->steering_absolute(set_point_);
	      if (rc == 0)
		last_set_point_ = set_point_;
	    }
	}
      else if (wheel_calibrated_)	// initial position known?
	{
	  int rc = tw_->Run(steering_angle_);
	  if (rc < 0)
	    {
	      ROS_FATAL("steering self-test failure: shutting down driver");
              ros::shutdown();
	    }
	  if (rc != 0)
	    wheel_tested_ = true;
	}
      else if (angle_known_)	// sensor data received?
	{
	  calibrate_wheel_position();
	}

      if (angle_known_)
	PublishStatus();		// publish current status

      last_sensor_time_ = cur_sensor_time_;

      cycle.sleep();                    // sleep until next cycle
    }
}
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "steering");
  ros::NodeHandle node;
  ArtSteer dvr;

  if (dvr.Setup(node) != 0)
    return 2;
  dvr.Main();
  dvr.Shutdown();

  return 0;
}
