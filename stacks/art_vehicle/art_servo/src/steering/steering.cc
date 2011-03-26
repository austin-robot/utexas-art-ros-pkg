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
  - use "/dev/null" when simulating the device
  - default: "/dev/steering" (actual hardware port)

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
  void	run();

private:

  // internal methods
  int	calibrate_wheel_position(void);
  void	close();
  void	GetCmd(const art_msgs::SteeringCommand::ConstPtr &cmdIn);
  void	GetPos(const art_msgs::IOadrState::ConstPtr &ioIn);
  int	open();
  void	PublishStatus(void);
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

  float	steering_angle_;                // current steering angle (degrees)
  float	steering_sensor_;		// current steering sensor reading
  double cur_sensor_time_;	        // current sensor data time (sec)
  double last_sensor_time_;	        // previous sensor data time (sec)
  float	set_point_;			// requested steering setting
  float	last_set_point_;                // previous requested steering setting

  // sensor calibration data
  int	calibration_cycle_;
  float	mean_voltage_;

  // driver state (from art_msgs)
  typedef art_msgs::DriverState DriverState;
  DriverState::_state_type driver_state_;

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

  boost::shared_ptr<devsteer> dev_;     // servo device interface
  boost::shared_ptr<testwheel> tw_;     // wheel self-test class

  // polynomials for converting between sensor voltage and angle
  boost::shared_ptr<Polynomial> apoly_; // angle to voltage conversion
#if defined(USE_VOLTAGE_POLYNOMIAL)
  boost::shared_ptr<Polynomial> vpoly_; // voltage to angle conversion
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

ArtSteer::ArtSteer():
  driver_state_(DriverState::CLOSED),
  angle_known_(false),
  wheel_calibrated_(false),
  wheel_tested_(false),
  dev_(new devsteer()),
  tw_(new testwheel(dev_))
{
  // polynomial to convert steering angles to sensor voltages
  // (inverse of vpoly_, only used when simulating steering angles)
  apoly_.reset(new Polynomial("apoly"));
  apoly_->add_coef(2.544);		// default constant coefficient
  apoly_->add_coef(-0.05456918);        // default coefficient of a
  apoly_->add_coef(0.00036568);		// default coefficient of a**2
  //apoly_->configure(cf, section);	// fill in .cfg values

#if defined(USE_VOLTAGE_POLYNOMIAL)
  // polynomial to convert sensor voltages to steering angles
  vpoly_.reset(new Polynomial("vpoly"));
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
    ROS_WARN("diagnostic mode ignored");

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
  dev_->Configure();

  // allocate and configure the steering wheel self-test
  tw_->Configure();

  // subscribe to relevant ROS topics
  static int qDepth = 1;
  ros::NodeHandle node;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  steering_cmd_ =
    node.subscribe("steering/cmd", qDepth, &ArtSteer::GetCmd, this, noDelay);
  steering_state_ =
    node.advertise<art_msgs::SteeringState>("steering/state", qDepth);
  steering_diag_ =
    node.advertise<art_msgs::SteeringDiagnostics>("steering/diag", qDepth);
  ioadr_state_ =
    node.subscribe("ioadr/state", qDepth, &ArtSteer::GetPos, this, noDelay);
}

ArtSteer::~ArtSteer()
{
  if (driver_state_ != DriverState::CLOSED)
    {
      close();
    }
}

/** open the device.
 *
 *  @return 0 if successful.
 *  @post state is OPENED (if successful)
 */
int ArtSteer::open()
{
  last_sensor_time_ = cur_sensor_time_ = 0.0;
  last_set_point_ = set_point_ = 180.0;	// preposterous starting value
  steering_angle_ = 0.0;                // needed for simulation
  calibration_cycle_ = 0;

  // open the device
  int rc = dev_->Open();
  if (rc == 0)
    {
      driver_state_ = DriverState::OPENED;
    }
  return rc;
}

/** close the device.
 *
 *  @post driver_state_ is CLOSED
 */
void ArtSteer::close()
{
  dev_->Close();
  driver_state_ = DriverState::CLOSED;
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

/* Calibrate current steering wheel position.
 *
 * Assumes the wheel will not move while in this state.  Unless
 * someone forgot to turn on the steering breaker, that will be
 * true. Otherwise, the wheel self-test should detect the error.
 *
 * @return 0 if success, < 0 if still calibrating, > 0 errno if failure
 *
 * @note This implementation minimizes the effects of sensor noise by
 * accumulating a moving average of the first "calibration_periods_"
 * readings.
 */
int ArtSteer::calibrate_wheel_position(void)
{
  int retval = -1;			// still working

  if (cur_sensor_time_ > last_sensor_time_) // new sensor data this cycle?
    {
      if (calibration_cycle_++ == 0)	// first time?
	  mean_voltage_ = 0.0;

      // accumulate moving average
      mean_voltage_ += ((steering_sensor_ - mean_voltage_)
		       / calibration_cycle_);
      ROS_DEBUG("cumulative steering sensor average: %.3f volts",
                mean_voltage_);

      if (calibration_cycle_ >= calibration_periods_)
	{
	  // finished with calibration
	  steering_angle_ = limit_travel(volts2degrees(mean_voltage_));
	  ROS_INFO("initial steering angle: %.2f degrees", steering_angle_);
	  retval = dev_->set_initial_angle(steering_angle_);
	}
    }
  return retval;
}

/** Get simulated wheel angle from driver
 *
 *  Only required when simulating the position sensor (even when using
 *  the real steering device).
 *
 *  @bug this should NOT be called until after calibrate_wheel_position
 *
 *  @post when simulate_ is true, tries to set steering_angle_,
 *        steering_sensor_, cur_sensor_time_, angle_known_
 */
void ArtSteer::read_wheel_angle(void)
{
  if (simulate_)                        // not using real sensor?
    {
      int rc = dev_->get_angle(steering_angle_);
      if (rc == 0)                      // got the angle?
        {
          steering_sensor_ = degrees2volts(steering_angle_);
          cur_sensor_time_ = ros::Time::now().toSec();
          angle_known_ = true;
        }
    }
}

/** publish current device status */
void ArtSteer::PublishStatus(void)
{
  art_msgs::SteeringState msg;         // steering state message

  msg.driver.state = driver_state_;
  msg.angle = steering_angle_;
  msg.sensor = steering_sensor_;
  msg.header.stamp = ros::Time::now();
  steering_state_.publish(msg);

  if (driver_state_ != DriverState::CLOSED)
    {
      // publish driver diagnostic info
      // (TODO: use ROS diagnostics package)
      dev_->publish_diag(steering_diag_);
    }
}

// Main function for device thread
// TODO rationalize these states and bits
void ArtSteer::run() 
{
  ros::Rate cycle(art_msgs::ArtHertz::STEERING); // set driver cycle rate

  while(ros::ok())
    {
      ros::spinOnce();                  // handle incoming commands

      switch (driver_state_)
        {
        case DriverState::CLOSED:
          {
            // TODO: spin slower while closed
            open();                     // try to open the device
            // state: CLOSED ==> OPENED (if successful)
            break;
          }

        case DriverState::OPENED:
          {
            read_wheel_angle();         // (may be simulated)
            if (wheel_tested_)          // wheel previously tested?
              {
                // state: OPENED ==> RUNNING
                driver_state_ = DriverState::RUNNING;
              }
            else if (wheel_calibrated_)	// initial position known?
              {
                int rc = tw_->Run(steering_angle_);
                if (rc < 0)             // test failed?
                  {
                    ROS_ERROR("steering self-test failure: closing driver");
                    close();            // (sets state to CLOSED)
                  }
                else if (rc > 0)       // test completed successfully?
                  {
                    wheel_tested_ = true;
                  }
              }
            else if (angle_known_)	// sensor data received?
              {
                int rc = calibrate_wheel_position();
		if (rc == 0)		// calibration succeeded?
		  {
		    wheel_calibrated_ = true;
		  }
		else if (rc > 0)	// calibration failed?
		  {
                    ROS_ERROR("steering calibration failure: closing driver");
                    close();            // (sets state to CLOSED)
		  }
              }
            break;
          }

        case DriverState::RUNNING:
          {
            read_wheel_angle();         // (may be simulated)
            int rc = dev_->check_status();
            if (rc != 0)                // status bad?
              {
		ROS_ERROR("bad steering status: closing driver");
		close();		// (sets state to CLOSED)
              }
            else if (fabs(set_point_ - last_set_point_) > epsilon)
              {
                // command steering position to match desired set point.
                rc = dev_->steering_absolute(set_point_);
                if (rc == 0)
                  last_set_point_ = set_point_;
              }
            break;
          }
        } // end switch on driver state

      PublishStatus();                  // publish current status
      last_sensor_time_ = cur_sensor_time_;
      cycle.sleep();                    // sleep until next cycle
    }
}
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "steering");
  ArtSteer dvr;
  dvr.run();

  return 0;
}
