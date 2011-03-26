/*
 *  Copyright (C) 2005-2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ROS node for controlling direction and speed of the ART
     autonomous vehicle.

     @todo make pilot driver a class
     @todo make speed interface more general
     @todo make each device a subclass of a common driver interface
     @todo (optionally) stop if no commands received recently.
     @todo shift to Park, when appropriate
 
     @author Jack O'Quin

 */

#include <ros/ros.h>

#include <angles/angles.h>
#include <driver_base/SensorLevels.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <art_msgs/ArtVehicle.h>
#include <art_msgs/ArtHertz.h>
#include <art_msgs/BrakeCommand.h>
#include <art_msgs/BrakeState.h>
#include <art_msgs/CarAccel.h>
#include <art_msgs/CarCommand.h>
#include <art_msgs/LearningCommand.h>
#include <art_msgs/PilotState.h>
#include <art_msgs/Shifter.h>
#include <art_msgs/SteeringCommand.h>
#include <art_msgs/SteeringState.h>
#include <art_msgs/ThrottleCommand.h>
#include <art_msgs/ThrottleState.h>

#include <art/conversions.h>
#include <art/epsilon.h>
#include <art/pid2.h>
#include <art/steering.h>

#include <art_pilot/PilotConfig.h>
typedef art_pilot::PilotConfig Config;

#include "learned_controller.h"
#include "speed.h"

typedef art_msgs::DriverState DriverState;


/**
 @brief controls the ART vehicle brake, throttle, steering and transmission

The pilot receives CarCommand messages from the navigator, then
translates them into commands to the servo motor actuators for
controlling the speed and direction of the vehicle.  It gets odometry
information from a separate node.

Subscribes:

- @b pilot/accel [art_msgs::CarAccel] acceleration and steering command
- @b pilot/cmd [art_msgs::CarCommand] velocity and steering angle command
- @b imu [sensor_msgs::Imu] estimate of robot accelerations
- @b odom [nav_msgs::Odometry] estimate of robot position and velocity.

- @b brake/state [art_msgs::BrakeState] brake status.
- @b shifter/state [art_msgs::Shifter] shifter relays status.
- @b steering/state [art_msgs::SteeringState] steering status.
- @b throttle/state [art_msgs::ThrottleState] throttle status.

Publishes:

- @b pilot/state [art_msgs::PilotState] current pilot state information.
- @b brake/cmd [art_msgs::BrakeCommand] brake commands.
- @b shifter/cmd [art_msgs::Shifter] shifter commands.
- @b steering/cmd [art_msgs::SteeringCommand] steering commands.
- @b throttle/cmd [art_msgs::ThrottleCommand] throttle commands.

*/

// global variables
namespace
{

  // ROS topics used by this node
  ros::Subscriber accel_cmd_;           // CarAccel command
  ros::Subscriber car_cmd_;             // CarCommand

  ros::Subscriber brake_state_;
  ros::Subscriber imu_state_;           // inertial measurements
  ros::Subscriber odom_state_;          // odometry
  ros::Subscriber shifter_state_;
  ros::Subscriber steering_state_;
  ros::Subscriber throttle_state_;

  ros::Subscriber learning_cmd_;

  ros::Publisher brake_cmd_;            // brake command
  ros::Publisher pilot_state_;          // pilot state
  ros::Publisher shifter_cmd_;          // shifter command
  ros::Publisher steering_cmd_;         // steering command
  ros::Publisher throttle_cmd_;         // throttle command

  // configuration
  Config config_;                       // dynamic configuration

  // servo control
  float brake_position_ = 1.0;
  float throttle_position_ = 0.0;

  art_msgs::BrakeCommand    brake_msg_;
  art_msgs::Shifter         shifter_msg_;
  art_msgs::SteeringCommand steering_msg_;
  art_msgs::ThrottleCommand throttle_msg_;

  // Odometry data
  sensor_msgs::Imu imu_msg_;
  nav_msgs::Odometry odom_msg_;

  ros::Time current_time_;              // time current cycle began
  ros::Time shift_time_;                // time last shift requested

  // times when messages received
  ros::Time brake_time_;
  ros::Time goal_time_;                 // latest goal command
  ros::Time imu_time_;
  ros::Time odom_time_;
  ros::Time shifter_time_;
  ros::Time steering_time_;
  ros::Time throttle_time_;

  art_msgs::PilotState pstate_msg_;     // pilot state message

  boost::shared_ptr<SpeedControl> speed_; // speed control
};

/** clamp value to range
 *
 *  @lower minimum value of range
 *  @value data to compare
 *  @upper maximum value of range
 *  @return @a value unless it is outside [@a lower, @a upper];
 *          or the nearest limit, otherwise.
 */
static inline float clamp(float lower, float value, float upper)
{
  return std::max(lower, std::min(value, upper));
}

/** allocate appropriate speed control subclass for this configuration */
void allocateSpeedControl(const Config &newconfig)
{

  if (newconfig.use_learned_controller) 
    {
      ROS_INFO("using RL learned controller for speed control");
      speed_.reset(new LearnedSpeedControl());
    }
  else if (newconfig.use_accel_matrix)
    {
      ROS_INFO("using acceleration matrix for speed control");
      speed_.reset(new SpeedControlMatrix());
    }
  else
    {
      ROS_INFO("using brake and throttle PID for speed control");
      speed_.reset(new SpeedControlPID());
    }

  // initialize brake and throttle positions in speed controller
  // TODO remove this, it is ugly
  speed_->set_brake_position(brake_position_);
  speed_->set_throttle_position(throttle_position_);
}

/** validate target CarCommand2 values */
void validateTarget(void)
{
  pstate_msg_.target.goal_velocity = clamp(config_.minspeed,
                                           pstate_msg_.target.goal_velocity,
                                           config_.maxspeed);
  ROS_DEBUG("target velocity goal is %.2f m/s",
            pstate_msg_.target.goal_velocity);

  // TODO clamp angle to permitted range
  ROS_DEBUG("target steering angle is %.3f (degrees)",
            angles::to_degrees(pstate_msg_.target.steering_angle));
}

void processCarAccel(const art_msgs::CarAccel::ConstPtr &msg)
{
  goal_time_ = msg->header.stamp;
  pstate_msg_.target = msg->control;
  validateTarget();
}

// will be deprecated
void processCarCommand(const art_msgs::CarCommand::ConstPtr &msg)
{
  goal_time_ = msg->header.stamp;
  pstate_msg_.target.acceleration = 0.0; // use some default?
  pstate_msg_.target.goal_velocity = msg->control.velocity;
  pstate_msg_.target.steering_angle = angles::from_degrees(msg->control.angle);
  if (pstate_msg_.target.goal_velocity > 0.0)
    pstate_msg_.target.gear = art_msgs::CarControl2::Drive;
  else if (pstate_msg_.target.goal_velocity < 0.0)
    pstate_msg_.target.gear = art_msgs::CarControl2::Reverse;
  validateTarget();
}

void processImu(const sensor_msgs::Imu::ConstPtr &imuIn)
{
  imu_time_ = imuIn->header.stamp;
  pstate_msg_.imu.state = art_msgs::DriverState::RUNNING;
  pstate_msg_.current.acceleration = imuIn->linear_acceleration.x;
  imu_msg_ = *imuIn;                    // save entire message
}

void processOdom(const nav_msgs::Odometry::ConstPtr &odomIn)
{
  odom_time_ = odomIn->header.stamp;
  pstate_msg_.odom.state = art_msgs::DriverState::RUNNING;
  pstate_msg_.current.goal_velocity = odomIn->twist.twist.linear.x;
  odom_msg_ = *odomIn;
}

void processBrake(const art_msgs::BrakeState::ConstPtr &brakeIn)
{
  brake_time_ = brakeIn->header.stamp;
  pstate_msg_.brake.state = art_msgs::DriverState::RUNNING;
  brake_position_ = brakeIn->position;
  speed_->set_brake_position(brake_position_);
}

void processThrottle(const art_msgs::ThrottleState::ConstPtr &throttleIn)
{
  throttle_time_ = throttleIn->header.stamp;
  pstate_msg_.throttle.state = art_msgs::DriverState::RUNNING;
  throttle_position_ = throttleIn->position;
  speed_->set_throttle_position(throttle_position_);
}

void processShifter(const art_msgs::Shifter::ConstPtr &shifterIn)
{
  shifter_time_ = shifterIn->header.stamp;
  pstate_msg_.shifter.state = art_msgs::DriverState::RUNNING;
  /// @todo reconcile CarControl2 and Shifter gear numbers
  switch (shifterIn->gear)
    {
    case art_msgs::Shifter::Drive:
      pstate_msg_.current.gear = art_msgs::CarControl2::Drive;
      break;
    case art_msgs::Shifter::Park:
      pstate_msg_.current.gear = art_msgs::CarControl2::Park;
      break;
    case art_msgs::Shifter::Reverse:
      pstate_msg_.current.gear = art_msgs::CarControl2::Reverse;
      break;
    default:
      ROS_WARN_STREAM("Unexpected gear number: " << shifterIn->gear
                      << " (ignored)");
    }
}

void processSteering(const art_msgs::SteeringState::ConstPtr &steeringIn)
{
  steering_time_ = steeringIn->header.stamp;
  pstate_msg_.steering.state = steeringIn->driver.state;
  pstate_msg_.current.steering_angle = angles::from_degrees(steeringIn->angle);
}

/// @todo create a better learning interface (perhaps a service?)
void processLearning(const art_msgs::LearningCommand::ConstPtr &learningIn)
{
  pstate_msg_.preempted = (learningIn->pilotActive == 0);
  ROS_INFO_STREAM("Pilot is "
                  << (pstate_msg_.preempted? "preempted": "active"));
}

/** handle dynamic reconfigure service request
 *
 * @param newconfig new configuration from dynamic reconfigure client,
 *        becomes the service reply message as updated here.
 * @param level SensorLevels value (0xffffffff on initial call)
 *
 * This is done without any locking because it is called in the same
 * thread as ros::spinOnce() and all the topic subscription callbacks.
 */
void reconfig(Config &newconfig, uint32_t level)
{
  ROS_INFO("pilot dynamic reconfigure, level 0x%08x", level);

  if (level & driver_base::SensorLevels::RECONFIGURE_CLOSE)
    {
      allocateSpeedControl(newconfig);
    }

  config_ = newconfig;
}

void halt(float cur_speed);             // forward declaration

/** Adjust velocity to match goal.
 *  
 *  @param cur_speed absolute value of current velocity in m/s
 *  @param error difference between that and our immediate goal
 */
void adjustVelocity(float cur_speed, float error)
{
  if (pstate_msg_.steering.state != DriverState::RUNNING
      || pstate_msg_.odom.state != DriverState::RUNNING)
    {
      // critical component failure: halt the car
      halt(cur_speed);
      return;
    }

  // Adjust brake and throttle settings.
  speed_->adjust(cur_speed, error,
                 &brake_msg_.position, &throttle_msg_.position);

  brake_msg_.position = clamp(0.0, brake_msg_.position, 1.0);
  if (fabsf(brake_msg_.position - brake_position_) > EPSILON_BRAKE)
    {
      brake_msg_.header.stamp = current_time_;
      brake_cmd_.publish(brake_msg_);
    }

  throttle_msg_.position = clamp(0.0, throttle_msg_.position, 1.0);
  if (fabsf(throttle_msg_.position - throttle_position_) > EPSILON_THROTTLE)
    {
      throttle_msg_.header.stamp = current_time_;
      throttle_cmd_.publish(throttle_msg_);
    }
}

/** halt -- soft version of hardware E-Stop.
 *  
 *   The goal is to bring the vehicle to a halt as quickly as possible,
 *   while remaining safely under control.  Normally, navigator sends
 *   gradually declining speed requests when bringing the vehicle to a
 *   controlled stop.  The only abrupt requests we see are in
 *   "emergency" stop situations, when there was a pause request, or no
 *   clear path around an obstacle.
 *  
 *   @param cur_speed absolute value of current velocity in m/sec
 */
void halt(float cur_speed)
{
  // At high speed use adjustVelocity() to slow the vehicle down some
  // before slamming on the brakes.  Even with ABS to avoid lock-up,
  // it should be safer to bring the vehicle to a more gradual stop.

#define SAFE_FULL_BRAKE_SPEED mph2mps(45) // 45 mph (in m/sec)

  if (cur_speed > SAFE_FULL_BRAKE_SPEED)
    {
      adjustVelocity(cur_speed, -cur_speed);
      return;
    }
  else if (cur_speed < Epsilon::speed)
    {
      // Already stopped.  Ease up on the brake to reduce strain on
      // the actuator.  Brake hold position *must* be adequate to
      // prevent motion, even on a hill.
      //
      // TODO: detect motion after stopping and apply more brake.
      brake_msg_.header.stamp = current_time_;
      brake_msg_.position = config_.brake_hold;
      brake_cmd_.publish(brake_msg_);
    }
  else
    {
      // Stop the moving vehicle very quickly.
      //
      //  Apply min throttle and max brake at the same time.  This is
      //  an exception to the general rule of never applying brake and
      //  throttle together.  There seems to be enough inertia in the
      //  brake mechanism for this to be safe.
      throttle_msg_.header.stamp = current_time_;
      throttle_msg_.position = 0.0;
      throttle_cmd_.publish(throttle_msg_);
      brake_msg_.header.stamp = current_time_;
      brake_msg_.position = 1.0;
      brake_cmd_.publish(brake_msg_);
    }
}

/** Adjust steering angle.
 *  
 *  We do not use PID control, because the odometry does not provide
 *  accurate yaw speed feedback.  Instead, we directly compute the
 *  corresponding steering angle.  We can use open loop control at this
 *  level, because navigator monitors our actual course and will
 *  request any steering changes needed to reach its goal.
 *
 *  @todo Limit angle actually requested based on current velocity to
 *  avoid sudden turns at high speeds.
 */
void adjustSteering()
{
  if (pstate_msg_.current.steering_angle != pstate_msg_.target.steering_angle)
    {
      // Set the steering angle in degrees.
      float steer_degrees =
        angles::to_degrees(pstate_msg_.target.steering_angle);
      ROS_DEBUG("requesting steering angle = %.1f (degrees)", steer_degrees);
      steering_msg_.header.stamp = current_time_;
      steering_msg_.angle = steer_degrees;
      steering_cmd_.publish(steering_msg_);
    }
}

/** check hardware device activity
 *
 *  @pre current_time_ updated for this pilot cycle
 *
 *  @param last_msg last message time stamp from this device
 *  @param state[in,out] pointer to this device's state
 *
 *  @post pstate_msg_ updated to reflect current status for this device
 */
void checkActivity(const ros::Time &last_msg,
                   DriverState::_state_type *state)
{
  if (current_time_.toSec() - last_msg.toSec() > config_.timeout)
    {
      // device not publishing often enough: consider it CLOSED
      *state = DriverState::CLOSED;
    }
}

/** monitor hardware status based on current inputs
 *
 *  @post current_time_ updated for this pilot cycle
 *  @post pstate_msg_ updated to reflect current control hardware status
 */
void monitorHardware(void)
{
  current_time_ = ros::Time::now();
  pstate_msg_.header.stamp = current_time_;

  // accumulate new pilot state based on device states
  checkActivity(brake_time_, &pstate_msg_.brake.state);
  checkActivity(imu_time_, &pstate_msg_.imu.state);
  checkActivity(odom_time_, &pstate_msg_.odom.state);
  checkActivity(shifter_time_, &pstate_msg_.shifter.state);
  checkActivity(steering_time_, &pstate_msg_.steering.state);
  checkActivity(throttle_time_, &pstate_msg_.throttle.state);

  pstate_msg_.pilot.state = DriverState::RUNNING;
  if (pstate_msg_.brake.state != DriverState::RUNNING
      || pstate_msg_.odom.state != DriverState::RUNNING
      || pstate_msg_.steering.state != DriverState::RUNNING
      || pstate_msg_.throttle.state != DriverState::RUNNING)
    {
      ROS_WARN_THROTTLE(40, "critical component failure, pilot not running");
      pstate_msg_.pilot.state = DriverState::OPENED;
    }
}

// this typedef and the function that follows categorize a given speed
// as moving forward, backward or stopped.
typedef enum
  {
    Stopped = 0,
    Forward,
    Backward
  } speed_range_t;

static inline speed_range_t speed_range(float speed)
{
  if (speed > Epsilon::speed)		// moving forward?
      return Forward;

  if (speed >= -Epsilon::speed)		// close to zero?
      return Stopped;

  return Backward;
}

// Transmission shifting states
namespace Transmission
{
  typedef enum
    {
      Drive		= 0x01,		// Transmission in Drive
      Reverse		= 0x02,		// Transmission in Reverse
      ShiftDrive	= 0x10,		// Shifting into Drive
      ShiftReverse	= 0x20		// Shifting into Reverse
    } state_t;
}

/** Speed control
 *  
 *  Manage the shifter as a finite state machine.  Inputs are the
 *  current and goal velocity ranges (+,0,-).  If these velocities
 *  differ in sign, the vehicle must first be brought to a stop, then
 *  one of the transmission shift relays set for one second, before
 *  the vehicle can begin moving in the opposite direction.
*/
void speedControl(void)
{
  // return immediately if pilot preempted for learning speed control
  if (pstate_msg_.preempted)
    return;

  static const double shift_duration = 1.0; // hold relay one second
  static Transmission::state_t shifting_state = Transmission::Drive;

  float speed = pstate_msg_.current.goal_velocity;
  float goal = pstate_msg_.target.goal_velocity;
  float error = goal - speed;

  speed_range_t cur_range = speed_range(speed);
  speed_range_t goal_range = speed_range(goal);

  ROS_DEBUG("Shifting state: 0x%02x, speed: %.3f m/s, goal: %.3f",
	    shifting_state, speed, goal);

  switch(shifting_state)
    {
    case Transmission::Drive:
      // TODO: make sure shifter relays are off now
      if (Forward == goal_range)
	{
	  adjustVelocity(speed, error);
	}
      else if (Stopped == cur_range && Backward == goal_range)
	{
	  shifting_state = Transmission::ShiftReverse;
          shifter_msg_.header.stamp = current_time_;
          shifter_msg_.gear = art_msgs::Shifter::Reverse;
          shifter_cmd_.publish(shifter_msg_);
          shift_time_ = current_time_;
	}
      else
	{
	  halt(speed);
          speed_->reset();
	}
      break;

    case Transmission::ShiftReverse:
      // make sure the transmission actually shifted
      if (pstate_msg_.current.gear != art_msgs::CarControl2::Reverse)
	{
	  // repeat shift command until it works
          shifter_msg_.header.stamp = current_time_;
          shifter_msg_.gear = art_msgs::Shifter::Reverse;
          shifter_cmd_.publish(shifter_msg_);
	  shift_time_ = current_time_;
          ROS_DEBUG("repeated shift command at %.6f", shift_time_.toSec());
	}
      // make sure the relay was set long enough
      else if ((current_time_.toSec() - shift_time_.toSec())
               >= shift_duration)
	{
          shifter_msg_.header.stamp = current_time_;
          shifter_msg_.gear = art_msgs::Shifter::Reset;
          shifter_cmd_.publish(shifter_msg_);
	  if (Backward == goal_range)
	    {
	      shifting_state = Transmission::Reverse;
	      adjustVelocity(-speed, -error);
	    }
	  else if (Stopped == goal_range)
	    {
	      shifting_state = Transmission::Reverse;
	      halt(-speed);
	      speed_->reset();
	    }
	  else // Dang!  we want to go forward now
	    {
	      shifting_state = Transmission::ShiftDrive;
              shifter_msg_.header.stamp = current_time_;
              shifter_msg_.gear = art_msgs::Shifter::Drive;
              shifter_cmd_.publish(shifter_msg_);
	      shift_time_ = current_time_;
	    }
	}
      break;

    case Transmission::Reverse:
      // TODO: make sure shifter relays are off now
      if (Backward == goal_range)
	{
	  adjustVelocity(-speed, -error);
	}
      else if (Stopped == cur_range && Forward == goal_range)
	{
	  shifting_state = Transmission::ShiftDrive;
          shifter_msg_.header.stamp = current_time_;
          shifter_msg_.gear = art_msgs::Shifter::Drive;
          shifter_cmd_.publish(shifter_msg_);
	  shift_time_ = current_time_;
	}
      else
	{
	  halt(-speed);
	  speed_->reset();
	}
      break;

    case Transmission::ShiftDrive:
      // make sure the transmission actually shifted
      if (pstate_msg_.current.gear != art_msgs::CarControl2::Drive)
	{
	  // repeat shift command until it works
          shifter_msg_.header.stamp = current_time_;
          shifter_msg_.gear = art_msgs::Shifter::Drive;
          shifter_cmd_.publish(shifter_msg_);
	  shift_time_ = current_time_;
          ROS_DEBUG("repeated shift command at %.6f", shift_time_.toSec());
	}
      // make sure the relay was set long enough
      else if ((current_time_.toSec() - shift_time_.toSec())
               >= shift_duration)
	{
          shifter_msg_.header.stamp = current_time_;
          shifter_msg_.gear = art_msgs::Shifter::Reset;
          shifter_cmd_.publish(shifter_msg_);
	  if (Forward == goal_range)
	    {
	      shifting_state = Transmission::Drive;
	      adjustVelocity(speed, error);
	    }
	  else if (Stopped == goal_range)
	    {
	      shifting_state = Transmission::Drive;
	      halt(speed);
	      speed_->reset();
	    }
	  else // Dang!  we want to go backward now
	    {
	      shifting_state = Transmission::ShiftReverse;
              shifter_msg_.header.stamp = current_time_;
              shifter_msg_.gear = art_msgs::Shifter::Reverse;
              shifter_cmd_.publish(shifter_msg_);
	      shift_time_ = current_time_;
	    }
	}
      break;
    }
}

// make this a constructor?
void setup(ros::NodeHandle node)
{
  // Declare dynamic reconfigure callback before subscribing to topics.
  dynamic_reconfigure::Server<Config> srv;
  dynamic_reconfigure::Server<Config>::CallbackType cb =
    boost::bind(&reconfig, _1, _2);
  srv.setCallback(cb);

  // no delay: we always want the most recent data
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  int qDepth = 1;

  // command topics (car_cmd will be deprecated)
  accel_cmd_ = node.subscribe("pilot/accel", qDepth, processCarAccel, noDelay);
  car_cmd_ = node.subscribe("pilot/cmd", qDepth, processCarCommand, noDelay);
  learning_cmd_ = node.subscribe("pilot/learningCmd", qDepth,
                                 processLearning, noDelay);

  // topics to read
  brake_state_ = node.subscribe("brake/state", qDepth, processBrake, noDelay);
  imu_state_ = node.subscribe("imu", qDepth, processImu, noDelay);
  odom_state_ = node.subscribe("odom", qDepth, processOdom, noDelay);
  shifter_state_ = node.subscribe("shifter/state", qDepth,
                                  processShifter, noDelay);
  steering_state_ = node.subscribe("steering/state", qDepth,
                                   processSteering, noDelay);
  throttle_state_ = node.subscribe("throttle/state", qDepth,
                                   processThrottle, noDelay);

  // topic for publishing pilot state
  pilot_state_ = node.advertise<art_msgs::PilotState>("pilot/state", qDepth);
  pstate_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;
  
  // initialize servo command interfaces and messages
  brake_cmd_ = node.advertise<art_msgs::BrakeCommand>("brake/cmd", qDepth);
  brake_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;
  brake_msg_.request = art_msgs::BrakeCommand::Absolute;
  brake_msg_.position = 1.0;

  shifter_cmd_ = node.advertise<art_msgs::Shifter>("shifter/cmd", qDepth);
  shifter_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;

  steering_cmd_ =
    node.advertise<art_msgs::SteeringCommand>("steering/cmd", qDepth);
  steering_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;
  steering_msg_.request = art_msgs::SteeringCommand::Degrees;

  throttle_cmd_ =
    node.advertise<art_msgs::ThrottleCommand>("throttle/cmd", qDepth);
  throttle_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;
  throttle_msg_.request = art_msgs::ThrottleCommand::Absolute;
  throttle_msg_.position = 0.0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pilot");
  ros::NodeHandle node;

  setup(node);
 
  // Main loop
  ros::Rate cycle(art_msgs::ArtHertz::PILOT); // set driver cycle rate
  while(ros::ok())
    {
      ros::spinOnce();                  // handle incoming messages

      monitorHardware();                // monitor device status

      // TODO only when reconfig() is called
      speed_->configure();              // check for parameter updates

      // issue control commands
      speedControl();
      adjustSteering();

      pilot_state_.publish(pstate_msg_); // publish updated state message

      cycle.sleep();                    // sleep until next cycle
    }

  return 0;
}
