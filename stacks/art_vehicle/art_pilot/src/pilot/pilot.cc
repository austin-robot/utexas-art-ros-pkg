/*
 *  Copyright (C) 2005-2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ROS node for controlling direction and speed of the ART
     autonomous vehicle.

     @todo (optionally) stop if no commands received recently.
     @todo shift to Park, when appropriate
     @todo distinguish device failures before and after initialization
     @todo deprecate old CarCommand message interface
 
     @author Jack O'Quin

 */

#include <ros/ros.h>

#include <angles/angles.h>
#include <driver_base/SensorLevels.h>
#include <dynamic_reconfigure/server.h>

#include "device_impl.h"                // sensor and servo device interfaces

#include <art_msgs/ArtVehicle.h>
#include <art_msgs/ArtHertz.h>
#include <art_msgs/CarDriveStamped.h>
#include <art_msgs/CarCommand.h>
#include <art_msgs/Epsilon.h>
#include <art_msgs/Gear.h>
#include <art_msgs/LearningCommand.h>
#include <art_msgs/PilotState.h>

#include <art/conversions.h>
#include <art/steering.h>

#include <art_pilot/PilotConfig.h>
typedef art_pilot::PilotConfig Config;

#include "accel.h"

typedef art_msgs::DriverState DriverState;
using art_msgs::Epsilon;


/**
 @brief controls the ART vehicle brake, throttle, steering and transmission

The pilot receives CarDriveStamped messages from the navigator, then
translates them into commands to the servo motor actuators for
controlling the speed and direction of the vehicle.  It gets odometry
information from a separate node.

Subscribes:

- @b pilot/drive [art_msgs::CarDriveStamped] driving command
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

/** Pilot node class. */
class PilotNode
{
public:

  PilotNode(ros::NodeHandle node);
  void spin(void);

private:

  void adjustSteering(void);
  void halt(void);
  void monitorHardware(void);
  void processCarDrive(const art_msgs::CarDriveStamped::ConstPtr &msg);
  void processCarCommand(const art_msgs::CarCommand::ConstPtr &msg);
  void processLearning(const art_msgs::LearningCommand::ConstPtr &learningIn);
  void reconfig(Config &newconfig, uint32_t level);
  void speedControl(void);
  void validateTarget(void);

  bool is_shifting_;                    // is transmission active?

  typedef dynamic_reconfigure::Server<Config> ReconfigServer;
  boost::shared_ptr<ReconfigServer> reconfig_server_;

  // ROS topics used by this node
  ros::Subscriber accel_cmd_;           // CarDriveStamped command
  ros::Subscriber car_cmd_;             // CarCommand

  // Device interfaces used by pilot
  boost::shared_ptr<device_interface::DeviceBrake> brake_;
  boost::shared_ptr<device_interface::DeviceImu> imu_;
  boost::shared_ptr<device_interface::DeviceOdom> odom_;
  boost::shared_ptr<device_interface::DeviceShifter> shifter_;
  boost::shared_ptr<device_interface::DeviceSteering> steering_;
  boost::shared_ptr<device_interface::DeviceThrottle> throttle_;

  ros::Subscriber learning_cmd_;

  ros::Publisher pilot_state_;          // pilot state

  // configuration
  Config config_;                       // dynamic configuration
  ros::Duration timeout_;               // device timeout (sec)

  ros::Time current_time_;              // time current cycle began

  // times when messages received
  ros::Time goal_time_;                 // latest goal command

  art_msgs::PilotState pstate_msg_;     // pilot state message

  boost::shared_ptr<pilot::AccelBase> accel_;  // acceleration controller
};


//////////////////////////////////////////////////////////////////
// public methods
//////////////////////////////////////////////////////////////////

/** constructor */
PilotNode::PilotNode(ros::NodeHandle node):
  is_shifting_(false),
  reconfig_server_(new dynamic_reconfigure::Server<Config>)
{
  // Must declare dynamic reconfigure callback before initializing
  // devices or subscribing to topics.
  reconfig_server_->setCallback(boost::bind(&PilotNode::reconfig,
                                            this, _1, _2));

  // allocate and initialize device interfaces
  brake_.reset(new device_interface::DeviceBrake(node));
  imu_.reset(new device_interface::DeviceImu(node));
  odom_.reset(new device_interface::DeviceOdom(node));
  shifter_.reset(new device_interface::DeviceShifter(node));
  steering_.reset(new device_interface::DeviceSteering(node));
  throttle_.reset(new device_interface::DeviceThrottle(node));

  // no delay: we always want the most recent data
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  int qDepth = 1;

  // command topics (car_cmd will be deprecated)
  accel_cmd_ = node.subscribe("pilot/drive", qDepth,
                              &PilotNode::processCarDrive, this, noDelay);
  car_cmd_ = node.subscribe("pilot/cmd", qDepth,
                            &PilotNode::processCarCommand, this , noDelay);
  learning_cmd_ = node.subscribe("pilot/learningCmd", qDepth,
                                 &PilotNode::processLearning, this, noDelay);

  // topic for publishing pilot state
  pilot_state_ = node.advertise<art_msgs::PilotState>("pilot/state", qDepth);
  pstate_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;
}

/** main loop */
void PilotNode::spin(void)
{
  // Main loop
  ros::Rate cycle(art_msgs::ArtHertz::PILOT); // set driver cycle rate
  while(ros::ok())
    {
      ros::spinOnce();                  // handle incoming messages

      monitorHardware();                // monitor device status

      // issue control commands
      speedControl();
      adjustSteering();

      pilot_state_.publish(pstate_msg_); // publish updated state message

      cycle.sleep();                    // sleep until next cycle
    }
}


//////////////////////////////////////////////////////////////////
// private methods
//////////////////////////////////////////////////////////////////

/** Adjust steering angle.
 *  
 *  We do not use PID control, because the odometry does not provide
 *  accurate yaw speed feedback.  Instead, we directly compute the
 *  corresponding steering angle.  We can use open loop control at this
 *  level, because navigator monitors our actual course and will
 *  request any steering changes needed to reach its goal.
 *
 *  @todo Limit angle actually requested based on current velocity to
 *        avoid unsafe high speed turns.
 */
void PilotNode::adjustSteering(void)
{
  if (config_.human_steering)           // pilot not steering?
    return;

  if (fabs(pstate_msg_.target.steering_angle
           - pstate_msg_.current.steering_angle)
      > Epsilon::steering_angle)
    {
      // Set the steering angle in degrees.
      float steer_degrees =
        angles::to_degrees(pstate_msg_.target.steering_angle);
      ROS_DEBUG("requesting steering angle = %.1f (degrees)", steer_degrees);
      steering_->publish(steer_degrees, current_time_);
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
 *   @note The target goal velocity may not always be zero, due to the
 *         gear shifting requirements.
 *
 *   @post Since this function bypasses the normal acceleration
 *         controller, it is reset on exit.
 */
void PilotNode::halt(void)
{
  // absolute value of current velocity in m/sec
  float abs_speed = pstate_msg_.current.speed;
  if (abs_speed < Epsilon::speed)
    {
      // Already stopped.  Ease up on the brake to reduce strain on
      // the actuator.  Brake hold position *must* be adequate to
      // prevent motion, even on a hill.
      brake_->publish(config_.brake_hold, current_time_);
    }
  else
    {
      // Stop the moving vehicle very quickly.
      //
      //  Apply min throttle and max brake at the same time.  This is
      //  an exception to the general rule of never applying brake and
      //  throttle together.  There seems to be enough inertia in the
      //  brake mechanism for this to be safe.
      throttle_->publish(0.0, current_time_);
      brake_->publish(1.0, current_time_);
    }
  accel_->reset();
}

/** monitor hardware status based on current inputs
 *
 *  @post pstate_msg_ updated to reflect current control hardware
 *        status and time of this cycle
 */
void PilotNode::monitorHardware(void)
{
  // update current pilot state
  current_time_ = ros::Time::now();
  pstate_msg_.header.stamp = current_time_;
  pstate_msg_.current.acceleration = fabs(imu_->value());
  pstate_msg_.current.speed = fabs(odom_->value());
  pstate_msg_.current.steering_angle =
    angles::from_degrees(steering_->value());
  pstate_msg_.current.gear.value = shifter_->value();

  // Stage time should not ever start at zero, but there seems to be a
  // bug.  In any case it could be < timeout_ (unlike wall time).
  ros::Time recently;              // minimum time for recent messages
  if (current_time_ > (recently + timeout_))
    {
      recently = current_time_ - timeout_;
    }

  // accumulate new pilot state based on device states
  pstate_msg_.brake.state = brake_->state(recently);
  pstate_msg_.imu.state = imu_->state(recently);
  pstate_msg_.odom.state = odom_->state(recently);
  pstate_msg_.shifter.state = shifter_->state(recently);
  pstate_msg_.steering.state = steering_->state(recently);
  pstate_msg_.throttle.state = throttle_->state(recently);

  /// @todo Optionally check if no commands received recently.

  pstate_msg_.pilot.state = DriverState::RUNNING;
  if (pstate_msg_.brake.state != DriverState::RUNNING
      || pstate_msg_.imu.state != DriverState::RUNNING
      || pstate_msg_.odom.state != DriverState::RUNNING
      || (!config_.human_steering
          && pstate_msg_.steering.state != DriverState::RUNNING)
      || pstate_msg_.throttle.state != DriverState::RUNNING)
    {
      // pilot is not running
      pstate_msg_.pilot.state = DriverState::OPENED;
      ROS_WARN_THROTTLE(40, "critical component failure, pilot not running");
      // reset latest target request
      pstate_msg_.target = art_msgs::CarDrive();
    }
}

/** CarDriveStamped message callback */
void PilotNode::processCarDrive(const art_msgs::CarDriveStamped::ConstPtr &msg)
{
  goal_time_ = msg->header.stamp;
  pstate_msg_.target = msg->control;
  validateTarget();
}

/** CarCommand message callback (now DEPRECATED) */
void PilotNode::processCarCommand(const art_msgs::CarCommand::ConstPtr &msg)
{
  ROS_WARN_THROTTLE(100, "CarCommand deprecated: use CarDriveStamped.");

  goal_time_ = msg->header.stamp;
  pstate_msg_.target.steering_angle = angles::from_degrees(msg->control.angle);
  pstate_msg_.target.behavior.value = art_msgs::PilotBehavior::Run;

  pstate_msg_.target.jerk = 0.0;
  pstate_msg_.target.acceleration = 0.0;
  pstate_msg_.target.speed = msg->control.velocity;
  if (pstate_msg_.target.speed > 0.0)
    {
      pstate_msg_.target.gear.value = art_msgs::Gear::Drive;
    }
  else if (pstate_msg_.target.speed < 0.0)
    {
      // in reverse: make speed positive
      pstate_msg_.target.speed = -msg->control.velocity;
      pstate_msg_.target.gear.value = art_msgs::Gear::Reverse;
    }
  else
    {
      pstate_msg_.target.gear.value = art_msgs::Gear::Naught;
    }

  validateTarget();
}

/** LearningCommand message callback (DEPRECATED) */
void PilotNode::processLearning(const art_msgs::LearningCommand::ConstPtr
                                &learningIn)
{
  ROS_WARN_THROTTLE(100, "LearningCommand deprecated: use CarDriveStamped.");
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
void PilotNode::reconfig(Config &newconfig, uint32_t level)
{
  ROS_INFO("pilot dynamic reconfigure, level 0x%08x", level);

  if (level & driver_base::SensorLevels::RECONFIGURE_CLOSE)
    {
      // reallocate acceleration controller using new configuration
      accel_ = pilot::allocAccel(newconfig);
    }
  else
    {
      // pass any other parameters to existing acceleration controller
      accel_->reconfigure(newconfig);
    }

  config_ = newconfig;
  timeout_ = ros::Duration(config_.timeout);
}


/** Speed control
 *  
 *  Manage the shifter.  Inputs are the current and target states.  If
 *  a gear shift is requested, the vehicle must first be brought to a
 *  stop, then one of the transmission shift relays set for one second
 *  (then reset), before the vehicle can begin moving in the opposite
 *  direction.
 */
void PilotNode::speedControl(void)
{
  // do nothing while pilot preempted for learning speed control (no
  // servo commands permitted)
  if (pstate_msg_.preempted)
    return;

  float dt = 1.0 / art_msgs::ArtHertz::PILOT;
  float abs_current_speed = pstate_msg_.current.speed;
  float abs_target_speed = pstate_msg_.target.speed;

  if (is_shifting_)
    {
      if (shifter_->is_reset())
        {
          // all relays are reset now
          is_shifting_ = false;
        }
      else if (!shifter_->busy())
        {
          // original operation complete, reset shifter relays
          shifter_->publish(art_msgs::Shifter::Reset, current_time_);
        }
    }

  if (pstate_msg_.current.gear.value == pstate_msg_.target.gear.value
      || pstate_msg_.target.gear.value == art_msgs::Gear::Naught)
    {
      // no shift required
      if ((pstate_msg_.pilot.state != DriverState::RUNNING)
          || (pstate_msg_.target.gear.value == art_msgs::Gear::Park)
          || (pstate_msg_.target.gear.value == art_msgs::Gear::Neutral)
          || shifter_->busy())
        {
          // unable to proceed
          halt();
        }
      else
        {
          // driving in the correct gear
          if ((abs_target_speed < Epsilon::speed)
              && ((pstate_msg_.target.acceleration == 0.0)
                  || (pstate_msg_.target.acceleration * dt
                      > abs_current_speed)))
            {
              // stop requested and acceleration not specified, or
              // almost stopped
              halt();
            }
          else
            {
              // have acceleration controller adjust speed
              accel_->adjust(pstate_msg_, brake_, throttle_);
            }
        }
    }
  else
    {
      // not in desired gear, shift (still) needed
      is_shifting_ = true;
      if (!shifter_->busy())
        {
          // request shift until driver reports success
          shifter_->publish(pstate_msg_.target.gear.value, current_time_);
        }

      halt();                           // never move while shifting
    }
}


/** validate target CarDrive values */
void PilotNode::validateTarget(void)
{
  // Warn if negative speed, acceleration and jerk.
  if (pstate_msg_.target.speed < 0.0
      || pstate_msg_.target.acceleration < 0.0
      || pstate_msg_.target.jerk < 0.0)
    {
      ROS_WARN_THROTTLE(100, "Negative speed, acceleration and jerk "
                        "are DEPRECATED (using absolute value).");
      pstate_msg_.target.speed = fabs(pstate_msg_.target.speed);
      pstate_msg_.target.acceleration = fabs(pstate_msg_.target.acceleration);
      pstate_msg_.target.jerk = fabs(pstate_msg_.target.jerk);
    }

  if (pstate_msg_.target.gear.value == art_msgs::Gear::Reverse)
    {
      if (pstate_msg_.target.speed > config_.limit_reverse)
        {
          ROS_WARN_STREAM_THROTTLE(100, "Requested speed ("
                                   << pstate_msg_.target.speed
                                   << ") exceeds reverse limit of "
                                   << config_.limit_reverse
                                   << " m/s");
          pstate_msg_.target.speed = config_.limit_reverse;
        }
    }
  else
    {
      if (pstate_msg_.target.speed > config_.limit_forward)
        {
          ROS_WARN_STREAM_THROTTLE(100, "Requested speed ("
                                   << pstate_msg_.target.speed
                                   << ") exceeds limit of "
                                   << config_.limit_forward
                                   << " m/s");
          pstate_msg_.target.speed = config_.limit_forward;
        }
    }

  // limit steering angle to permitted range
  using namespace pilot;
  using namespace art_msgs;
  pstate_msg_.target.steering_angle = clamp(-ArtVehicle::max_steer_radians,
                                            pstate_msg_.target.steering_angle,
                                            ArtVehicle::max_steer_radians);
}

/** main entry point */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pilot");
  ros::NodeHandle node;

  PilotNode pilot(node);
  pilot.spin();

  return 0;
}
