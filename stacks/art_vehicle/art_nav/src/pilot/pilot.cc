/*
 *  Copyright (C) 2005, 2007, 2009 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  \file
 
     ROS node for controlling direction and speed of the ART
     autonomous vehicle.

     \todo check that devices are responding, (optionally) stop if
     they are not

     \todo (optionally) stop if no commands received recently.

     \todo provide state feedback

     \todo shift to Park, when appropriate
 
     \author Jack O'Quin

 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>

#include <art_msgs/ArtVehicle.h>
#include <art_msgs/ArtHertz.h>
#include <art_msgs/CarCommand.h>

#include <art_servo/BrakeCommand.h>
#include <art_servo/BrakeState.h>
#include <art_servo/Shifter.h>
#include <art_servo/SteeringCommand.h>
#include <art_servo/SteeringState.h>
#include <art_servo/ThrottleCommand.h>
#include <art_servo/ThrottleState.h>

#include <art/conversions.h>
#include <art/epsilon.h>
#include <art/pid2.h>
#include <art/steering.h>

#include <art_nav/PilotConfig.h>

#include <art_msgs/LearningCommand.h>

#include "learned_controller.h"
#include "speed.h"

#define NODE "pilot"

/**
 @brief controls the ART vehicle brake, throttle, steering and transmission

The pilot receives CarCommand messages from the navigator, then
translates them into commands to the servo motor actuators for
controlling the speed and direction of the vehicle.  It gets odometry
information from a separate node.  For compatibility with other ROS
tools, it also responds to Twist messages on the cmd_vel topic.

Subscribes:

- \b pilot/cmd [art_msgs::CarCommand] velocity and steering angle command
- \b vel_cmd [geometry_msgs::Twist] standard ROS velocity and angle command
- \b odom [nav_msgs::Odometry] estimate of robot position and velocity.

- \b brake/state [art_servo::BrakeState] brake status.
- \b shifter/state [art_servo::Shifter] shifter relays status.
- \b steering/state [art_servo::SteeringState] steering status.
- \b throttle/state [art_servo::ThrottleState] throttle status.

Publishes:

- \b brake/cmd [art_servo::BrakeCommand] brake commands.
- \b shifter/cmd [art_servo::Shifter] shifter commands.
- \b steering/cmd [art_servo::SteeringCommand] steering commands.
- \b throttle/cmd [art_servo::ThrottleCommand] throttle commands.

*/

// global variables
namespace
{

  // ROS topics used by this driver
  ros::Subscriber car_cmd_;             // pilot command
  ros::Subscriber twist_cmd_;           // Twist command
  ros::Subscriber odom_state_;          // odometry

  ros::Subscriber brake_state_;
  ros::Subscriber shifter_state_;
  ros::Subscriber steering_state_;
  ros::Subscriber throttle_state_;

  ros::Subscriber learning_state_;

  ros::Publisher brake_cmd_;            // brake command
  ros::Publisher shifter_cmd_;          // shifter command
  ros::Publisher steering_cmd_;         // steering command
  ros::Publisher throttle_cmd_;         // throttle command

  // configuration
  art_nav::PilotConfig config_;         // dynamic configuration

  // servo control
  float brake_position_ = 1.0;
  uint8_t shifter_gear_ = art_servo::Shifter::Drive;
  float steering_angle_ = 0.0;
  float throttle_position_ = 0.0;

  art_servo::BrakeCommand    brake_msg_;
  art_servo::Shifter         shifter_msg_;
  art_servo::SteeringCommand steering_msg_;
  art_servo::ThrottleCommand throttle_msg_;

  ros::Time shift_time_;                // time last shift requested

  // Odometry data
  nav_msgs::Odometry odom_msg_;

  // pilot command messages
  art_msgs::CarControl goal_msg_;
  art_msgs::LearningCommand current_learning_cmd_;
  
  ros::Time goal_time_;                 // time of last CarCommand
  geometry_msgs::Twist twist_msg_;

  SpeedControl *speed_ = NULL;          // speed control
};

// clamp value to range: [lower, upper]
static inline float clamp(float value, float lower, float upper)
{
  return (value > upper? upper: (value < lower? lower: value));
}

/** allocate appropriate speed control subclass for this configuration */
void allocateSpeedControl(void)
{

  if (config_.use_learned_controller) 
    {
      ROS_INFO("using RL learned controller for speed control");
      speed_ = new LearnedSpeedControl();
    }
  else if (config_.use_accel_matrix)
    {
      ROS_INFO("using acceleration matrix for speed control");
      speed_ = new SpeedControlMatrix();
    }
  else
    {
      ROS_INFO("using brake and throttle PID for speed control");
      speed_ = new SpeedControlPID();
    }

  // initialize brake and throttle positions in speed controller
  speed_->set_brake_position(brake_position_);
  speed_->set_throttle_position(throttle_position_);
}

/** deallocate speed control subclass */
void deallocateSpeedControl(void)
{
  if (speed_)
    {
      ROS_DEBUG("stopping pilot speed control");
      delete speed_;
      speed_ = NULL;
    }
}

/** get node parameters */
int getParameters(int argc, char *argv[])
{
  // use private node handle to get parameters
  ros::NodeHandle private_nh("~");

  // Allocate speed controller.  This needs to happen before setup()
  // starts reading topics.
  allocateSpeedControl();

  return 0;
}

void setGoal(const art_msgs::CarControl *command)
{
  //ROS_DEBUG("setting (velocity ,angle) to (%.3f, %.3f)",
  //          command->velocity, command->angle);

  if (goal_msg_.velocity != command->velocity)
    {
      ROS_DEBUG("changing speed goal from %.2f m/s to %.2f",
                goal_msg_.velocity, command->velocity);

      if (config_.maxspeed > 0 && command->velocity > config_.maxspeed)
        {
          ROS_WARN("excessive speed of %.2f m/s requested", command->velocity);
          goal_msg_.velocity = config_.maxspeed;
        }
      else if (config_.minspeed < 0 && command->velocity < config_.minspeed)
        {
          ROS_WARN("excessive reverse speed of %.2f m/s requested",
                   command->velocity);
          goal_msg_.velocity = config_.minspeed;
        }
      else
        {
          goal_msg_.velocity = command->velocity;
        }
    }

  if (goal_msg_.angle != command->angle)
    {
      ROS_DEBUG("changing steering angle from %.3f to %.3f (degrees)",
                goal_msg_.angle, command->angle);
      goal_msg_.angle = command->angle;
    }
}

void processCommand(const art_msgs::CarCommand::ConstPtr &msg)
{
  goal_time_ = msg->header.stamp;
  ROS_DEBUG("pilot command (v,a) = (%.3f, %.3f)",
            msg->control.velocity, msg->control.angle);
  art_msgs::CarControl car_ctl = msg->control;
  setGoal(&car_ctl);
}

// This allows pilot to accept ROS cmd_vel messages
void processTwist(const geometry_msgs::Twist::ConstPtr &twistIn)
{
  twist_msg_ = *twistIn;

  // convert to a CarControl message for setGoal()
  art_msgs::CarControl car_ctl;
  car_ctl.velocity = twistIn->linear.x;
  car_ctl.angle = Steering::steering_angle(car_ctl.velocity, twistIn->angular.z);

  setGoal(&car_ctl);
}

void processOdom(const nav_msgs::Odometry::ConstPtr &odomIn)
{
  ROS_DEBUG("Odometry pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f)",
            odomIn->pose.pose.position.x,
            odomIn->pose.pose.position.y,
            odomIn->pose.pose.position.z,
            odomIn->twist.twist.linear.x,
            odomIn->twist.twist.linear.y,
            odomIn->twist.twist.linear.z);

  odom_msg_ = *odomIn;

  ROS_DEBUG("current velocity = %.3f m/sec, (%02.f mph)",
            odom_msg_.twist.twist.linear.x,
            mps2mph(odom_msg_.twist.twist.linear.x));
}

void processBrake(const art_servo::BrakeState::ConstPtr &brakeIn)
{
  brake_position_ = brakeIn->position;
  speed_->set_brake_position(brake_position_);
  ROS_DEBUG("Brake reports position %.3f", brake_position_);
}

void processThrottle(const art_servo::ThrottleState::ConstPtr &throttleIn)
{
  throttle_position_ = throttleIn->position;
  speed_->set_throttle_position(throttle_position_);
  ROS_DEBUG("Throttle reports position %.3f", throttle_position_);
}

void processShifter(const art_servo::Shifter::ConstPtr &shifterIn)
{
  shifter_gear_ = shifterIn->gear;
  ROS_DEBUG("Shifter reports gear %d", shifter_gear_);
}

void processSteering(const art_servo::SteeringState::ConstPtr &steeringIn)
{
  steering_angle_ = steeringIn->angle;
  ROS_DEBUG("Steering reports angle %.1f (degrees)", steering_angle_);
}


void processLearning(const art_msgs::LearningCommand::ConstPtr &learningIn)
{
  current_learning_cmd_.pilotActive = learningIn->pilotActive;
  ROS_INFO("Pilot Active set to %d", current_learning_cmd_.pilotActive);
}


/** handle dynamic reconfigure service request
 *
 * @param newconfig new configuration from dynamic reconfigure client,
 *        becomes the service reply message as updated here.
 * @param level SensorLevels value (0xffffffff on initial call)
 *
 * This is done without any locking because I believe it is called in
 * the same main thread as ros::spinOnce() and all the topic
 * subscription callbacks. If not, we need a lock.
 */
void reconfig(art_nav::PilotConfig &newconfig, uint32_t level)
{
  ROS_INFO("pilot dynamic reconfigure, level 0x%x", level);

  // need to reallocate speed controller when use_accel_matrix changes
  // or use_learned_controller changes
  bool realloc = (newconfig.use_accel_matrix != config_.use_accel_matrix ||
                  newconfig.use_learned_controller != config_.use_learned_controller);

  if (realloc)
    {
      deallocateSpeedControl();
    }

  config_ = newconfig;

  if (realloc)
    {
      allocateSpeedControl();
    }
}

// Adjust velocity to match goal.
//
//  cur_speed	absolute value of current velocity in m/sec
//  speed_delta	difference between that and our immediate goal
void adjustVelocity(float cur_speed, float error)
{
  // Adjust brake and throttle settings.
  speed_->adjust(cur_speed, error,
                 &brake_msg_.position, &throttle_msg_.position);

  brake_msg_.position = clamp(brake_msg_.position, 0.0, 1.0);
  if (fabsf(brake_msg_.position - brake_position_) > EPSILON_BRAKE)
    {
      brake_msg_.header.stamp = ros::Time::now();
      brake_cmd_.publish(brake_msg_);
    }

  throttle_msg_.position = clamp(throttle_msg_.position, 0.0, 1.0);
  if (fabsf(throttle_msg_.position - throttle_position_) > EPSILON_THROTTLE)
    {
      throttle_msg_.header.stamp = ros::Time::now();
      throttle_cmd_.publish(throttle_msg_);
    }
}

// Halt -- soft version of hardware E-Stop.
//
//  The goal is to bring the vehicle to a halt as quickly as possible,
//  while remaining safely under control.  Normally, navigator sends
//  gradually declining speed requests when bringing the vehicle to a
//  controlled stop.  The only abrupt requests we see are in
//  "emergency" stop situations, when there was a pause request, or no
//  clear path around an obstacle.
//
//  cur_speed	absolute value of current velocity in m/sec
//
void Halt(float cur_speed)
{
  // At high speed use adjustVelocity() to slow the vehicle down some
  // before slamming on the brakes.  Even with ABS to avoid lock-up,
  // it seems safer to bring the vehicle to a more gradual stop.

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
      brake_msg_.header.stamp = ros::Time::now();
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
      ros::Time now = ros::Time::now();
      throttle_msg_.header.stamp = now;
      throttle_msg_.position = 0.0;
      throttle_cmd_.publish(throttle_msg_);
      brake_msg_.header.stamp = now;
      brake_msg_.position = 1.0;
      brake_cmd_.publish(brake_msg_);
    }
}

// Adjust steering angle.
//
// We do not use PID control, because the odometry does not provide
// accurate yaw speed feedback.  Instead, we directly compute the
// corresponding steering angle.  We can use open loop control at this
// level, because navigator monitors our actual course and will
// request any steering changes needed to reach its goal.
//
void adjustSteering()
{
  static float cur_degrees = 360.0;     // (an impossible value)

  // Set the steering angle in degrees.
  if (cur_degrees != goal_msg_.angle)
    {
      ROS_DEBUG("requesting steering angle = %.1f (degrees)", goal_msg_.angle);
      steering_msg_.header.stamp = ros::Time::now();
      steering_msg_.angle = goal_msg_.angle;
      steering_cmd_.publish(steering_msg_);
      cur_degrees = goal_msg_.angle;
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

// Speed control
//
//  Manage the shifter as a finite state machine.  Inputs are the
//  current and goal velocity ranges (+,0,-).  If these velocities
//  differ in sign, the vehicle must first be brought to a stop, then
//  one of the transmission shift relays set for one second, before
//  the vehicle can begin moving in the opposite direction.
//
void speedControl(float speed)
{
  // If the learning needs to bypass pilot then return here
  if (!current_learning_cmd_.pilotActive)
    return;
  
  static const double shift_duration = 1.0; // hold relay one second
  static Transmission::state_t shifting_state = Transmission::Drive;

  float goal = goal_msg_.velocity;       // goal velocity
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
          shifter_msg_.header.stamp = ros::Time::now();
          shifter_msg_.gear = art_servo::Shifter::Reverse;
          shifter_cmd_.publish(shifter_msg_);
          shift_time_ = ros::Time::now();
	}
      else
	{
	  Halt(speed);
          speed_->reset();
	}
      break;

    case Transmission::ShiftReverse:
      // make sure the transmission actually shifted
      if (shifter_gear_ != art_servo::Shifter::Reverse)
	{
	  // repeat shift command until it works
          shifter_msg_.header.stamp = ros::Time::now();
          shifter_msg_.gear = art_servo::Shifter::Reverse;
          shifter_cmd_.publish(shifter_msg_);
	  shift_time_ = ros::Time::now();
          ROS_DEBUG("repeated shift command at %.6f", shift_time_.toSec());
	}
      // make sure the relay was set long enough
      else if ((ros::Time::now().toSec() - shift_time_.toSec())
               >= shift_duration)
	{
          shifter_msg_.header.stamp = ros::Time::now();
          shifter_msg_.gear = art_servo::Shifter::Reset;
          shifter_cmd_.publish(shifter_msg_);
	  if (Backward == goal_range)
	    {
	      shifting_state = Transmission::Reverse;
	      adjustVelocity(-speed, -error);
	    }
	  else if (Stopped == goal_range)
	    {
	      shifting_state = Transmission::Reverse;
	      Halt(-speed);
	      speed_->reset();
	    }
	  else // Dang!  we want to go forward now
	    {
	      shifting_state = Transmission::ShiftDrive;
              shifter_msg_.header.stamp = ros::Time::now();
              shifter_msg_.gear = art_servo::Shifter::Drive;
              shifter_cmd_.publish(shifter_msg_);
	      shift_time_ = ros::Time::now();
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
          shifter_msg_.header.stamp = ros::Time::now();
          shifter_msg_.gear = art_servo::Shifter::Drive;
          shifter_cmd_.publish(shifter_msg_);
	  shift_time_ = ros::Time::now();
	}
      else
	{
	  Halt(-speed);
	  speed_->reset();
	}
      break;

    case Transmission::ShiftDrive:
      // make sure the transmission actually shifted
      if (shifter_gear_ != art_servo::Shifter::Drive)
	{
	  // repeat shift command until it works
          shifter_msg_.header.stamp = ros::Time::now();
          shifter_msg_.gear = art_servo::Shifter::Drive;
          shifter_cmd_.publish(shifter_msg_);
	  shift_time_ = ros::Time::now();
          ROS_DEBUG("repeated shift command at %.6f", shift_time_.toSec());
	}
      // make sure the relay was set long enough
      else if ((ros::Time::now().toSec() - shift_time_.toSec())
               >= shift_duration)
	{
          shifter_msg_.header.stamp = ros::Time::now();
          shifter_msg_.gear = art_servo::Shifter::Reset;
          shifter_cmd_.publish(shifter_msg_);
	  if (Forward == goal_range)
	    {
	      shifting_state = Transmission::Drive;
	      adjustVelocity(speed, error);
	    }
	  else if (Stopped == goal_range)
	    {
	      shifting_state = Transmission::Drive;
	      Halt(speed);
	      speed_->reset();
	    }
	  else // Dang!  we want to go backward now
	    {
	      shifting_state = Transmission::ShiftReverse;
              shifter_msg_.header.stamp = ros::Time::now();
              shifter_msg_.gear = art_servo::Shifter::Reverse;
              shifter_cmd_.publish(shifter_msg_);
	      shift_time_ = ros::Time::now();
	    }
	}
      break;
    }
}

int setup(ros::NodeHandle node)
{
  static int qDepth = 1;

  // no delay: we always want the most recent data
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  // topics to read
  car_cmd_ = node.subscribe("pilot/cmd", qDepth, processCommand, noDelay);
  twist_cmd_ = node.subscribe("cmd_vel", qDepth, processTwist, noDelay);
  odom_state_ = node.subscribe("odom", qDepth, processOdom, noDelay);

  brake_state_ = node.subscribe("brake/state", qDepth, processBrake, noDelay);
  shifter_state_ = node.subscribe("shifter/state", qDepth,
                                  processShifter, noDelay);
  steering_state_ = node.subscribe("steering/state", qDepth,
                                   processSteering, noDelay);
  throttle_state_ = node.subscribe("throttle/state", qDepth,
                                   processThrottle, noDelay);
  learning_state_ = node.subscribe("pilot/learningCmd", qDepth,
                                   processLearning, noDelay);

  
  // initialize servo command interfaces and messages
  brake_cmd_ = node.advertise<art_servo::BrakeCommand>("brake/cmd", qDepth);
  brake_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;
  brake_msg_.request = art_servo::BrakeCommand::Absolute;
  brake_msg_.position = 1.0;

  shifter_cmd_ = node.advertise<art_servo::Shifter>("shifter/cmd", qDepth);
  shifter_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;

  steering_cmd_ =
    node.advertise<art_servo::SteeringCommand>("steering/cmd", qDepth);
  steering_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;
  steering_msg_.request = art_servo::SteeringCommand::Degrees;

  throttle_cmd_ =
    node.advertise<art_servo::ThrottleCommand>("throttle/cmd", qDepth);
  throttle_msg_.header.frame_id = art_msgs::ArtVehicle::frame_id;
  throttle_msg_.request = art_servo::ThrottleCommand::Absolute;
  throttle_msg_.position = 0.0;

  //
  current_learning_cmd_.pilotActive = true;
  
  return 0;
}

void shutdown(void)
{
  deallocateSpeedControl();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  if (getParameters(argc, argv) != 0)
    {
      shutdown();
      return 1;
    }

  // declare dynamic reconfigure callback
  dynamic_reconfigure::Server<art_nav::PilotConfig> srv;
  dynamic_reconfigure::Server<art_nav::PilotConfig>::CallbackType cb =
    boost::bind(&reconfig, _1, _2);
  srv.setCallback(cb);

  if (setup(node) != 0)
    {
      shutdown();
      return 2;
    }
 
  // Main loop
  ros::Rate cycle(art_msgs::ArtHertz::PILOT); // set driver cycle rate
  while(ros::ok())
    {
      ros::spinOnce();                  // handle incoming commands

      speed_->configure();              // check for parameter updates

      // issue control commands
      speedControl(odom_msg_.twist.twist.linear.x);
      adjustSteering();

      cycle.sleep();                    // sleep until next cycle
    }

  shutdown();

  return 0;
}
