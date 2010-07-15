#!/usr/bin/env python

"""
/*
 *  Copyright (C) 2010, Austin Robot Technology
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
 
     \author Jack O'Quin, David Kraft-Ishihama

"""

# Most of the code in this file is meant to just be the C++ file
# except written in Python. I've tried to make most of it the same,
# though I made a few changes (for example, using generators where
# a function would ordinarily use static variables).

# TODO: Test this program using a simulator

import roslib
roslib.load_manifest('art_nav')

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from dynamic_reconfigure import server
from dynamic_reconfigure.msg import SensorLevels

from art_servo.msg import BrakeCommand
from art_servo.msg import BrakeState
from art_servo.msg import Shifter
from art_servo.msg import SteeringCommand
from art_servo.msg import SteeringState
from art_servo.msg import ThrottleCommand
from art_servo.msg import ThrottleState

#from art_common.msg import conversions
#from art_common.msg import epsilon
#from art_common.msg import hertz
#from art_common.msg import vehicle

#from art_servo import steering
#from art_common import pid2

from art_nav.msg import CarCommand
from art_nav.msg import CarControl
from art_nav.cfg import PilotConfig

#import speed

import sys

NODE = 'pilot'

"""
 @brief controls the ART vehicle brake, throttle, steering and transmission

The pilot receives CarCommand messages from the navigator, then
translates them into commands to the servo motor actuators for
controlling the speed and direction of the vehicle.  It gets odometry
information from a separate node.  For compatibility with other ROS
tools, it also responds to Twist messages on the cmd_vel topic.

Subscribes:

- \b pilot/cmd [art_nav.CarCommand] velocity and steering angle command
- \b vel_cmd [geometry_msgs.Twist] standard ROS velocity and angle command
- \b odom [nav_msgs.Odometry] estimate of robot position and velocity.

- \b brake/state [art_servo.BrakeState] brake status.
- \b shifter/state [art_servo.Shifter] shifter relays status.
- \b steering/state [art_servo.SteeringState] steering status.
- \b throttle/state [art_servo.ThrottleState] throttle status.

Publishes:

- \b brake/cmd [art_servo.BrakeCommand] brake commands.
- \b shifter/cmd [art_servo.Shifter] shifter commands.
- \b steering/cmd [art_servo.SteeringCommand] steering commands.
- \b throttle/cmd [art_servo.ThrottleCommand] throttle commands.

"""
 # global variables

# ROS topics used by this driver
car_cmd_ = None             # Subscriber: pilot command
twist_cmd_ = None           # Subscriber: Twist command
odom_state_ = None          # Subscriber: odometry

brake_state_ = None	    # Subscriber
shifter_state_ = None       # Subscriber
steering_state_ = None      # Subscriber
throttle_state_ = None      # Subscriber

brake_cmd_ = None           # Publisher: brake command
shifter_cmd_ = None         # Publisher: shifter command
steering_cmd_ = None        # Publisher: steering command
throttle_cmd_ = None        # Publisher: throttle command

  # configuration
config_ = PilotConfig       # Pilot config: dynamic configuration

  # servo control
brake_position_ = 1.0
shifter_gear_ = Shifter.Drive
steering_angle_ = 0.0
throttle_position_ = 0.0

brake_msg_ = BrakeCommand()
shifter_msg_ = Shifter()
steering_msg_ = SteeringCommand()
throttle_msg_ = ThrottleCommand()

shift_time_ = rospy.Time()  # time last shift requested

  # Odometry data
odom_msg_ = Odometry()

  # pilot command messages
goal_msg_ = CarControl()
goal_time_ = rospy.Time()   # time of last CarCommand
twist_msg_ = Twist()
speed_ = None

 # clamp value to range: [lower, upper]
def clamp (value, lower, upper) :
    return min(max(lower, value), upper)

def allocateSpeedControl() :
    # allocate appropriate speed control subclass for this configuration #
    #if (config_.use_accel_matrix) :
    #  rospy.loginfo("using acceleration matrix for speed control")
    #  speed_ = speed.SpeedControlMatrix()
    #else :
    #  rospy.loginfo("using brake and throttle PID for speed control")
    #  speed_ = speed.SpeedControlPID()

    rospy.loginfo("using brake and throttle PID for speed control")
    speed_ = speed.SpeedControlPID()

    # initialize brake and throttle positions in speed controller
    speed_.set_brake_position(brake_position_)
    speed_.set_throttle_position(throttle_position_)
    

def getParameters(argv) :
    # Allocate speed controller.  This needs to happen before setup()
    # starts reading topics.
    allocateSpeedControl()
    return 0

 # setGoal
def setGoal(command) :
    
    # rospy.logdebug("setting (velocity ,angle) to (%.3f, %.3f)", command.velocity, command.angle)

    if (goal_msg_.velocity != command.velocity) :
      #if (config_.maxspeed > 0 and command.velocity > config_.maxspeed) :
      #  rospy.logwarn("excessive speed of %.3f m/s requested", command.velocity)
      #  goal_msg_.velocity = config_.maxspeed
      #  
      #elif (config_.minspeed < 0 and command.velocity < config_.minspeed) :
      #  rospy_logwarn("excessive reverse speed of %.2f m/s requested",  command.velocity)
      #  goal_msg_.velocity = config_.minspeed
      #  
      #else :
      #  goal_msg_.velocity = command.velocity
        
      goal_msg_.velocity = command.velocity

      rospy.logdebug("changing speed goal from %.2f m/s to %.2f", goal_msg_.velocity, command.velocity)
    

    if (goal_msg_.angle != command.angle):
      rospy.logdebuf("changing steering angle from %.3f to %.3f (degrees)", goal_msg_.angle, command.angle)
      goal_msg_.angle = command.angle
       


 # processCommand
def processCommand(msg) :
    goal_time_ = msg.header.stamp
    rospy.logdebug("pilot command (v,a) = (%.3f, %.3f)", msg.control.velocity, msg.control.angle)
    car_ctl = msg.control
    setGoal(car_ctl)
    

 # This allows pilot to accept ROS cmd_vel messages
def processTwist(twistIn) :
    twist_msg_ = twistIn

    # convert to a CarControl message for setGoal()
    car_ctl = art_nav.CarControl()
    car_ctl.velocity = twistIn.linear.x
    car_ctl.angle = Steering.steering_angle(car_ctl.velocity, twistIn.angular.z)

    setGoal(car_ctl)
    

 # processOdom
def processOdom(odomIn) :
    rospy.logdebug("Odometry pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f)", odomIn.pose.pose.position.x,  odomIn.pose.pose.position.y, odomIn.pose.pose.position.z, odomIn.twist.twist.linear.x, odomIn.twist.twist.linear.y, odomIn.twist.twist.linear.z)
    odom_msg_ = odomIn
    rospy.logdebug("current velocity = %.3f m/sec, (%02.f mph)", odom_msg_.twist.twist.linear.x, mps2mph(odom_msg_.twist.twist.linear.x))
    

 # processBrake
def processBrake(brakeIn) :
    brake_position_ = brakeIn.position
    speed_.set_brake_position(brake_position_)
    rospy.logdebug("Brake reports position %.3f", brake_position_)

 # processThrottle
def processThrottle(throttleIn) :
    throttle_position_ = throttleIn.position
    speed_.set_throttle_position(throttle_position_)
    rospy.logdebug("Throttle reports position %.3f", throttle_position_)
    
 # processShifter
def processShifter(shifterIn) :
    shifter_gear_ = shifterIn.gear
    rospy.logdebug("Shifter reports gear %d", shifter_gear_)

 # processSteering
def processSteering(steeringIn) :
    steering_angle_ = steeringIn.angle
    rospy.logdebug("Steering reports angle %.1f (degrees)", steering_angle_)
    
 # handle dynamic reconfigure service request
 #
 # @param newconfig new configuration from dynamic reconfigure client,
 #        becomes the service reply message as updated here.
 # @param level SensorLevels value (0xffffffff on initial call)
 #
 # This is done without any locking because I believe it is called in
 # the same main thread as ros::spinOnce() and all the topic
 # subscription callbacks. If not, we need a lock.
 
def reconfig(newconfig, level) :
    rospy.loginfo("pilot dynamic reconfigure, level 0x%x", level)

    # need to reallocate speed controller when use_accel_matrix changes
    #realloc = (newconfig.use_accel_matrix != config_.use_accel_matrix)
    #
    #config_ = newconfig
    #
    #if (realloc) :
    #  allocateSpeedControl()

 # Adjust velocity to match goal.
 #
 #  cur_speed	absolute value of current velocity in m/sec
 #  speed_delta	difference between that and our immediate goal
def adjustVelocity(cur_speed, error) :
    
    throttle_msg_.position, brake_msg_.position = speed_.adjust(cur_speed, error, throttle_msg_.position, brake_msg_.position)

    brake_msg_.position = clamp(brake_msg_.position, 0.0, 1.0)
    if (fabsf(brake_msg_.position - brake_position_) > EPSILON_BRAKE) :
    # Do I need to replace fabsf(x) with math.fabs(x)?
      brake_msg_.header.stamp = rospy.Time.now()
      brake_cmd_.publish(brake_msg_)
    

    throttle_msg_.position = clamp(throttle_msg_.position, 0.0, 1.0)
    if (fabsf(throttle_msg_.position - throttle_position_) > EPSILON_THROTTLE) :
    # Do I need to replace fabsf(x) with math.fabs(x)?
      throttle_msg_.header.stamp = rospy.Time.now()
      throttle_cmd_.publish(throttle_msg_)
    

 # Halt -- soft version of hardware E-Stop.
 #
 #  The goal is to bring the vehicle to a halt as quickly as possible,
 #  while remaining safely under control.  Normally, navigator sends
 #  gradually declining speed requests when bringing the vehicle to a
 #  controlled stop.  The only abrupt requests we see are in
 #  "emergency" stop situations, when there was a pause request, or no
 #  clear path around an obstacle.
 #
 #  cur_speed	absolute value of current velocity in m/sec
 #
def Halt(cur_speed) :
  # At high speed use adjustVelocity() to slow the vehicle down some
  # before slamming on the brakes.  Even with ABS to avoid lock-up,
  # it seems safer to bring the vehicle to a more gradual stop.
    
    SAFE_FULL_BRAKE_SPEED = mph2mps(45) # 45 mph (in m/sec)
    if (cur_speed > SAFE_FULL_BRAKE_SPEED) :
      adjustVelocity(cur_speed, -cur_speed)
      return
    
    elif (cur_speed < Epsilon.speed) :
      # Already stopped.  Ease up on the brake to reduce strain on
      # the actuator.  Brake hold position *must* be adequate to
      # prevent motion, even on a hill.
      #
      # TODO: detect motion after stopping and apply more brake.
      brake_msg_.header.stamp = rospy.Time.now()
      #brake_msg_.position = config_.brake_hold
      brake_msg_.position = 0.7
      brake_cmd_.publish(brake_msg_)
    else :
      # Stop the moving vehicle very quickly.
      #
      #  Apply min throttle and max brake at the same time.  This is
      #  an exception to the general rule of never applying brake and
      #  throttle together.  There seems to be enough inertia in the
      #  brake mechanism for this to be safe.
      now = rospy.Time.now()
      throttle_msg_.header.stamp = now
      throttle_msg_.position = 0.0
      throttle_cmd_.publish(throttle_msg_)
      brake_msg_.header.stamp = now
      brake_msg_.position = 1.0
      brake_cmd_.publish(brake_msg_)
    


 # Adjust steering angle.
 #
 # We do not use PID control, because the odometry does not provide
 # accurate yaw speed feedback.  Instead, we directly compute the
 # corresponding steering angle.  We can use open loop control at this
 # level, because navigator monitors our actual course and will
 # request any steering changes needed to reach its goal.
 #
def adjustSteering() :
    cur_degrees = 360.0     # (an impossible value)
    # Since cur_degrees was originally intended to be a static
    # variable and Python has no concept of static variables,
    # I've turned this function into a generator.
    while True :
      # Set the steering angle in degrees.
      if (cur_degrees != goal_msg_.angle) :
        rospy.logdebug("requesting steering angle = %.1f (degrees)", goal_msg_.angle)
        steering_msg_.header.stamp = rospy.Time.now()
        steering_msg_.angle = goal_msg_.angle
        steering_cmd_.publish(steering_msg_)
        cur_degrees = goal_msg_.angle
      yield
 
# These assignments are meant to replace the enumeration used in the
# C++ version of Pilot.
Stopped = 0
Forward = 1
Backward = 2   

 # speed_range
def speed_range(speed) :
    if (speed > Epsilon.speed):		# moving forward?
      return Forward

    if (speed >= -Epsilon.speed):	# close to zero?
      return Stopped

    return Backward

# Transmission shifting states
# The assignments here are meant to replace the enumeration in the
# C++ version.

Drive		= 0x01		# Transmission in Drive
Reverse		= 0x02		# Transmission in Reverse
ShiftDrive	= 0x10		# Shifting into Drive
ShiftReverse	= 0x20		# Shifting into Reverse

 # Speed control
 #
 #  Manage the shifter as a finite state machine.  Inputs are the
 #  current and goal velocity ranges (+,0,-).  If these velocities
 #  differ in sign, the vehicle must first be brought to a stop, then
 #  one of the transmission shift relays set for one second, before
 #  the vehicle can begin moving in the opposite direction.
 #
def speedControl(speed) :
  shift_duration = 1.0 # hold relay one second
  shifting_state = Drive
  # Note: Python has no concept of static variables.
  #       To get static variable behavior out of this function,
  #       I have turned it into a generator.
  while True :
    goal = goal_msg_.velocity       # goal velocity
    error = goal - speed

    cur_range = speed_range(speed)
    goal_range = speed_range(goal)

    rospy.logdebug("Shifting state: 0x%02x, speed: %.3f m/s, goal: %.3f", shifting_state, speed, goal)

    if shifting_state == Drive :
      # TODO: make sure shifter relays are off now
      if (Forward == goal_range) :
	adjustVelocity(speed, error)
      elif (Stopped == cur_range and Backward == goal_range) :
	shifting_state = ShiftReverse
        shifter_msg_.header.stamp = rospy.Time.now()
        shifter_msg_.gear = Shifter.Reverse
        shifter_cmd_.publish(shifter_msg_)
        shift_time_ = rospy.Time.now()
      else :
	Halt(speed)
        speed_.reset()
    elif shifting_state == shiftReverse :
      # make sure the transmission actually shifted
      if (shifter_gear_ != Shifter.Reverse) :
	# repeat shift command until it works
        shifter_msg_.header.stamp = rospy.Time.now()
        shifter_msg_.gear = Shifter.Reverse
        shifter_cmd_.publish(shifter_msg_)
        shift_time_ = rospy.Time.now()
        tospy.logdebug("repeated shift command at %.6f", shift_time_.toSec())
      # make sure the relay was set long enough
      elif ((rospy.Time.now().toSec() - shift_time_.toSec()) >= shift_duration) :
	shifter_msg_.header.stamp = rospy.Time.now();
        shifter_msg_.gear = Shifter.Reset
        shifter_cmd_.publish(shifter_msg_)
	if (Backward == goal_range) :
	  shifting_state = Reverse
	  adjustVelocity(-speed, -error)
	elif (Stopped == goal_range) :
	  shifting_state = Reverse
	  Halt(-speed)
	  speed_.reset()
	else : # Dang!  we want to go forward now
	  shifting_state = ShiftDrive
          shifter_msg_.header.stamp = rospy.Time.now()
          shifter_msg_.gear = Shifter.Drive
          shifter_cmd_.publish(shifter_msg_)
	  shift_time_ = rospy.Time.now()

    elif shifting_state == Reverse :
      # TODO: make sure shifter relays are off now
      if (Backward == goal_range) :
        adjustVelocity(-speed, -error)
      elif (Stopped == cur_range and Forward == goal_range) :
	shifting_state = ShiftDrive
        shifter_msg_.header.stamp = rospy.Time.now()
        shifter_msg_.gear = Shifter.Drive
        shifter_cmd_.publish(shifter_msg_)
	shift_time_ = rospy.Time.now()
      else :
	Halt(-speed)
	speed_.reset()

    elif shifting_state == ShiftDrive :
      # make sure the transmission actually shifted
      if (shifter_gear_ != Shifter.Drive) :
	# repeat shift command until it works
        shifter_msg_.header.stamp = rospy.Time.now();
        shifter_msg_.gear = Shifter.Drive
        shifter_cmd_.publish(shifter_msg_)
	shift_time_ = rospy.Time.now()
        rospy.logdebug("repeated shift command at %.6f", shift_time_.toSec())
	
      # make sure the relay was set long enough
      elif ((rospy.Time.now().toSec() - shift_time_.toSec()) >= shift_duration) :
	shifter_msg_.header.stamp = rospy.Time.now()
        shifter_msg_.gear = Shifter.Reset
        shifter_cmd_.publish(shifter_msg_)
	if (Forward == goal_range) :
	  shifting_state = Drive
	  adjustVelocity(speed, error)
	elif (Stopped == goal_range) :
	  shifting_state = Drive
	  Halt(speed)
	  speed_.reset()
	else : # Dang!  we want to go backward now
	  shifting_state = ShiftReverse
          shifter_msg_.header.stamp = rospy.Time.now()
          shifter_msg_.gear = Shifter.Reverse
          shifter_cmd_.publish(shifter_msg_)
	  shift_time_ = rospy.Time.now()
    yield

 # setup
def setup() :
  # topics to read
  car_cmd_ = rospy.Subscriber("pilot/cmd", CarCommand, processCommand, tcp_nodelay=True)
  twist_cmd_ = rospy.Subscriber("cmd_vel", Twist, processTwist, tcp_nodelay=True)
  odom_state_ = rospy.Subscriber("odom", Odometry, processOdom, tcp_nodelay=True)
  brake_state_ = rospy.Subscriber("brake/state", BrakeState, processBrake, tcp_nodelay=True)
  shifter_state_ = rospy.Subscriber("shifter/state", Shifter, processShifter, tcp_nodelay=True)
  steering_state_ = rospy.Subscriber("steering/state", SteeringState, processSteering, tcp_nodelay=True)
  throttle_state_ = rospy.Subscriber("throttle/state", ThrottleState, processThrottle, tcp_nodelay=True)
  
  # initialize servo command interfaces and messages
  brake_cmd_ = rospy.Publisher("brake/cmd", BrakeCommand)
  brake_msg_.header.frame_id = ArtVehicle.frame_id
  brake_msg_.request = art_servo.BrakeCommand.Absolute
  brake_msg_.position = 1.0

  shifter_cmd_ = rospy.Publisher("shifter/cmd", Shifter)
  shifter_msg_.header.frame_id = ArtVehicle.frame_id

  steering_cmd_ = rospy.Publisher("steering/cmd", SteeringCommand)
  steering_msg_.header.frame_id = ArtVehicle.frame_id
  steering_msg_.request = art_servo.SteeringCommand.Degrees

  throttle_cmd_ = rospy.Publisher("throttle/cmd", ThrottleCommand)
  throttle_msg_.header.frame_id = ArtVehicle.frame_id
  throttle_msg_.request = art_servo.ThrottleCommand.Absolute
  throttle_msg_.position = 0.0
  
  return 0

 # main
def main(argv) :
    # The main function is the part I'm most unsure about.
    # Does the structure need to change?
    rospy.init_node(NODE)

    if ( getParameters(argv) != 0 ) : return 1

    # I'm not sure how to do this part in Python
    """
    # declare dynamic reconfigure callback
    dynamic_reconfigure::Server<art_nav::PilotConfig> srv;
    dynamic_reconfigure::Server<art_nav::PilotConfig>::CallbackType cb =  boost::bind(&reconfig, _1, _2);
    srv.setCallback(cb);
    """
    if (setup(node) != 0) : return 2

    cycle = rospy.Rate(HERTZ_PILOT)        # set driver cycle rate

    # Main loop
    while not rospy.is_shutdown():
      # Rospy doesn't seem to have a spinOnce() function. What
      # should I do instead? Does it not need anything here?
      #ros::spinOnce();               # handle incoming commands

      speed_.configure()              # check for parameter updates

      # issue control commands
      speedControl(odom_msg_.twist.twist.linear.x)
      adjustSteering()

      cycle.sleep()                   # sleep until next cycle

    return 0

if __name__ == '__main__':
    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.ROSInterruptException: pass
