/*
 * Queue manager front end for ROS navigator node.
 *
 *  Copyright (C) 2007, 2010 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

// define this to send turn signal messages based on driver feedback
#define RELAY_FEEDBACK 1

// define this to send turn signal messages on every cycle
#define SPAM_SIGNALS 1

#include <unistd.h>

#include <art/frames.h>

#include <art_servo/IOadrCommand.h>
#include <art_servo/IOadrState.h>
#include <art_servo/steering.h>
#include <art_map/ZoneOps.h>

#include <art_msgs/CarCommand.h>
#include <art_nav/NavEstopState.h>
#include <art_nav/NavRoadState.h>
//#include <art_msgs/Observers.h>

#include "navigator_internal.h"
#include "course.h"
//#include "obstacle.h"

/** @file

  @brief navigate the vehicle towards way-point goals

  Use the commander node to control this node.

  @todo Add Observers interface.

  @author Jack O'Quin
*/

#define CLASS "NavQueueMgr"

// The class for the driver
class NavQueueMgr
{
public:
    
  // Constructor
  NavQueueMgr();
  ~NavQueueMgr()
    {
      delete nav;
    };

  bool setup(ros::NodeHandle node);
  bool shutdown();
  void spin(void);

private:

  void processNavCmd(const art_msgs::NavigatorCommand::ConstPtr &cmdIn);
  void processOdom(const nav_msgs::Odometry::ConstPtr &odomIn);
  void processRoadMap(const art_msgs::ArtLanes::ConstPtr &cmdIn);
  void processRelays(const art_servo::IOadrState::ConstPtr &sigIn);
  void PublishState(void);
  void SetRelays(void);
  void SetSpeed(pilot_command_t pcmd);

  // .cfg variables:
  int verbose;				// log level verbosity

  // ROS topics
  ros::Publisher  car_cmd_;             // pilot CarCommand
  ros::Subscriber nav_cmd_;             // NavigatorCommand topic
  ros::Publisher  nav_state_;           // navigator state topic
  ros::Subscriber odom_state_;          // odometry
  ros::Subscriber roadmap_;             // local road map polygons
  ros::Publisher  signals_cmd_;
  ros::Subscriber signals_state_;

  //ros::Subscriber observers_;

  // relay variables
  bool signal_on_left_;			// reported turn signal states
  bool signal_on_right_;
  bool flasher_on_;
  bool alarm_on_;

  // Odometry data
  nav_msgs::Odometry odom_msg_;

  // time stamps of latest messages received
  ros::Time cmd_time_;
  ros::Time map_time_;

  // navigator implementation class
  Navigator *nav;
};

// constructor, requesting overwrite of commands
NavQueueMgr::NavQueueMgr()
{
  // create control driver
  nav = new Navigator(&odom_msg_);
  nav->configure();
  
  signal_on_left_ = signal_on_right_ = false;
  flasher_on_ = alarm_on_ = false;
}

/** Handle command input. */
void NavQueueMgr::processNavCmd(const
                                art_msgs::NavigatorCommand::ConstPtr &cmdIn)
{
  ROS_DEBUG_STREAM("Navigator order:"
                   << NavBehavior(cmdIn->order.behavior).Name());
  cmd_time_ = cmdIn->header.stamp;
  nav->order = cmdIn->order;
}

/** Handle Odometry input. */
void NavQueueMgr::processOdom(const nav_msgs::Odometry::ConstPtr &odomIn)
{
  ROS_DEBUG("Odometry pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f)",
            odomIn->pose.pose.position.x,
            odomIn->pose.pose.position.y,
            odomIn->pose.pose.position.z,
            odomIn->twist.twist.linear.x,
            odomIn->twist.twist.linear.y,
            odomIn->twist.twist.linear.z);

  float vel = odomIn->twist.twist.linear.x;
  ROS_DEBUG("current velocity = %.3f m/sec, (%02.f mph)", vel, mps2mph(vel));
  odom_msg_ = *odomIn;
}

/** Handle road map polygons. */
void NavQueueMgr::processRoadMap(const art_msgs::ArtLanes::ConstPtr &mapIn)
{
  ROS_DEBUG_STREAM(mapIn->polygons.size() << " lanes polygons received");
  map_time_ = mapIn->header.stamp;
  nav->course->lanes_message(*mapIn);
}

/** Handle relays state message. */
void NavQueueMgr::processRelays(const art_servo::IOadrState::ConstPtr &sigIn)
{
  using art_servo::IOadrState;

  bool left_on = (sigIn->relays && IOadrState::TURN_LEFT);
  if (left_on != signal_on_left_)
    {
      ROS_INFO("left turn signal now %s", (left_on? "on": "off"));
      signal_on_left_ = left_on;
    }

  bool right_on = (sigIn->relays && IOadrState::TURN_RIGHT);
  if (right_on != signal_on_right_)
    {
      ROS_INFO("right turn signal now %s", (right_on? "on": "off"));
      signal_on_right_ = right_on;
    }

  bool flasher_on = (sigIn->relays && IOadrState::FLASHER);
  if (flasher_on != flasher_on_)
    {
      ROS_INFO("flasher now %s", (flasher_on? "on": "off"));
      flasher_on_ = flasher_on;
    }

  bool alarm_on = (sigIn->relays && IOadrState::ALARM);
  if (alarm_on != alarm_on_)
    {
      ROS_INFO("alarm now %s", (alarm_on? "on": "off"));
      alarm_on_ = alarm_on;
    }
}

/** Set up ROS topics for navigator node */
bool NavQueueMgr::setup(ros::NodeHandle node)
{
  // no delay: we always want the most recent data
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  static uint32_t qDepth = 1;

  // topics to read
  odom_state_ = node.subscribe("odom", qDepth,
                               &NavQueueMgr::processOdom, this, noDelay);
  nav_cmd_ = node.subscribe("navigator/cmd", qDepth,
                            &NavQueueMgr::processNavCmd, this, noDelay);
  roadmap_ = node.subscribe("roadmap_local", qDepth,
                            &NavQueueMgr::processRoadMap, this, noDelay);
  signals_state_ = node.subscribe("ioadr/state", qDepth,
                                  &NavQueueMgr::processRelays, this, noDelay);

  // topics to write
  car_cmd_ = node.advertise<art_msgs::CarCommand>("pilot/cmd", qDepth);
  nav_state_ =
    node.advertise<art_msgs::NavigatorState>("navigator/state", qDepth);
  signals_cmd_ = node.advertise<art_servo::IOadrCommand>("ioadr/cmd", qDepth);

  return true;
}

// Shutdown the node
bool NavQueueMgr::shutdown()
{
  ROS_INFO("Shutting down navigator node");

  // issue immediate halt command to pilot
  pilot_command_t cmd;
  cmd.velocity = 0.0;
  cmd.yawRate = 0.0;
  SetSpeed(cmd);

#if 0
  nav->obstacle->lasers->unsubscribe_lasers();
  nav->odometry->unsubscribe();
#endif

  return 0;
}

/** Set or reset relays
 *
 *  @pre signal_on_left_, signal_on_right_, flasher_on_ and alarm_on_
 *       reflect the most recently reported states of those relays;
 *
 *  @pre nav->navdata contains desired signal relay states
 */
void NavQueueMgr::SetRelays(void)
{
  if (signal_on_left_ != (bool) nav->navdata.signal_left
      || signal_on_right_ != (bool) nav->navdata.signal_right
      || flasher_on_ != (bool) nav->navdata.flasher
      || alarm_on_ != (bool) nav->navdata.alarm)
    {
      // something needs to change
      art_servo::IOadrCommand sig_cmd;

      // set or reset left signal relay
      if (nav->navdata.signal_left)
        sig_cmd.relays_on = art_servo::IOadrState::TURN_LEFT;
      else
        sig_cmd.relays_off = art_servo::IOadrState::TURN_LEFT;

      // or in right signal relay value
      if (nav->navdata.signal_right)
        sig_cmd.relays_on |= art_servo::IOadrState::TURN_RIGHT;
      else
        sig_cmd.relays_off |= art_servo::IOadrState::TURN_RIGHT;

      // or in flasher relay value
      if (nav->navdata.flasher)
        sig_cmd.relays_on |= art_servo::IOadrState::FLASHER;
      else
        sig_cmd.relays_off |= art_servo::IOadrState::FLASHER;

      // or in alarm relay value
      if (nav->navdata.alarm)
        sig_cmd.relays_on |= art_servo::IOadrState::ALARM;
      else
        sig_cmd.relays_off |= art_servo::IOadrState::ALARM;

      ROS_DEBUG("setting relays: left turn %s, right turn %s, "
                "flasher %s, alarm %s",
                (nav->navdata.signal_left? "on": "off"),
                (nav->navdata.signal_right? "on": "off"),
                (nav->navdata.flasher? "on": "off"),
                (nav->navdata.alarm? "on": "off"));

      // send command to relay driver
      signals_cmd_.publish(sig_cmd);
    }
}

/** Send a command to the pilot */
void NavQueueMgr::SetSpeed(pilot_command_t pcmd)
{
  art_msgs::CarCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = ArtFrames::vehicle;

  cmd.control.velocity = pcmd.velocity;
  float yawRate = pcmd.yawRate;
  if (pcmd.velocity < 0)
    yawRate = -yawRate;                 // steer in opposite direction
  cmd.control.angle =
    Steering::steering_angle(fabs(pcmd.velocity), yawRate);
    
  ROS_DEBUG("Navigator CMD_CAR (%.3f m/s, %.3f degrees)",
	    cmd.control.velocity, cmd.control.angle);
  
  car_cmd_.publish(cmd);
}

/** Publish current navigator state data */
void NavQueueMgr::PublishState(void)
{
  nav->navdata.header.stamp = ros::Time::now();
  nav->navdata.header.frame_id = ArtFrames::vehicle;

  ROS_DEBUG("Publishing Navigator state = %s, %s, last_waypt %s"
	    ", replan_waypt %s, R%d S%d Z%d, next waypt %s, goal chkpt %s",
	    NavEstopState(nav->navdata.estop).Name(),
	    NavRoadState(nav->navdata.road).Name(),
	    ElementID(nav->navdata.last_waypt).name().str,
	    ElementID(nav->navdata.replan_waypt).name().str,
	    (bool) nav->navdata.reverse,
	    (bool) nav->navdata.stopped,
	    (bool) nav->navdata.have_zones,
	    ElementID(nav->navdata.last_order.waypt[1].id).name().str,
	    ElementID(nav->navdata.last_order.chkpt[0].id).name().str);

  // Publish this info for all subscribers
  nav_state_.publish(nav->navdata);
}

/** Spin method for main thread */
void NavQueueMgr::spin() 
{
  ros::Rate cycle(art_common::ArtHertz::NAVIGATOR);
  while(ros::ok())
    {
      ros::spinOnce();                  // handle incoming messages

      // invoke appropriate Navigator method, pass result to Pilot
      SetSpeed(nav->navigate());

      SetRelays();

      PublishState();

      // wait for next cycle
      cycle.sleep();
    }
}

/** Main program */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigator");
  ros::NodeHandle node;
  NavQueueMgr nq;
  
  // set up ROS topics
  if (!nq.setup(node))
    return 1;

  // keep running until mission completed
  nq.spin();

  // shut down node
  if (!nq.shutdown())
    return 3;

  return 0;
}
