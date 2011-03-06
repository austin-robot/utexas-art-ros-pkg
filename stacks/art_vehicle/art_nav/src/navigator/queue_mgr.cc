/*
 * Queue manager front end for ROS navigator node.
 *
 *  Copyright (C) 2007, 2010 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <unistd.h>

#include <dynamic_reconfigure/server.h>

#include <art/frames.h>

#include <art_msgs/IOadrCommand.h>
#include <art_msgs/IOadrState.h>
#include <art/steering.h>
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
      delete nav_;
    };

  bool setup(ros::NodeHandle node);
  bool shutdown();
  void spin(void);

private:

  void processNavCmd(const art_msgs::NavigatorCommand::ConstPtr &cmdIn);
  void processOdom(const nav_msgs::Odometry::ConstPtr &odomIn);
  void processRoadMap(const art_msgs::ArtLanes::ConstPtr &cmdIn);
  void processRelays(const art_msgs::IOadrState::ConstPtr &sigIn);
  void PublishState(void);
  void reconfig(Config &newconfig, uint32_t level);
  void SetRelays(void);
  void SetSpeed(pilot_command_t pcmd);

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
  Navigator *nav_;

  // configuration callback
  dynamic_reconfigure::Server<Config> ccb_;
};

/** constructor */
NavQueueMgr::NavQueueMgr()
{
  signal_on_left_ = signal_on_right_ = false;
  flasher_on_ = alarm_on_ = false;

  // create control driver, declare dynamic reconfigure callback
  nav_ = new Navigator(&odom_msg_);
  ccb_.setCallback(boost::bind(&NavQueueMgr::reconfig, this, _1, _2));
}

/** Handle command input. */
void NavQueueMgr::processNavCmd(const
                                art_msgs::NavigatorCommand::ConstPtr &cmdIn)
{
  ROS_DEBUG_STREAM("Navigator order:"
                   << NavBehavior(cmdIn->order.behavior).Name());
  cmd_time_ = cmdIn->header.stamp;
  nav_->order = cmdIn->order;
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
  nav_->course->lanes_message(*mapIn);
}

/** Handle relays state message. */
void NavQueueMgr::processRelays(const art_msgs::IOadrState::ConstPtr &sigIn)
{
  using art_msgs::IOadrState;

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


/** handle dynamic reconfigure service request
 *
 * @param newconfig new configuration from dynamic reconfigure client,
 *        becomes the service reply message as updated here.
 * @param level SensorLevels value (0xffffffff on initial call)
 *
 * This is done without any locking because it is called in the same
 * thread as ros::spinOnce() and the topic subscription callbacks.
 */
void NavQueueMgr::reconfig(Config &newconfig, uint32_t level)
{
  ROS_INFO("navigator dynamic reconfigure, level 0x%x", level);
  nav_->config_ = newconfig;
  nav_->configure();
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
  signals_cmd_ = node.advertise<art_msgs::IOadrCommand>("ioadr/cmd", qDepth);

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
  nav_->obstacle->lasers->unsubscribe_lasers();
  nav_->odometry->unsubscribe();
#endif

  return 0;
}

/** Set or reset relays
 *
 *  @pre signal_on_left_, signal_on_right_, flasher_on_ and alarm_on_
 *       reflect the most recently reported states of those relays;
 *
 *  @pre nav_->navdata contains desired signal relay states
 */
void NavQueueMgr::SetRelays(void)
{
  if (signal_on_left_ != (bool) nav_->navdata.signal_left
      || signal_on_right_ != (bool) nav_->navdata.signal_right
      || flasher_on_ != (bool) nav_->navdata.flasher
      || alarm_on_ != (bool) nav_->navdata.alarm)
    {
      // something needs to change
      art_msgs::IOadrCommand sig_cmd;

      // set or reset left signal relay
      if (nav_->navdata.signal_left)
        sig_cmd.relays_on = art_msgs::IOadrState::TURN_LEFT;
      else
        sig_cmd.relays_off = art_msgs::IOadrState::TURN_LEFT;

      // or in right signal relay value
      if (nav_->navdata.signal_right)
        sig_cmd.relays_on |= art_msgs::IOadrState::TURN_RIGHT;
      else
        sig_cmd.relays_off |= art_msgs::IOadrState::TURN_RIGHT;

      // or in flasher relay value
      if (nav_->navdata.flasher)
        sig_cmd.relays_on |= art_msgs::IOadrState::FLASHER;
      else
        sig_cmd.relays_off |= art_msgs::IOadrState::FLASHER;

      // or in alarm relay value
      if (nav_->navdata.alarm)
        sig_cmd.relays_on |= art_msgs::IOadrState::ALARM;
      else
        sig_cmd.relays_off |= art_msgs::IOadrState::ALARM;

      ROS_DEBUG("setting relays: left turn %s, right turn %s, "
                "flasher %s, alarm %s",
                (nav_->navdata.signal_left? "on": "off"),
                (nav_->navdata.signal_right? "on": "off"),
                (nav_->navdata.flasher? "on": "off"),
                (nav_->navdata.alarm? "on": "off"));

      // send command to relay driver
      signals_cmd_.publish(sig_cmd);
    }
}

/** Send a command to the pilot */
void NavQueueMgr::SetSpeed(pilot_command_t pcmd)
{
  if (NavEstopState(nav_->navdata.estop) == NavEstopState::Suspend)
    {
      ROS_DEBUG_THROTTLE(20,
                         "Navigator suspended, not sending pilot commands.");
      return;
    }

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
  nav_->navdata.header.stamp = ros::Time::now();
  nav_->navdata.header.frame_id = ArtFrames::vehicle;

  ROS_DEBUG("Publishing Navigator state = %s, %s, last_waypt %s"
	    ", replan_waypt %s, R%d S%d Z%d, next waypt %s, goal chkpt %s",
	    NavEstopState(nav_->navdata.estop).Name(),
	    NavRoadState(nav_->navdata.road).Name(),
	    ElementID(nav_->navdata.last_waypt).name().str,
	    ElementID(nav_->navdata.replan_waypt).name().str,
	    (bool) nav_->navdata.reverse,
	    (bool) nav_->navdata.stopped,
	    (bool) nav_->navdata.have_zones,
	    ElementID(nav_->navdata.last_order.waypt[1].id).name().str,
	    ElementID(nav_->navdata.last_order.chkpt[0].id).name().str);

  // Publish this info for all subscribers
  nav_state_.publish(nav_->navdata);
}

/** Spin method for main thread */
void NavQueueMgr::spin() 
{
  ros::Rate cycle(art_msgs::ArtHertz::NAVIGATOR);
  while(ros::ok())
    {
      ros::spinOnce();                  // handle incoming messages

      // invoke appropriate Navigator method, pass result to Pilot
      SetSpeed(nav_->navigate());

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
