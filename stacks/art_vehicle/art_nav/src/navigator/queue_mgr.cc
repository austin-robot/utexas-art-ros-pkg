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

#include <art_servo/IOadrCommand.h>
#include <art_servo/IOadrState.h>
#include <art_servo/steering.h>
#include <art_map/ZoneOps.h>

#include <art_nav/Observers.h>

#include "navigator_internal.h"
#include "course.h"
//#include "obstacle.h"

/** @file

 @brief navigate the vehicle towards waypoint goals from commander

Use the commander node to control this driver.

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

  void PublishState(void);
  void SetSignals(void);
  void SetSpeed(pilot_command_t pcmd);

  // .cfg variables:
  int verbose;				// log level verbosity

  // ROS topics
  ros::Subscriber ctl_topic_;		// control device (pilot) topic
  ros::Subscriber map_topic_;           // road map topic

#if 0
  Device *intersection;			// intersection device
  player_devaddr_t intersection_addr;	// intersection device address
  bool have_intersection;

  Device *observers;			// observers device
  player_devaddr_t observers_addr;	// observers device address
  bool have_observers;
#endif

  // turn signal variables
  //AioServo *signals;
  bool signal_on_left;			// requested turn signal states
  bool signal_on_right;

  // latest order command
  art_nav::Order order_cmd;

  // navigator implementation class
  Navigator *nav;
};

// constructor, requesting overwrite of commands
NavQueueMgr::NavQueueMgr()
{
  // subscribe to control driver

  nav = new Navigator();
  nav->configure();
  
  //signals = new AioServo("signals", 0.0, verbose);
  signal_on_left = signal_on_right = false;
  

}

// Set up node message topics.
bool NavQueueMgr::setup(ros::NodeHandle node)
{   
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

#if 0 // read corresponding ROS topics instead
int NavQueueMgr::ProcessInput(player_msghdr *hdr, void *data)
{
  if (Message::MatchMessage(hdr,
			    PLAYER_MSGTYPE_CMD,
			    PLAYER_OPAQUE_CMD,
			    device_addr))
    {
      player_opaque_data_t *opaque = (player_opaque_data_t*) data;
      art_message_header_t *art_hdr = (art_message_header_t*) opaque->data;
 
      //ART_MSG(8, "Incoming opaque message: Type: %d Subtype: %d",
      //	      art_hdr->type,
      //	      art_hdr->subtype);

      if ((art_hdr->type == NAVIGATOR_MESSAGE)
	  && (art_hdr->subtype == NAVIGATOR_MESSAGE_ORDER))
	{
	  // TODO: this queue needs to handle multiple orders in one
	  // cycle.  The highest priority goes to PAUSE, RUN, and DONE,
	  // followed by the others.  Lower priority orders will be
	  // ignored.  They are resent in the following cycle.

	  // save message body
	  order_message_t *order_message =
	    (order_message_t*) (opaque->data + sizeof(art_message_header_t));
	  memcpy(&order_cmd, order_message, sizeof(order_message_t));
	  nav->order = order_cmd.od;

	  if (verbose >= 2)
	    {
	      ART_MSG(8, "OPAQUE_CMD(%s) received, speed range [%.3f, %.3f]"
		      " next_uturn %d",
		      nav->order.behavior.Name(),
		      nav->order.min_speed, nav->order.max_speed,
		      nav->order.next_uturn);
	      ART_MSG(8, "way0 = %s, way1 = %s, way2 = %s",
		      nav->order.waypt[0].id.name().str,
		      nav->order.waypt[1].id.name().str,
		      nav->order.waypt[2].id.name().str);
	    }
	}
      else if ((art_hdr->type == NAVIGATOR_MESSAGE)
	  && (art_hdr->subtype == NAVIGATOR_MESSAGE_ZONES)) {
	if (!nav->navdata.have_zones)
	  {
	    nav->course->zones = 
	      ZoneOps::unpackage_zone_list_from_opaque(*opaque);
	
	    nav->navdata.have_zones = true;

	    ART_MSG(1, 
		    "ZonePerimeterList received by Navigator from Commander:");
	    ART_MSG(1, "Number of Zones: %d", nav->course->zones.size());
	    for(unsigned i = 0; i < nav->course->zones.size(); i++) {
	      ART_MSG(1, 
		      "Zone ID: %d Perimeter: ", nav->course->zones[i].zone_id);
	      for(unsigned j = 0; 
		  j < nav->course->zones[i].perimeter_points.size(); j++) {
		ART_MSG(1, "(%.3f, %.3f), ",
			nav->course->zones[i].perimeter_points[j].map.x,
			nav->course->zones[i].perimeter_points[j].map.y);
	      }
	    }
	  }
      }
      else {
	ART_MSG(2, CLASS " unknown OPAQUE_CMD received");
	return -1;
      }
    }
  else if (Message::MatchMessage(hdr,
				 PLAYER_MSGTYPE_DATA,
				 PLAYER_POSITION2D_DATA_STATE,
				 ctl_addr))
    {}
  else if (nav->obstacle->lasers->match_lasers(hdr, data))
    {
      // tell observers about this later, after all laser scans queued
      // for this cycle have been processed
      ART_MSG(3, "NEW LASER RECEIVED");
    }
  else if (nav->odometry->match(hdr, data))
    {
      // update observers state pose
      ART_MSG(3, "NEW ODOMETRY RECEIVED");
      
    }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, 
				PLAYER_OPAQUE_DATA_STATE, 
				map_lanes_addr))
    {
      // This block handles lane messages and passes the data
      // over to navigator
      player_opaque_data_t *opaque = (player_opaque_data_t*) data;
      art_message_header_t *art_hdr = (art_message_header_t*) opaque->data;
      if((art_hdr->type == LANES_MESSAGE) &&
	 (art_hdr->subtype == LANES_MESSAGE_DATA_STATE) )
	{
	  lanes_state_msg_t* lanes = 
	    (lanes_state_msg_t*)(opaque->data + sizeof(art_message_header_t));

	  nav->course->lanes_message(lanes);
	}
      else
	{
	  ART_MSG(2, CLASS "unknown OPAQUE_DATA_STATE received from "
		  "map lanes driver");
	  return -1;
	}
    }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, 
				 PLAYER_OPAQUE_DATA_STATE, intersection_addr))
    {
      // handle intersection driver messages
      player_opaque_data_t *opaque = (player_opaque_data_t*) data;

      return nav->obstacle->intersection_message(hdr, opaque);
    }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, 
				 PLAYER_OPAQUE_DATA_STATE, observers_addr))
    {
      // handle observer messages
      player_opaque_data_t *opaque = (player_opaque_data_t*) data;
      art_message_header_t *art_hdr = (art_message_header_t*) opaque->data;
      if ((art_hdr->type == OBSERVERS_MESSAGE) &&
	  (art_hdr->subtype == OBSERVERS_MESSAGE_DATA_STATE))
	{
	  observers_state_msg_t* obs_msg = (observers_state_msg_t*)
	    (opaque->data + sizeof(art_message_header_t));

	  nav->obstacle->observers_message(hdr, obs_msg);
	}
      else
	{
	  ART_MSG(2, CLASS "unknown OPAQUE_DATA_STATE received from "
		  "observers driver");
	  return -1;
	}
    }
  else if (signals && signals->MatchInput(hdr, data))
    {
      player_aio_data_t *aio = (player_aio_data_t *) data;
      signals->position = rintf(aio->voltages[RelaysID]);
      if (verbose >= 6)
	ART_MSG(6, "turn signal relays %.f", signals->position);
#ifdef RELAY_FEEDBACK
      uint8_t relay_bits = (int) signals->position;
      signal_on_left = (relay_bits >> Relay_Turn_Left) & 0x01;
      signal_on_right = (relay_bits >> Relay_Turn_Right) & 0x01;
#endif
    }
  else					// some other message
    {
      ART_ERROR("unknown " CLASS " command: %d", hdr->subtype);
      return -1;
    }
  return 0;
}
#endif

/** Set or reset turn signal relays */
void NavQueueMgr::SetSignals(void)
{
#if 0
#ifdef RELAY_FEEDBACK

  if (signal_on_left != nav->navdata.signal_left)
    {
      if (signals)
	signals->SendCmd(nav->navdata.signal_left, Relay_Turn_Left);
      if (verbose)	
	ART_MSG(2, "setting left turn signal %s",
		(nav->navdata.signal_left? "on": "off"));
    }

  if (signal_on_right != nav->navdata.signal_right)
    {
      if (signals)
	signals->SendCmd(nav->navdata.signal_right, Relay_Turn_Right);
      if (verbose)	
	ART_MSG(2, "setting right turn signal %s",
		(nav->navdata.signal_right? "on": "off"));
    }

#else // RELAY_FEEDBACK

#ifdef SPAM_SIGNALS
      if (signals)
	signals->SendCmd(signal_on_left, Relay_Turn_Left);
#endif
  if (signal_on_left != nav->navdata.signal_left)
    {
      signal_on_left = nav->navdata.signal_left;
#ifndef SPAM_SIGNALS
      if (signals)
	signals->SendCmd(signal_on_left, Relay_Turn_Left);
#endif
      if (verbose)	
	ART_MSG(1, "setting left turn signal %s",
		(signal_on_left? "on": "off"));
    }

#ifdef SPAM_SIGNALS
      if (signals)
	signals->SendCmd(signal_on_right, Relay_Turn_Right);
#endif
  if (signal_on_right != nav->navdata.signal_right)
    {
      signal_on_right = nav->navdata.signal_right;
#ifndef SPAM_SIGNALS
      if (signals)
	signals->SendCmd(signal_on_right, Relay_Turn_Right);
#endif
      if (verbose)	
	ART_MSG(1, "setting right turn signal %s",
		(signal_on_right? "on": "off"));
    }
#endif // RELAY_FEEDBACK
#endif
}

/** Send a command to the pilot */
void NavQueueMgr::SetSpeed(pilot_command_t pcmd)
{
#if 0
#if 1
  player_position2d_cmd_car_t cmd;
  memset(&cmd, 0, sizeof(cmd));
  
  cmd.velocity = pcmd.velocity;

  float yawRate=pcmd.yawRate;

  if (cmd.velocity < 0)
    yawRate = -yawRate;		// steer in opposite direction
  cmd.angle = Steering::steering_angle(fabs(cmd.velocity), yawRate);
    
  if (verbose >= 2)
    ART_MSG(7, "Navigator CMD_CAR (%.3f m/s, %.3f degrees)",
	    cmd.velocity, cmd.angle);
  
  ctl->PutMsg(this->InQueue, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_CAR,
	      (void*) &cmd, sizeof(cmd), NULL);
#else
  player_position2d_cmd_vel_t cmd;
  memset(&cmd, 0, sizeof(cmd));
  
  cmd.vel.px =  pcmd.velocity;
  cmd.vel.py =  0;
  cmd.vel.pa =  pcmd.yawRate;
  
  if (verbose >= 2)
    ART_MSG(7, "Navigator CMD_VEL (%.3f m/s, %.3f rad/s)",
	    cmd.vel.px, cmd.vel.pa);
  
  ctl->PutMsg(this->InQueue, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL,
	      (void*) &cmd, sizeof(cmd), NULL);
#endif
#endif
}

// Publish current navigator state data
void NavQueueMgr::PublishState(void)
{
#if 0
  if (verbose >= 2)
    ART_MSG(7, "Publishing Navigator state = %s, %s, last_waypt %s"
	    ", replan_waypt %s, R%d S%d Z%d, next waypt %s, goal chkpt %s",
	    nav->navdata.estop_state.Name(),
	    nav->navdata.road_state.Name(),
	    nav->navdata.last_waypt.name().str,
	    nav->navdata.replan_waypt.name().str,
	    nav->navdata.reverse,
	    nav->navdata.stopped,
	    nav->navdata.have_zones,
	    nav->navdata.last_order.waypt[1].id.name().str,
	    nav->navdata.last_order.chkpt[0].id.name().str);

  // Publish this info for all subscribers
  Publish(device_addr,
	  PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_DATA_STATE,
	  (void*)&opaque, sizeof(opaque.data_count) + opaque.data_count);
#endif
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

      SetSignals();

      PublishState();

      // wait for next cycle
      cycle.sleep();
    }
}

/** Main program */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "commander");
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
