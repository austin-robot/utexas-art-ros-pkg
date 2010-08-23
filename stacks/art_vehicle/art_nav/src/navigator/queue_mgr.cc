/*
 * Queue manager front end for ROS navigator node.
 *
 *  Copyright (C) 2005, 2010 Austin Robot Technology
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
#include "obstacle.h"

/** @file

 @brief navigate the vehicle towards waypoint goals from commander

Use the commander node to control this driver.

@author Jack O'Quin
*/

#define CLASS "NavQueueMgr"

// The class for the driver
class NavQueueMgr : public Driver
{
public:
    
  // Constructor
  NavQueueMgr(ConfigFile* cf, int section);
  ~NavQueueMgr()
    {
      delete nav;
      delete cycle;
      if (signals) delete signals;
      if (opaque.data != NULL)
	delete [] opaque.data;
    };

  int Setup();
  int Shutdown();

  // method invoked on each incoming message
  int ProcessMessage(QueuePointer& resp_queue, 
		     player_msghdr * hdr, void * data);

private:

  int  Configure(QueuePointer resp_queue,
		 player_msghdr *hdr,
		 void *data);
  void Main(void);			// main device thread function
  int  ProcessInput(player_msghdr *hdr, void *data);
  void PublishState(void);
  void SetSignals(void);
  void SetSpeed(pilot_command_t pcmd);
  void EnableMotor();

  // .cfg variables:
  int verbose;				// log level verbosity
  Cycle *cycle;				// driver cycle class

  // control interface
  Device *ctl;				// control device
  player_devaddr_t ctl_addr;		// control device address
  bool have_ctl;


  Device *map_lanes;			//map lanes device
  player_devaddr_t map_lanes_addr;	//map lanes device address
  bool have_map_lanes;

  Device *intersection;			// intersection device
  player_devaddr_t intersection_addr;	// intersection device address
  bool have_intersection;

  Device *observers;			// observers device
  player_devaddr_t observers_addr;	// observers device address
  bool have_observers;

  // turn signal variables
  AioServo *signals;
  bool signal_on_left;			// requested turn signal states
  bool signal_on_right;

  // latest order command
  order_message_t order_cmd;

  player_opaque_data_t opaque;

  // navigator implementation class
  Navigator* nav;
};

Driver* NavQueueMgr_Init(ConfigFile* cf, int section)
{
  // Create and return a new instance of this driver
  return((Driver*)(new NavQueueMgr(cf, section)));
}

void NavQueueMgr_Register(DriverTable* table)
{
  table->AddDriver("navigator", NavQueueMgr_Init);
}

// constructor, requesting overwrite of commands
NavQueueMgr::NavQueueMgr(ConfigFile* cf, int section)
    : Driver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, 
             PLAYER_OPAQUE_CODE)
{
  // TODO: allow queuing of opaque Order messages, but nothing else
  //  //InQueue->SetPull(true);
  InQueue->SetReplace(true);

  opaque.data_count = (sizeof(art_message_header_t)
		       + sizeof(nav_state_msg_t));
  opaque.data = new uint8_t[opaque.data_count];

  // set debug output verbosity, cycle rate
  verbose = cf->ReadInt(section, "verbose", 0);
  PLAYER_MSG1(2, CLASS " constructor: verbose level %d", verbose);
  cycle = new Cycle("navigator", verbose);
  cycle->Configure(cf, section, HERTZ_NAVIGATOR);

  // subscribe to control driver
  have_ctl = (0 == cf->ReadDeviceAddr(&ctl_addr, section, "requires",
				      PLAYER_POSITION2D_CODE, -1, "control"));
  if (have_ctl)
    ART_MSG(1, "using control:::position2d:%d", ctl_addr.index);

  nav = new Navigator(verbose, cycle);
  nav->configure(cf, section);
  
  have_map_lanes = 
    (0 == cf->ReadDeviceAddr(&map_lanes_addr, section, "requires",
			     PLAYER_OPAQUE_CODE, -1, "maplanes"));
  if (have_map_lanes)
    ART_MSG(1, "using maplanes:::opaque:%d", map_lanes_addr.index);
  else
    {
      ART_ERROR("Required maplanes:::opaque interface missing!");
      SetError(-1);
      return;
    }

  have_intersection = 
    (0 == cf->ReadDeviceAddr(&intersection_addr, section, "requires",
			     PLAYER_OPAQUE_CODE, -1, "intersection"));
  if (have_intersection)
    ART_MSG(1, "using intersection:::opaque:%d", intersection_addr.index);

  have_observers = 
    (0 == cf->ReadDeviceAddr(&observers_addr, section, "requires",
			     PLAYER_OPAQUE_CODE, -1, "observers"));
  if (have_observers)
	ART_MSG(1, "using observers:::opaque:%d", observers_addr.index);

  signals = new AioServo("signals", 0.0, verbose);
  if (0 != signals->CfgRequires(cf, section)) // no turn signal device
    {
      ART_MSG(4, "no turn signal interface available");
      delete signals;
      signals = NULL;
    }
  signal_on_left = signal_on_right = false;
  

}

// Set up the device.
int NavQueueMgr::Setup()
{   
  ctl = NULL;
  if (have_ctl)
    {
      // get the pointer to the odometry
      if (verbose)
	ART_MSG(2, "finding control:::position2d:%d", ctl_addr.index);
      ctl = deviceTable->GetDevice(ctl_addr);
      if (ctl == NULL)
	{
	  ART_ERROR("unable to find control driver");
	  return -1;
	}
      if (ctl->Subscribe(InQueue) != 0)
	{
	  ART_ERROR("unable to subscribe to control driver");
	  return -1;
	}
      if (verbose)
	ART_MSG(2, CLASS ": subscribed to control.");
    }
  else					// control missing
    {
      ART_ERROR(CLASS " Required control interface not provided!\n");
      return -1;
    }

  if (nav->odometry->have_odom)
    {
      if (0 != nav->odometry->subscribe(InQueue))
	return -1;
    }
  else ART_MSG(2, CLASS " No odometry interface, using control instead.\n");
  
  nav->obstacle->lasers->subscribe_lasers(InQueue);  
  
  map_lanes = NULL;
  if(have_map_lanes)
  {
	if(verbose)
		ART_MSG(2, "finding maplanes:::opaque:%d..",
			map_lanes_addr.index);
	map_lanes = deviceTable->GetDevice(map_lanes_addr);
	if(map_lanes == NULL)
	  {
		ART_ERROR("unable to find map lane driver");
		return -1;
	  }
	if(map_lanes->Subscribe(InQueue) != 0)
	  {
		ART_ERROR("unable to subscribe to map lanes");
		return -1;
	  }
	if(verbose)
		ART_MSG(2, CLASS ": subscribed to map lanes");
  }
  
  intersection = NULL;
  if(have_intersection)
    {
      if(verbose)
	ART_MSG(2, "finding intersection:::opaque:%d..",
		intersection_addr.index);
      intersection = deviceTable->GetDevice(intersection_addr);
      if(intersection == NULL)
	{
	  ART_ERROR("unable to find intersection driver");
	  return -1;
	}
      if(intersection->Subscribe(InQueue) != 0)
	{
	  ART_ERROR("unable to subscribe to intersection");
	  return -1;
	}
      if(verbose)
	ART_MSG(2, CLASS ": subscribed to intersection");
  }
  
  observers = NULL;
  if(have_observers)
    {
      if(verbose)
	ART_MSG(2, "finding observers:::opaque:%d..",
		observers_addr.index);
      observers = deviceTable->GetDevice(observers_addr);
      if(observers == NULL)
	{
	  ART_ERROR("unable to find observers driver");
	  return -1;
	}
      if(observers->Subscribe(InQueue) != 0)
	{
	  ART_ERROR("unable to subscribe to observers");
	  return -1;
	}
      if(verbose)
	ART_MSG(2, CLASS ": subscribed to observers");
  }

  if (signals && 0 != signals->Subscribe(InQueue))
    return -1;
  
  StartThread();
  return 0;
}


// Shutdown the device
int NavQueueMgr::Shutdown()
{
  // Stop and join the driver thread
  ART_MSG(1, CLASS " Shutting down Navigator");
  StopThread();

  cycle->End();

  // issue immediate halt command to pilot
  pilot_command_t cmd;
  cmd.velocity = 0.0;
  cmd.yawRate = 0.0;
  SetSpeed(cmd);

  nav->obstacle->lasers->unsubscribe_lasers();


  if (ctl)
    ctl->Unsubscribe(InQueue);

  nav->odometry->unsubscribe();

  if (have_map_lanes)
    map_lanes->Unsubscribe(InQueue);

  if (have_intersection)
    intersection->Unsubscribe(InQueue);

  if (have_observers)
    observers->Unsubscribe(InQueue);

  if (signals)
    signals->Unsubscribe();

  return 0;
}

int NavQueueMgr::ProcessMessage(QueuePointer& resp_queue, 
                                  player_msghdr * hdr,
                                  void * data)
{
  if (verbose >= 2)
    ART_MSG(4, CLASS "::ProcessMessage() type: (%d, %d) [time %.6f]",
	    hdr->type, hdr->subtype, cycle->Time());

  // Look for configuration requests
  if (hdr->type == PLAYER_MSGTYPE_RESP_ACK) 
    return 0;

  if (hdr->type == PLAYER_MSGTYPE_REQ)
    return Configure(resp_queue, hdr, data);
  else
    return ProcessInput(hdr, data);

  ART_ERROR("unknown " CLASS " message type: %d", hdr->type);
  return -1;
}  

int NavQueueMgr::Configure(QueuePointer resp_queue,
		    player_msghdr *hdr,
		    void *data)
{
  ART_ERROR("unknown " CLASS " config request: %d", hdr->subtype);
  return -1;
}

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

// Set or reset turn signal relays
void NavQueueMgr::SetSignals(void)
{
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
}

// Send a command to the underlying position2d control device
void NavQueueMgr::SetSpeed(pilot_command_t pcmd)
{

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
}

void NavQueueMgr::EnableMotor() {

  player_position2d_power_config_t power_config;
  power_config.state=true;
  
  ctl->PutMsg(this->InQueue, PLAYER_MSGTYPE_REQ, 
	      PLAYER_POSITION2D_REQ_MOTOR_POWER,
	      (void*) &power_config, sizeof(power_config), NULL);
}


// Publish current navigator state data
void NavQueueMgr::PublishState(void)
{
  // format opaque message

  art_message_header_t msghdr;
  msghdr.type = NAVIGATOR_MESSAGE;
  msghdr.subtype = NAVIGATOR_MESSAGE_STATE_DATA;


  memcpy(opaque.data, &msghdr, sizeof(msghdr));
  memcpy(opaque.data+sizeof(msghdr), &nav->navdata, sizeof(nav->navdata));

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

}

// Main function for device thread
void NavQueueMgr::Main() 
{
  cycle->Start();

  EnableMotor();

  for(;;)
    {
      // see if we are supposed to exit
      pthread_testcancel();

      // Process incoming messages.  NavQueueMgr::ProcessMessage() is
      // called on each message.
      ProcessMessages();

      // invoke appropriate Navigator method, pass result to Pilot
      SetSpeed(nav->navigate());

      SetSignals();

      PublishState();

      // wait for next cycle
      cycle->Wait();
    }
}

// Extra stuff for building a shared object.
extern "C" {
  int player_driver_init(DriverTable* table)
  {
    NavQueueMgr_Register(table);
    return 0;
  }
}
