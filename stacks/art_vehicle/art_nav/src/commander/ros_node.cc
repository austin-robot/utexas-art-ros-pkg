/*
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 *
 *  Author:  Patrick Beeson, Jack O'Quin
 */

// Commander node ROS front end

#include <iostream>

#include <ros/ros.h>

#include <art/hertz.h>
#include <art_map/ZoneOps.h>
#include <applanix/applanix_info.h>

using namespace applanix_info;          // defines gps_info

#include <art_nav/NavEstopState.h>
#include <art_nav/NavRoadState.h>

#include "command.h"

/** @brief ART vehicle commander client

    @section Synopsis

    commander [options] RNDF MDF

    @section Description

    The commander reads the Route Network Definition File (<b>RNDF</b>)
    and Mission Data File (<b>MDF</b>).  It selects a route for
    accomplishing the assigned mission, and passes appropriate waypoint
    following orders to the @ref interface_navigator of the @ref
    driver_navigator.

    Normally, the navigator driver waits for a <b>Run</b> command from the
    @ref client_diag before actively pursuing its mission.  As a
    convenience when testing in simulation, commander provides a <b>-r</b>
    option which causes the vehicle to start running immediately without
    the requiring the diagnostic controller.

    @section Options

    @verbatim
    -h <host>	player host name (default "localhost")
    -m <speed>	maximum vehicle speed in m/s (default 7.5)
    -p <port>	player host port (default 6665)
    -q		quiet (no verbose messages)
    -r		run vehicle immediately
    -s		start anywhere, even outside RNDF
    -v        	verbose messages (-vv for more)
    -?		print usage message
    @endverbatim

    @section Examples

    @verbatim
    $ commander example.rndf example.mdf
    @endverbatim

    Connect to the robot vehicle at "localhost" port 6665.  Send
    navigation commands to "opaque:0".  Path names for the rndf and
    mdf files are required.

    @verbatim
    $ commander -r example.rndf example.mdf
    @endverbatim

    Start running the robot vehicle immediately.

    @verbatim
    $ commander -d2 -h remotehost -i3 example.rndf example.mdf
    @endverbatim

    Connect to the robot vehicle at "remotehost" port 6665.  Send
    navigation commands to "opaque:3".  Produce verbose debug output.

    @author Patrick Beeson, Jack O'Quin
*/
/** @} */

//const  uint data_size=sizeof(art_message_header_t) + sizeof(nav_state_msg_t);
//uint8_t data[data_size];

art_nav::NavigatorState navdata;

#if 0 // comment out Player interfaces: will replace with ROS versions
// proably should move to NavigatorProxy()
art_nav::NavigatorState get_nav_state(PlayerCc::OpaqueProxy *nav)
{
  
  if (nav->GetCount() != data_size)
    {
      // error in message size
      ROS_INFO_STREAM(10) << "ERROR: Navigator message size is incorrect\n";
    }
  
  // get data packet
  nav->GetData(data);
  memcpy(&hdr, data, sizeof(art_message_header_t));
  memcpy(&navdata, data+sizeof(art_message_header_t), sizeof(art_nav::NavigatorState));
  if (hdr.type != NAVIGATOR_MESSAGE
      || hdr.subtype != NAVIGATOR_MESSAGE_STATE_DATA)
    {
      // wrong message type
      ROS_INFO_STREAM(10) << 
	"ERROR: Execute received the wrong message type from Navigator\n";
    }
  return navdata;
}

gps_info get_gps_state(PlayerCc::OpaqueProxy *gps) {
  if (gps->GetCount() > sizeof(gps_info)) {
    // error in message size
    ROS_INFO_STREAM(10) << "ERROR: GPS opqaue message size is incorrect\n";
  }
  
  gps_info pos;
  
  // get data packet
  gps->GetData((uint8_t*) &pos);
  
  return pos;
}


#endif

class CommanderNode
{
public:
  CommanderNode()
  {
    mission_file_ = "mission_state";
    load_mission_=false;
    startrun_ = false;
    verbose_ = 1;
    speedlimit_ = 7.5;
    //speedlimit_ = 5.5;			// for Area A test

    rndf_ = NULL;
    mdf_ = NULL;
    graph_ = NULL;
    mission_ = NULL;
  }

  ~CommanderNode()
  {
    if (rndf_ != NULL)
      delete rndf_;
    if (mdf_ != NULL)
      delete mdf_;
    if (graph_ != NULL)
      delete graph_;
    if (mission_ != NULL)
      delete mission_;
  }

  bool parse_args(int argc, char** argv)
  {

    // set the flags
    const char* optflags = "h:l:p:m:rv?";
    int ch;
    std::cerr<<"\n";

    // use getopt to parse the flags
    while(-1 != (ch = getopt(argc, argv, optflags)))
      {
	switch(ch)
	  {
	  case 'l':				// hostname
	    mission_file_ = optarg;
	    load_mission_=true;
	    break;
	  case 'r':				// run
	    startrun_ = true;
	    break;
	  case 'm': 
	    speedlimit_ = atof(optarg);
	    break;
	  case 'v':				// extra verbosity
	    verbose_++;
	      break;
	  case '?': // help
	  default:  // unknown
	    print_usage(argc, argv);
	    return false;
	  }
      }

    if (speedlimit_ < 0)
      {
	std::cerr << "ERROR: Maximum speed must be >= 0" << std::endl;
	return false;
      }


    if (optind >= argc-1)
      {
	std::cerr << "ERROR: RNDF or MDF names missing. \nOR they were read "
		  << "as values to another option with a missing argument.\n" 
		  << std::endl;
	print_usage(argc, argv);
	return false;
      }
  
    rndf_name_ = argv[optind++];  

    mdf_name_ = argv[optind++];

    if (optind < argc)
      {
	std::cerr << "ERROR: invalid extra parameter: " 
		  << argv[optind] << std::endl;
	print_usage(argc, argv);
	return false;
      }

    return true;
  }

#if 0 // replace with ROS version
  bool initialize_player()
  {
    
    // we throw exceptions on creation if we fail
    bool connected=false;

    while (!connected)
      try {
	robot = new PlayerCc::PlayerClient(_hostname, _port);
	connected=true;
      }
      catch (PlayerCc::PlayerError e) {
	//std::cerr << "Waiting to for Player connection" << std::endl;
	sleep(3);
      }

    robot->SetReplaceRule(true);
    robot->SetDataMode(PLAYER_DATAMODE_PULL);
    
    connected=false;
    while (!connected)
      try {
	nav = new PlayerCc::OpaqueProxy(robot, 0);
	connected=true;
      }
      catch (PlayerCc::PlayerError e) {
	//std::cerr << "Waiting to for Navigator interface" << std::endl;
	sleep(3);
      }

    connected=false;
    while (!connected)
      try {
	maplane = new ArtProxy::LanesProxy(robot);
	connected=true;
      }
      catch (PlayerCc::PlayerError e) {
	//std::cerr << "Waiting to for MapLanes interface" << std::endl;
	sleep(3);
      }

    connected=false;
    while (!connected)
      try {
	gps = new PlayerCc::OpaqueProxy(robot, 99);
	connected=true;
      }
      catch (PlayerCc::PlayerError e) {
	//std::cerr << "Waiting to for initial GPS interface" << std::endl;
	sleep(3);
      }

    connected=false;
    while (!connected)
      try {
	pos2d = new PlayerCc::Position2dProxy(robot, 0);
	connected=true;
      }
      catch (PlayerCc::PlayerError e) {
	//std::cerr << "Waiting for initial Position2d interface" << std::endl;
	sleep(3);
      }

    while (!gps->IsValid() || !pos2d->IsValid()) {
      ROS_INFO_STREAM(10) << "Waiting for initial lat/long info\n";
      try {
	robot->Read();
      }
      catch (PlayerCc::PlayerError e) {
	ROS_INFO_STREAM(10) << "Cannot read initial gps info from odometry...stopping\n";
	return false;
      }
    }
    
    gps_info pos = get_gps_state(gps);
    
    if (!graph_->rndf_is_gps())
      {
	ROS_INFO_STREAM(10) << 
	  "RNDF seems too large to be composed of GPS waypoints...stopping\n";
	return false;	
      }
    
    graph_->find_mapxy(pos, pos2d->GetXPos(), pos2d->GetYPos());
    maplane->SendRNDF(graph_);
    
    graph_->find_implicit_edges();

    while (!nav->IsValid()) {
      ROS_INFO_STREAM(10) << "Waiting for navigator to publish info\n";
      try {
	robot->Read();
      }
      catch (PlayerCc::PlayerError e) {
	ROS_INFO_STREAM(10) << "Cannot read initial info from navigator...stopping\n";
	return false;
      }
    }
    
    zones_ = ZoneOps::build_zone_list_from_rndf(*rndf_, *graph_);    

    while (true)
      {
	// this blocks until new data comes; nominally 10Hz by default
	try { 
	  robot->Read();
	}
	catch (PlayerCc::PlayerError e) {
	  ROS_INFO_STREAM(10) << "Cannot read info from Player...stopping\n";
	  return false;
	}    
	
	art_nav::NavigatorState navState = get_nav_state(nav);
	
	if(navState.have_zones) 
	  {
	    ROS_INFO_STREAM(2) << "Navigator has received zones.\n";
	    break;
	  }
      }
    
    return true;    
  }
#endif // old Player code

  bool build_RNDF()
  {

    rndf_ = new RNDF(rndf_name_);
  
    if (!rndf_->is_valid)
      {
        ROS_FATAL("RNDF not valid");
        return false;;
      }

    mdf_ = new MDF(mdf_name_);

    if (!mdf_->is_valid)
      {
        ROS_FATAL("MDF not valid");
        return false;;
      }

    graph_ = new Graph();
    rndf_->populate_graph(*graph_);
    
    mdf_->add_speed_limits(*graph_);
    mission_ = new Mission(*mdf_);

    if (load_mission_) 
      {
	mission_->clear();
	if (!mission_->load(mission_file_.c_str(), *graph_))
	  {
	    ROS_FATAL_STREAM("Unable to load stored mission file, "
                             <<mission_file_<<" is missing or corrupt");
	    return false;
	  }
	ROS_INFO_STREAM("Loaded stored mission from "<<mission_file_);
      }
    else
      {
	if (!mission_->populate_elementid(*graph_))
	  {
	    ROS_FATAL("Mission IDs not same size as Element IDs");
	    return false;
	  }
	ROS_INFO("Running full mission from MDF");
      }
    
    
    if (mission_->remaining_points() < 1) {
      ROS_FATAL("No checkpoints left");
      return false;
    }

    return true;
  }

  
  /** send order to navigator driver */
  void putOrder(art_nav::Order order)
  {
  }

  /** main spin loop */
  bool spin()
  {
    if (startrun_)
      {
        ROS_INFO("ordering navigator to RUN");
        art_nav::Order run_order;
        run_order.behavior.value = NavBehavior::Run;
        putOrder(run_order);
      }

    // initialize Commander class
    Commander commander(verbose_, speedlimit_, graph_, mission_, zones_);

    ros::Rate cycle(HERTZ_COMMANDER);

    // loop until end of mission
    while(ros::ok())
      {
        ROS_DEBUG_STREAM("navstate = "
                         << NavEstopState(navState_.estop).Name()
                         << ", " << NavRoadState(navState_.road).Name()
                         << ", last_waypt = "
                         << ElementID(navState_.last_waypt).name().str
                         << ", replan_waypt = "
                         << ElementID(navState_.replan_waypt).name().str
                         << ", L" << navState_.lane_blocked
                         << " R" << navState_.road_blocked
                         << " S" << navState_.stopped
                         << " Z" << navState_.have_zones);

	// exit loop when Navigator has shut down
	if (navState_.estop.state == NavEstopState::Done)
          {
            ROS_INFO("Estop Done. Stopping.");
            break;
          }

	// generate navigator order for this cycle
	ROS_DEBUG("Calling command");
        art_nav::Order next_order= commander.command(navState_);
	
	// send next order to Navigator, if any
	if (next_order.behavior.value != NavBehavior::None)
	  putOrder(next_order);

        cycle.sleep();                  // sleep until next cycle

      }	//end of mission while loop

    ROS_INFO("Robot shut down.");
    return true;
  };
  
  void print_usage(int argc, char** argv)
  {
    std::cerr << "usage:  rosrun art_nav commander [options] <RNDF> <MDF>"
	      << std::endl << std::endl;
    std::cerr << "The <RNDF> and <MDF> file names are required." 
	      << std::endl << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  -l <filename>  load mission file if restarting mission " 
	      << mission_file_ << ")" << std::endl;
    std::cerr << "  -r             start running robot immediately"
	      << std::endl;
    std::cerr << "  -m <speed>     absolute maximum speed (default "
	      << speedlimit_ << " m/s)" << std::endl;
    std::cerr << "  -v             verbose messages (-vv for more)"
	      << std::endl;
    std::cerr << "  -?             print this help message"
	      << std::endl;
  }

  
private:

  // parameters
  std::string mission_file_;
  bool load_mission_;
  bool startrun_; 
  float speedlimit_;
  const char* rndf_name_;
  const char* mdf_name_;
  int verbose_;

  RNDF *rndf_;
  MDF *mdf_;
  Graph* graph_;
  Mission* mission_;
  ZonePerimeterList zones_;
  art_nav::NavigatorState navState_;
};

/** main program */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "commander");
  ros::NodeHandle node;

  ROS_INFO("commander starting");

  CommanderNode cmdr;
  
  if (!cmdr.parse_args(argc,argv))
    {
      std::cerr<<"\n";
      exit(1);    
    }

  if (!cmdr.build_RNDF())
    exit(2);

#if 0
  if (!cmdr.initialize_player())
    exit(3);
#endif

  if (!cmdr.spin())
    exit(4);

  return 0;
}

