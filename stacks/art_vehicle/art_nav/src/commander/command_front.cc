//
// Commander node ROS front end
//
//  Author:  Patrick Beeson, Jack O'Quin
//


#include <iostream>

#include <art_map/ZoneOps.h>
#include <applanix/applanix_info.h>

using namespace applanix_info;          // defines gps_info

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

// proably should move to NavigatorProxy()
art_nav::NavigatorState get_nav_state(PlayerCc::OpaqueProxy *nav)
{
  
  if (nav->GetCount() != data_size)
    {
      // error in message size
      logc(10) << "ERROR: Navigator message size is incorrect\n";
    }
  
  // get data packet
  nav->GetData(data);
  memcpy(&hdr, data, sizeof(art_message_header_t));
  memcpy(&navdata, data+sizeof(art_message_header_t), sizeof(art_nav::NavigatorState));
  if (hdr.type != NAVIGATOR_MESSAGE
      || hdr.subtype != NAVIGATOR_MESSAGE_STATE_DATA)
    {
      // wrong message type
      logc(10) << 
	"ERROR: Execute received the wrong message type from Navigator\n";
    }
  return navdata;
}

gps_info get_gps_state(PlayerCc::OpaqueProxy *gps) {
  if (gps->GetCount() > sizeof(gps_info)) {
    // error in message size
    logc(10) << "ERROR: GPS opqaue message size is incorrect\n";
  }
  
  gps_info pos;
  
  // get data packet
  gps->GetData((uint8_t*) &pos);
  
  return pos;
}


// send order to navigator driver
// Probably should move to NavigatorProxy()
void PutOrder(PlayerCc::OpaqueProxy *nav, Order order, player_opaque_data_t &opaque)
{
  order_message_t ocmd;
  ocmd.od = order;

  // format opaque message

  art_message_header_t msghdr;
  msghdr.type = NAVIGATOR_MESSAGE;
  msghdr.subtype = NAVIGATOR_MESSAGE_ORDER;

  memcpy(opaque.data, &msghdr, sizeof(msghdr));
  memcpy(opaque.data+sizeof(msghdr), &ocmd, sizeof(ocmd));
  nav->SendCmd(&opaque);

}

void SendZones(PlayerCc::OpaqueProxy *nav, const ZonePerimeterList &zones) {
  
  /*  logc(10) << "Number of Zones: " << zones.size() << "\n";
  for(unsigned i = 0; i < zones.size(); i++) {
    logc(10) << "Zone ID: " << zones[i].zone_id << " Perimeter: ";
    for(unsigned j = 0; j < zones[i].perimeter_points.size(); j++) {
      logc(10) << "("<<zones[i].perimeter_points[j].map.x <<", "<<zones[i].perimeter_points[j].map.y << "), ";
    }
    logc(10) << "\n";
    }*/
  

  player_opaque_data_t opaque;
  ZoneOps::package_zone_list_into_opaque(zones, opaque);

  //art_message_header_t *art_hdr = (art_message_header_t*) opaque.data;
  //logc(10)<<"Sending opaque: Type: "<<art_hdr->type<<" Subtype: "<<art_hdr->subtype<<"\n";

  nav->SendCmd(&opaque);
  if (opaque.data != NULL)
    delete [] opaque.data;
  /*
  int j = 0;
  for(int i = 0; i < 100000; i++)
    j += i * (i%2==0)?-1:1;

  logc(10) << "Stuff to eat up cycles: "<<j<<"\n";*/
}


class Startup {
public:
  Startup() {
    _hostname = PlayerCc::PLAYER_HOSTNAME;
    _mission_file = "mission_state";
    load_mission=false;
    _port = PlayerCc::PLAYER_PORTNUM;
    _startrun = false;
    _verbose = 1;
    _speedlimit = 7.5;
    //_speedlimit = 5.5;			// for Area A test

    robot = NULL;
    nav = NULL;
    maplane = NULL;
    gps = NULL;
    pos2d = NULL;

    rndf = NULL;
    mdf = NULL;
    graph = NULL;
    mission = NULL;

    opaque.data_count = (sizeof(art_message_header_t)
			 + sizeof(order_message_t));
    opaque.data = new uint8_t[opaque.data_count];
  }

  ~Startup() {

    if (nav != NULL)
      delete nav;
    if (maplane != NULL)
      delete maplane;
    if (gps != NULL)
      delete gps;
    if (pos2d != NULL)
      delete pos2d;
    if (robot != NULL)
      delete robot;
    if (rndf != NULL)
      delete rndf;
    if (mdf != NULL)
      delete mdf;
    if (graph != NULL)
      delete graph;
    if (mission != NULL)
      delete mission;
    delete [] opaque.data;
  }

  bool parse_args(int argc, char** argv) {

    // set the flags
    const char* optflags = "h:l:p:m:rv?";
    int ch;
    std::cerr<<"\n";

    // use getopt to parse the flags
    while(-1 != (ch = getopt(argc, argv, optflags)))
      {
	switch(ch)
	  {
	  case 'h':				// hostname
	    _hostname = optarg;
	    break;
	  case 'l':				// hostname
	    _mission_file = optarg;
	    load_mission=true;
	    break;
	  case 'p':				// port
	    _port = atoi(optarg);
	    break;
	  case 'r':				// run
	    _startrun = true;
	    break;
	  case 'm': 
	    _speedlimit = atof(optarg);
	    break;
	  case 'v':				// extra verbosity
	    _verbose++;
	      break;
	  case '?': // help
	  default:  // unknown
	    print_usage(argc, argv);
	    return false;
	  }
      }

    if (_speedlimit < 0)
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
  
    _rndf_filename = argv[optind++];  

    _mdf_filename = argv[optind++];

    if (optind < argc)
      {
	std::cerr << "ERROR: invalid extra parameter: " 
		  << argv[optind] << std::endl;
	print_usage(argc, argv);
	return false;
      }
    
    for (int i=0; i< argc; i++)
      logc(10) << argv[i] <<" ";
    logc(10)<<"\n";

    return true;
  }

  bool initialize_player() {
    
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
      logc(10) << "Waiting for initial lat/long info\n";
      try {
	robot->Read();
      }
      catch (PlayerCc::PlayerError e) {
	logc(10) << "Cannot read initial gps info from odometry...stopping\n";
	return false;
      }
    }
    
    gps_info pos = get_gps_state(gps);
    
    if (!graph->rndf_is_gps())
      {
	logc(10) << 
	  "RNDF seems too large to be composed of GPS waypoints...stopping\n";
	return false;	
      }
    
    graph->find_mapxy(pos, pos2d->GetXPos(), pos2d->GetYPos());
    maplane->SendRNDF(graph);
    
    graph->find_implicit_edges();

    while (!nav->IsValid()) {
      logc(10) << "Waiting for navigator to publish info\n";
      try {
	robot->Read();
      }
      catch (PlayerCc::PlayerError e) {
	logc(10) << "Cannot read initial info from navigator...stopping\n";
	return false;
      }
    }
    
    zones = ZoneOps::build_zone_list_from_rndf(*rndf, *graph);    

    while (true)
      {
	logc(2) << "Trying to send zones...\n";
	SendZones(nav, zones);
	
	// this blocks until new data comes; nominally 10Hz by default
	try { 
	  robot->Read();
	}
	catch (PlayerCc::PlayerError e) {
	  logc(10) << "Cannot read info from Player...stopping\n";
	  return false;
	}    
	
	art_nav::NavigatorState navState = get_nav_state(nav);
	
	if(navState.have_zones) 
	  {
	    logc(2) << "Navigator has received zones.\n";
	    break;
	  }
      }
    
    return true;    
  }
  
  bool build_RNDF() {

    rndf = new RNDF(_rndf_filename);
  
    if (!rndf->is_valid) {
      logc(10)<< "RNDF not valid\n";
      return false;;
    }

    mdf = new MDF(_mdf_filename);

    if (!mdf->is_valid) {
      logc(10)<<"MDF not valid\n";
      return false;;
    }

    graph = new Graph();
    rndf->populate_graph(*graph);
    
    if (graph->nodes_size > MAX_NODE_SIZE ||
	graph->edges_size > MAX_EDGE_SIZE) {
      logc(10) << "Graph too big. Currently lanes.h sets MAX_NODE_SIZE to be "
	       << MAX_NODE_SIZE<<" but really is "<<graph->nodes_size
	       << "and MAX_EDGE size to be "<<MAX_EDGE_SIZE<<" but really is "
	       <<graph->edges_size<<".\n";
      return false;
    }

    mdf->add_speed_limits(*graph);
    mission = new Mission(*mdf);

    if (load_mission) 
      {
	mission->clear();
	if (!mission->load(_mission_file.c_str(), *graph))
	  {
	    logc(10)<<"Unable to load stored mission file. "
		    <<_mission_file<<" is missing or corrupt\n";
	    return false;
	  }
	logc(8)<<"Loaded stored mission from "<<_mission_file<<"\n";
      }
    else
      {
	if (!mission->populate_elementid(*graph))
	  {
	    logc(10)<<"Mission IDs not same size as Element IDs\n";
	    return false;
	  }
	logc(8)<<"Running full mission from MDF\n";
      }
    
    
    if (mission->remaining_points() < 1) {
      logc(10)<<"No checkpoints left\n";
      return false;
    }

    return true;
  }


  bool run() {

    //set up the previous time for cycle benchmarking
    timeval time;
  
    if (_startrun) {
      logc(10) << "ordering navigator to RUN" << "\n";
      Order run_order;
      memset(&run_order, 0, sizeof(run_order));
      run_order.behavior = NavBehavior::Run;
      PutOrder(nav, run_order, opaque);
    }

    // initialize Commander class
    Commander commander(_verbose, _speedlimit, graph, mission, zones);

    // loop until end of mission
    while (true)
      {
	//print how long this commander cycle took
	gettimeofday(&time, NULL);
	logc(0) << "Commander cycle started at "
		<< std::setprecision(16) 
		<< time.tv_sec + (time.tv_usec/1000000.0)
		<< "\n";

	art_nav::NavigatorState navState = get_nav_state(nav);

	if (_verbose) {
	  logc(2) << "navstate = " << navState.estop_state.Name()
		  << ", " << navState.road_state.Name()
		  << ", last_waypt = " << navState.last_waypt.name().str
		  << ", replan_waypt = " << navState.replan_waypt.name().str
		  << ", L" << navState.lane_blocked
		  << " R" << navState.road_blocked
		  << " S" << navState.stopped
		  << " Z" << navState.have_zones
		  << "\n";
	}
	

	// exit loop when Navigator has shut down
	if (navState.estop_state == NavEstopState::Done) {
	  logc(5) << "Estop Done. Stopping.\n";
	  break;
	}

	logc(2) << "Calling command" << "\n";
	// generate navigator order for this cycle
	Order next_order= commander.command(navState);
	
	// send next order to Navigator, if any
	if (next_order.behavior != NavBehavior::None)
	  PutOrder(nav, next_order, opaque);

	// this blocks until new data comes; nominally 10Hz by default
	try { 
	  robot->Read();
	}
	catch (PlayerCc::PlayerError e) {
	  logc(10) << "Cannot read info from Player...stopping\n";
	  return false;
	}
	
      }	//end of mission while loop

    logc(10) << "Robot shut down." << "\n";
    return true;
  };
  
  void print_usage(int argc, char** argv)
  {
    std::cerr << "usage:  " << *argv << " [options] RNDF MDF"
	      << std::endl << std::endl;
    std::cerr << "The RNDF and MDF names are required.  Run " 
	      << *argv << " in the" << std::endl;
    std::cerr << "same directory as player, if these are relative file names."
	      << std::endl << std::endl;
    std::cerr << "Where [options] can be:" << std::endl;
    
    std::cerr << "  -h <hostname>  hostname to connect to (default "
	      << _hostname << ")" << std::endl;
    std::cerr << "  -l <filename>  load mission file if restarting mission " 
	      << _mission_file << ")" << std::endl;
    std::cerr << "  -p <port>      port where Player will listen (default "
	      << _port << ")" << std::endl;
    std::cerr << "  -r             start running robot immediately"
	      << std::endl;
    std::cerr << "  -m <speed>     absolute maximum speed (default "
	      << _speedlimit << " m/s)" << std::endl;
    std::cerr << "  -v             verbose messages (-vv for more)"
	      << std::endl;
    std::cerr << "  -?             print this help message"
	      << std::endl;
  }

  
private:
  std::string _hostname;
  std::string _mission_file;
  bool load_mission;
  int _port;
  bool _startrun; 
  float _speedlimit;
  const char* _rndf_filename;
  const char* _mdf_filename;
  int _verbose;

  // PlayerCc::PlayerClient* robot;
  // PlayerCc::OpaqueProxy* nav;
  // ArtProxy::LanesProxy *maplane;
  // PlayerCc::OpaqueProxy* gps;
  // PlayerCc::Position2dProxy* pos2d;

  RNDF *rndf;
  MDF *mdf;
  Graph* graph;
  Mission* mission;

  ZonePerimeterList zones;
};


int main(int argc, char **argv)
{
  timeval time;

  gettimeofday(&time, NULL);
  logc(0) << "commander run at "
	  << std::setprecision(16) 
	  << time.tv_sec + (time.tv_usec/1000000.0)
	  << "\n";

  Startup start;
  
  if (!start.parse_args(argc,argv))
    {
      std::cerr<<"\n";
      exit(1);    
    }

  if (!start.build_RNDF())
    exit(2);

  if (!start.initialize_player())
    exit(3);

  if (!start.run())
    exit(4);

  return 0;
}

