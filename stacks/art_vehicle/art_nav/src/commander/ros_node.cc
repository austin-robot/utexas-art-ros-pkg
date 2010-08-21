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

#include <art_nav/NavigatorState.h>

#include <art_nav/NavEstopState.h>
#include <art_nav/NavRoadState.h>

#include "command.h"

/** @file

    @brief ART vehicle commander client

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
    -m <speed>	maximum vehicle speed in m/s (default 7.5)
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

    Connect to the robot vehicle.  Path names for the rndf and
    mdf files are required.

    @verbatim
    $ commander -r example.rndf example.mdf
    @endverbatim

    Start running the robot vehicle immediately.

    @author Patrick Beeson, Jack O'Quin
*/


class CommanderNode
{
public:
  CommanderNode()
  {
    verbose_ = 1;
    nav_state_received_ = false;

    // use private node handle to get parameters
    ros::NodeHandle nh("~");

    // ROS parameters (in alphabetical order)
    nh.param("mdf", mdf_name_, std::string(""));
    ROS_INFO_STREAM("MDF: " << mdf_name_);
      
    nh.param("mission_state", mission_file_, std::string(""));
    load_mission_ = (mission_file_ != std::string(""));
    if (load_mission_)
      ROS_INFO_STREAM("mission state file: " << mission_file_);

    rndf_name_ = "";
    std::string rndf_param;
    if (nh.searchParam("rndf", rndf_param))
      {
        nh.param(rndf_param, rndf_name_, std::string(""));
        ROS_INFO_STREAM("RNDF: " << rndf_name_);
      }
    else
      {
        ROS_ERROR("RNDF not defined");
      }

    nh.param("speed_limit", speed_limit_, 7.5);
    if (speed_limit_ < 0)
      {
	ROS_ERROR("Maximum speed must be >= 0");
        speed_limit_ = 7.5;
      }
    ROS_INFO_STREAM("Maximum speed: " << speed_limit_);

    nh.param("start_run", startrun_, false);

    // class objects
    rndf_ = new RNDF(rndf_name_);
    mdf_ = new MDF(mdf_name_);
    graph_ = NULL;
    mission_ = NULL;
  }

  ~CommanderNode()
  {
    if (graph_ != NULL)
      delete graph_;
    if (mission_ != NULL)
      delete mission_;
    delete mdf_;
    delete rndf_;
  }

  /** Set up ROS topics */
  int setup(ros::NodeHandle node)
  {   
    static int qDepth = 1;
    nav_state_topic_ = node.subscribe("navigator/state", qDepth,
                                      &CommanderNode::processNavState, this);
    return 0;
  }

  /** Process navigator state input */
  void processNavState(const art_nav::NavigatorState::ConstPtr &nst)
  {
    nav_state_msg_ = *nst;
    if (nav_state_received_ == false)
      {
        ROS_INFO("initial navigator state received");
        nav_state_received_ = true;
      }
  }

  bool parse_args(int argc, char** argv)
  {

    // set the flags
    const char* optflags = "rv?";
    int ch;

    // use getopt to parse the flags
    while(-1 != (ch = getopt(argc, argv, optflags)))
      {
	switch(ch)
	  {
	  case 'r':                     // start run immediately
	    startrun_ = true;
	      break;
	  case 'v':                     // extra verbosity
	    verbose_++;
	      break;
	  case '?': // help
	  default:  // unknown
	    print_usage(argc, argv);
	    return false;
	  }
      }

    if (optind < argc)
      {
	std::cerr << "ERROR: invalid extra parameter: " 
		  << argv[optind] << std::endl;
	print_usage(argc, argv);
	return false;
      }

    return true;
  }

  bool build_graph()
  {
    if (!rndf_->is_valid)
      {
        ROS_FATAL("RNDF not valid");
        return false;;
      }

    graph_ = new Graph();
    rndf_->populate_graph(*graph_);
    graph_->find_mapxy();
    graph_->find_implicit_edges();
    //zones_ = ZoneOps::build_zone_list_from_rndf(*rndf_, *graph_);    

    // Fill in mission data
    if (!mdf_->is_valid)
      {
        ROS_FATAL("MDF not valid");
        return false;;
      }

    mdf_->add_speed_limits(*graph_);
    mission_ = new Mission(*mdf_);

    if (load_mission_) 
      {
        // Load state of previously started mission
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
        // No started mission
	if (!mission_->populate_elementid(*graph_))
	  {
	    ROS_FATAL("Mission IDs not same size as Element IDs");
	    return false;
	  }
	ROS_INFO("Running full mission from MDF");
      }
    
    if (mission_->remaining_points() < 1)
      {
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
    if (startrun_)                      // -r option specified?
      {
        ROS_INFO("ordering navigator to RUN");
        art_nav::Order run_order;
        run_order.behavior.value = NavBehavior::Run;
        putOrder(run_order);
      }

    // initialize Commander class
    Commander commander(verbose_, speed_limit_, graph_, mission_, zones_);

    ros::Rate cycle(HERTZ_COMMANDER);

    // loop until end of mission
    while(ros::ok())
      {
        // ROS_DEBUG_STREAM:
        ROS_INFO_STREAM("navstate = "
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
    std::cerr << "usage:  rosrun art_nav commander [options]"
	      << std::endl << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  -r             start running robot immediately"
	      << std::endl;
    std::cerr << "  -v             verbose messages (-vv for more)"
	      << std::endl;
    std::cerr << "  -?             print this help message"
	      << std::endl;
  }

  /** wait until all required input topics available */
  bool wait_for_input()
  {
    ROS_INFO("Waiting for navigator input");
    ros::Rate cycle(HERTZ_COMMANDER);
    while(ros::ok())
      {
        if (nav_state_received_)
          {
            ROS_INFO("Navigator input received");
            return true;                // navigator running
          }
        cycle.sleep();
      }
    return false;                       // node shut down
  }
  
private:

  // parameters
  std::string mission_file_;
  bool load_mission_;
  bool startrun_; 
  double speed_limit_;
  std::string rndf_name_;
  std::string mdf_name_;
  int verbose_;

  // topics and messages
  ros::Subscriber nav_state_topic_;       // navigator state topic
  art_nav::NavigatorState nav_state_msg_; // last received
  bool nav_state_received_;

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

  // build the road map graph
  if (!cmdr.build_graph())
    exit(2);

  // wait for input topics
  if (!cmdr.wait_for_input())
    exit(3);

  if (!cmdr.spin())
    exit(4);

  return 0;
}
