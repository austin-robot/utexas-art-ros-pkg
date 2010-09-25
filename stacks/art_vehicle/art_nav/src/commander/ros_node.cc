/*
 * Commander node ROS front end
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 *
 *  Author:  Patrick Beeson, Jack O'Quin
 */

#include <iostream>

#include <ros/ros.h>

#include <art_msgs/ArtHertz.h>
#include <art_map/ZoneOps.h>

#include <art_msgs/NavigatorState.h>
#include <art_nav/NavEstopState.h>
#include <art_nav/NavRoadState.h>

#include "command.h"

/** @file

    @brief ART vehicle commander node

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
    -r		run vehicle immediately
    -v        	verbose messages (-vv for more)
    -?		print usage message
    @endverbatim

    @section Examples

    @verbatim
    $ rosrun art_nav commander _rndf:=example.rndf _mdf:=example.mdf
    @endverbatim

    Run the robot vehicle using the specified road network and mission.

    @verbatim
    $ rosrun art_nav commander -r _rndf:=example.rndf _mdf:=example.mdf
    @endverbatim

    Start running the robot immediately.

    @todo Make separate ROS packages for commander, navigator and pilot.

    @author Patrick Beeson, Jack O'Quin
*/


/** @brief Commander node class */
class CommanderNode
{
public:
  CommanderNode()
  {
    verbose_ = 1;

    // use private node handle to get parameters
    ros::NodeHandle nh("~");

    // ROS parameters (in alphabetical order)
    frame_id_ = "/map";
    if (nh.getParam("frame_id", frame_id_))
      {
        ROS_INFO_STREAM("map frame ID = " << frame_id_);
      }

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
  bool setup(ros::NodeHandle node)
  {   
    // no delay: we always want the most recent data
    ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
    static int qDepth = 1;
    nav_state_topic_ = node.subscribe("navigator/state", qDepth,
                                      &CommanderNode::processNavState, this,
                                      noDelay);
    nav_cmd_pub_ = 
      node.advertise<art_msgs::NavigatorCommand>("navigator/cmd", qDepth);
    return true;
  }

  /** Process navigator state input */
  void processNavState(const art_msgs::NavigatorState::ConstPtr &nst)
  {
    ROS_DEBUG("navigator state message received");
    navState_ = *nst;
  }

  /** Parse command line arguments */
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

  /** Build road map graph */
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

  
  /** Send order in command to navigator driver */
  void putOrder(const art_msgs::Order &order)
  {
    art_msgs::NavigatorCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = frame_id_;
    cmd.order = order;
    ROS_DEBUG_STREAM("sending behavior "
                     << NavBehavior(cmd.order.behavior).Name());
    nav_cmd_pub_.publish(cmd);
  }

  /** Main spin loop */
  bool spin()
  {
    if (startrun_)                      // -r option specified?
      {
        ROS_INFO("ordering navigator to Run");
        art_msgs::Order run_order;
        run_order.behavior.value = NavBehavior::Run;
        putOrder(run_order);
      }

    // initialize Commander class
    Commander commander(verbose_, speed_limit_, graph_, mission_, zones_);

    // loop until end of mission
    ROS_INFO("begin mission");
    ros::Rate cycle(art_msgs::ArtHertz::COMMANDER);
    while(ros::ok())
      {
        ros::spinOnce();                  // handle incoming messages

        ROS_DEBUG_STREAM("navstate = "
                         << NavEstopState(navState_.estop).Name()
                         << ", " << NavRoadState(navState_.road).Name()
                         << ", last_waypt = "
                         << ElementID(navState_.last_waypt).name().str
                         << ", replan_waypt = "
                         << ElementID(navState_.replan_waypt).name().str
                         << ", L" << (bool) navState_.lane_blocked
                         << " R" << (bool) navState_.road_blocked
                         << " S" << (bool) navState_.stopped
                         << " Z" << (bool) navState_.have_zones);

	// exit loop when Navigator has shut down
	if (navState_.estop.state == NavEstopState::Done)
          {
            ROS_INFO("Estop Done. Stopping.");
            break;
          }

	// generate navigator order for this cycle
        art_msgs::Order next_order = commander.command(navState_);
	
	// send next order to Navigator, if any
	if (next_order.behavior.value != NavBehavior::None)
	  putOrder(next_order);

        cycle.sleep();                  // sleep until next cycle

      }	//end of mission while loop

    ROS_INFO("Robot shut down.");
    return true;
  };

  /** Print command argument usage message */
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

  /** Wait for navigator state message to arrive */
  bool wait_for_input()
  {
    ROS_INFO("Waiting for navigator input");
    ros::Rate cycle(art_msgs::ArtHertz::COMMANDER);
    while(ros::ok())
      {
        ros::spinOnce();                // handle incoming messages
        if (navState_.header.stamp != ros::Time())
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
  std::string frame_id_;        ///< frame ID of map (default "/map")

  // topics and messages
  ros::Subscriber nav_state_topic_;       // navigator state topic
  ros::Publisher nav_cmd_pub_;            // navigator command topic
  art_msgs::NavigatorState navState_;     // last received

  RNDF *rndf_;
  MDF *mdf_;
  Graph* graph_;
  Mission* mission_;
  ZonePerimeterList zones_;
};

/** Main program */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "commander");
  ros::NodeHandle node;

  ROS_INFO("commander starting");

  CommanderNode cmdr;
  
  if (!cmdr.parse_args(argc,argv))
    {
      std::cerr<<"\n";
      return 1;    
    }

  // set up ROS topics
  if (!cmdr.setup(node))
    return 2;

  // build the road map graph
  if (!cmdr.build_graph())
    return 3;

  // wait for input topics
  if (!cmdr.wait_for_input())
    return 4;

  // keep invoking commander until mission completed
  if (!cmdr.spin())
    return 5;

  return 0;
}
