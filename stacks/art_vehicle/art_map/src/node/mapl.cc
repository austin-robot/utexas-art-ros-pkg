/*
 * ROS node for ART map lanes.
 *
 *  Copyright (C) 2005 Austin Robot Technology
 *  Copyright (C) 2010 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <unistd.h>
#include <string.h>
#include <iostream>

#include <ros/ros.h>

#include <art/hertz.h>
#include <nav_msgs/Odometry.h>

#include <art_map/ArtLanes.h>
#include <art_map/Graph.h>
#include <art_map/MapLanes.h>
#include <art_map/RNDF.h>


/** @file

 @brief provide RNDF map lane boundaries for ART vehicle

Subscribes:

- @b odom [nav_msgs::Odometry] estimate of robot position and velocity.

Publishes:

- @b roadmap_global [art_map::ArtLanes] global road map lanes (latched topic)
- @b roadmap_local [art_map::ArtLanes] local area road map lanes

These data are published for the \b /map frame of reference.

@author Jack O'Quin, Patrick Beeson

*/

/** ROS node class for road map driver. */
class MapLanesDriver
{
public:
    
  // Constructor
  MapLanesDriver();
  ~MapLanesDriver()
    {
      delete map_;
      if (graph_)
        delete graph_;
    };

  bool buildRoadMap(void);
  int  Setup(ros::NodeHandle node);
  int  Shutdown(void);
  void Spin(void);

private:

  void processOdom(const nav_msgs::Odometry::ConstPtr &odomIn);
  void publishGlobalMap(void);
  void publishLocalMap(void);

  // parameters:
  double range_;                ///< radius of local lanes to report (m)
  double poly_size_;            ///< maximum polygon size (m)
  std::string rndf_name_;       ///< Road Network Definition File name

  // topics and messages
  ros::Subscriber odom_topic_;       // odometry topic
  nav_msgs::Odometry odom_msg_;      // last Odometry message received

  ros::Publisher roadmap_global_;       // global road map publisher
  ros::Publisher roadmap_local_;        // local road map publisher

  Graph *graph_;                  ///< graph object (used by MapLanes)
  MapLanes* map_;                 ///< MapLanes object instance
  bool initial_position_;         ///< true if initial odometry received
};

/** constructor */
MapLanesDriver::MapLanesDriver(void)
{
  initial_position_ = false;

  // use private node handle to get parameters
  ros::NodeHandle nh("~");

  nh.param("range", range_, 80.0);
  ROS_INFO("range to publish = %.0f meters", range_);

  nh.param("poly_size", poly_size_, MIN_POLY_SIZE);
  ROS_INFO("polygon size = %.0f meters", poly_size_);

  nh.param("rndf", rndf_name_, std::string(""));
  ROS_INFO_STREAM("RNDF name = " << rndf_name_);

  // create the MapLanes class
  map_ = new MapLanes(range_);
  graph_ = NULL;
}

/** Set up ROS topics */
int MapLanesDriver::Setup(ros::NodeHandle node)
{   
  static int qDepth = 1;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);

  odom_topic_ = node.subscribe("odom", qDepth, &MapLanesDriver::processOdom,
                               this, noDelay);

  // Local road map publisher
  roadmap_local_ =
    node.advertise<art_map::ArtLanes>("roadmap_local", qDepth);

  // Use latched publisher for global road map topic
  roadmap_global_ =
    node.advertise<art_map::ArtLanes>("roadmap_global", 1, true);

  return 0;
}


/** Shutdown the driver */
int MapLanesDriver::Shutdown()
{
  // Stop and join the driver thread
  ROS_INFO("shutting down maplanes");
  return 0;
}

/** Handle odometry input */
void MapLanesDriver::processOdom(const nav_msgs::Odometry::ConstPtr &odomIn)
{
  odom_msg_ = *odomIn;
  if (initial_position_ == false)
    {
      ROS_INFO("initial odometry received");
      initial_position_ = true;         // have position data now
    }
}

/** Publish global road map */
void MapLanesDriver::publishGlobalMap(void)
{
  art_map::ArtLanes lane_data;
  if (0 == map_->getAllLanes(&lane_data))
    {
      ROS_WARN("no map data available to publish");
      return;
    }

  // the map is in the /map frame of reference with present time
  lane_data.header.stamp = ros::Time::now();
  lane_data.header.frame_id = "/map";

  ROS_INFO_STREAM("publishing " <<  lane_data.polygons.size()
                  <<" global roadmap polygons");
  roadmap_global_.publish(lane_data);
}

/** Publish current local road map */
void MapLanesDriver::publishLocalMap(void)
{
  art_map::ArtLanes lane_data;
  if (0 != map_->getLanes(&lane_data, MapXY(odom_msg_.pose.pose.position)))
    {
      ROS_DEBUG("no map data available to publish");
      return;
    }

  // the map is in the /map frame of reference with time of the
  // latest odometry message
  lane_data.header.stamp = odom_msg_.header.stamp;
  lane_data.header.frame_id = "/map";

  ROS_DEBUG_STREAM("publishing " <<  lane_data.polygons.size()
                   <<" local roadmap polygons");
  roadmap_local_.publish(lane_data);
}

/** Spin function for driver thread */
void MapLanesDriver::Spin() 
{
  publishGlobalMap();                   // publish global map once at start

  ros::Rate cycle(HERTZ_MAPLANES);      // set driver cycle rate

  // Loop publishing MapLanes state until driver Shutdown().
  while(ros::ok())
    {
      ros::spinOnce();                  // handle incoming messages

      if (initial_position_)
        {
          // Publish local roadmap
          publishLocalMap();
        }

      cycle.sleep();                    // sleep until next cycle
    }
}

/** Build road map
 *
 *  @return true if successful
 */  
bool MapLanesDriver::buildRoadMap(void)
{
  if (rndf_name_ == "")
    {
      ROS_FATAL("required ~rndf parameter missing");
      return false;
    }

  RNDF *rndf = new RNDF(rndf_name_);
  
  if (!rndf->is_valid)
    {
      ROS_FATAL("RNDF not valid");
      delete rndf;
      return false;;
    }

  // Allocate a way-point graph.  Populate with nodes from the RNDF,
  // then fill in the MapXY coordinates relative to a UTM grid based
  // on the first way-point in the graph.
  graph_ = new Graph();
  rndf->populate_graph(*graph_);
  graph_->find_mapxy();

  // MapRNDF() saves a pointer to the Graph object, so we can't delete it here.
  // That is why graph_ is a class variable, deleted in the deconstructor.
  // TODO: fix this absurd interface
  int rc = map_->MapRNDF(graph_, poly_size_);
  delete rndf;

  if (rc != 0)
    {
      ROS_FATAL("cannot process RNDF! (%s)", strerror(rc));
      return false;
    }

  return true;
}

/** main program */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "maplanes");
  ros::NodeHandle node;
  MapLanesDriver dvr;

  if (dvr.Setup(node) != 0)
    return 2;
  if (!dvr.buildRoadMap())
    return 3;
  dvr.Spin();
  dvr.Shutdown();

  return 0;
}
