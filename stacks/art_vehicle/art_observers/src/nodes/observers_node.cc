/*
 *  Copyright (C) 2010 UT-Austin &  Austin Robot Technology, Michael Quinlan
 *  Copyright (C) 2011 UT-Austin & Austin Robot Technology
 *  License: Modified BSD Software License 
 */

/** @file

    This node publishes lane observations.

    @author Michael Quinlan
    @author Jack O'Quin

*/

#include <ros/ros.h>
#include <art_observers/lane_observations.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "observers_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  LaneObservations* obs = new LaneObservations(node, priv_nh); 

  // handle incoming data until shutdown
  obs->spin();

  return 0;
}
