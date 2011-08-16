/*
 *  Copyright (C) 2010 UT-Austin &  Austin Robot Technology, Michael Quinlan
 *  Copyright (C) 2011 UT-Austin & Austin Robot Technology
 *  License: Modified BSD Software License 
 */

/** @file

    This node creates a pseudo occupancy grid ontop of the maplanes
    data structure.

    @author Michael Quinlan
    @author Jack O'Quin

*/

#include <ros/ros.h>
#include <art_observers/map_grid.h>

#define NODE "map_grid"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;
  MapGrid* mGrid = new MapGrid(node); 
  mGrid->spin();                        // handle incoming data

  return 0;
}
