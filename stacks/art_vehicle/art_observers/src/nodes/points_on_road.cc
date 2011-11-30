/*
 *  Copyright (C) 2010 UT-Austin &  Austin Robot Technology, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */

/** \file

  This node takes in a point cloud an republishes only the points that
  lie inside road polygons

  This probably not the neatest or fastest way of doing this, but it's
  an example.

  @author Michael Quinlan

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <art_msgs/ArtLanes.h>

#include <art_map/PolyOps.h>
#include <tf/transform_listener.h>

#include <string>

#define NODE "maplanes_grid"

static int qDepth = 1;                  // ROS topic queue size
static ros::Publisher output;
sensor_msgs::PointCloud pc;             // outgoing PointCloud message

tf::TransformListener* listener;
art_msgs::ArtLanes map;                 // local map
PolyOps* pops;

bool isPointInMap(float x, float y) 
{
  int numPolys = map.polygons.size();
  for (int i=0; i<numPolys; i++)
    {
      poly p(map.polygons[i]);
      if (pops->pointInPoly(x,y,p))
        {
          return true;
        }
    }
  return false;
}

/** \brief callback for incoming point cloud

    Transforms cloud to /map frame of reference
    CHecks if each point is in a polygon, if so adds it to output point cloud
 */
void processObstacles(const sensor_msgs::PointCloud &msg)
{
   //ROS_INFO("num 3d points = %d", msg->points.size());  
	if(msg.points.size()<1)
		return;
  
  sensor_msgs::PointCloud trans;
  try {
    listener->transformPointCloud("/map", msg, trans); 

  } catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
 
  // pass along original time stamp and frame ID
  pc.header.stamp = trans.header.stamp;
  pc.header.frame_id = trans.header.frame_id;

  // set the exact point cloud size -- the vectors should already have
  // enough space
  size_t npoints = trans.points.size();
  pc.points.resize(npoints);
  size_t count=0;
  for (unsigned i = 0; i < npoints; ++i)
    {
      bool in = isPointInMap(trans.points[i].x,trans.points[i].y);
      if (in) {
        pc.points[count].x = trans.points[i].x;
        pc.points[count].y = trans.points[i].y;
        pc.points[count].z = trans.points[i].z;
        count++;
      }
    }
  pc.points.resize(count);
  output.publish(pc);
}

// \brief stores the values of the map
void processMap(const art_msgs::ArtLanes &msg)
{
  map = msg;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  pops = new PolyOps();
  
  // subscribe to velodyne input -- make sure queue depth is minimal,
  // so any missed scans are discarded.  Otherwise latency gets out of
  // hand.  It's bad enough anyway.
  ros::Subscriber velodyne_obstacles =
    node.subscribe("velodyne/obstacles", qDepth, processObstacles,
                   ros::TransportHints().tcpNoDelay(true));

  
  ros::Subscriber road_map =
    node.subscribe("roadmap_local", qDepth, processMap,
                   ros::TransportHints().tcpNoDelay(true));

  output = node.advertise<sensor_msgs::PointCloud>("obstacles/points_on_road",
                                                   qDepth);
  
  listener = new tf::TransformListener();
 
  ros::spin();                          // handle incoming data

  return 0;
}
