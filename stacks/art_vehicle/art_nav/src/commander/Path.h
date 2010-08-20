/* -*- mode: C++ -*-
 *
 *  Commander route path interface
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __PATH_H__
#define __PATH_H__

#include <deque>
#include <iostream>

#include <art/epsilon.h>
#include <art/conversions.h>
#include <art_map/zones.h>

class Path
{
public:
  Path()
  {
    path.clear();
  };
  
  ~Path()
  {
    // invalidate any iterators
    path.clear();
  };
  

  // Return way-point from Path[index] relatively safely,
  // use last WayPointEdge if index too big,
  // return null WayPointEdge if Path was empty.

  WayPointEdge at(unsigned index) const
  {
    if (path.size() > index)
      return path.at(index);
    else if (path.size() > 0)
      return last();
    else
      return WayPointEdge();
  }

  void clear(void)
  {
    path.clear();
  }

  // return last WayPointN of Path
  WayPointEdge last(void) const
  {
    if (path.empty())
      return WayPointEdge();
    return path.at(path.size()-1);
  }

  void pop_front(void)
  {
    if (path.size() > 1)
      path.pop_front();
  }

  void new_path(waypt_index_t startid,
		waypt_index_t endid,
		const WayPointEdgeList& edges)
  {
    path.clear();

    append_path(startid, endid, edges);

  }


  void append_path(waypt_index_t startid,
		   waypt_index_t endid,
		   const WayPointEdgeList& edges)
  {

    if (edges.empty())
      {
	WayPointEdge new_edge;
	new_edge.startnode_index=startid;
	new_edge.endnode_index=endid;
	new_edge.speed_min=0;
	new_edge.distance=0.0;
	new_edge.speed_max=DEFAULT_ZONE_SPEED;
	path.push_back(new_edge);
	return;
      }

    /*    
    // If A* was run from inside a zone, then first edge touches exit
    // of zone, not inner waypoint.
    if (edges[0].startnode_index != startid) 
      {
	WayPointEdge new_edge;
	new_edge.startnode_index=startid;
	new_edge.distance=0.0;
	new_edge.endnode_index=edges[0].startnode_index;
	new_edge.speed_min=0;
	new_edge.speed_max=DEFAULT_ZONE_SPEED;
	path.push_back(new_edge);
      }
    */
    for (uint i=0; i < edges.size(); i++)
      {
	path.push_back(edges[i]);
	
      }

    /*
    // If A* was run to get into zone, then last edge goes to zone
    // entry, not to waypoint inside zone
    if (edges[edges.size()-1].endnode_index != endid) 
      {
	WayPointEdge new_edge;
	new_edge.startnode_index=edges[edges.size()-1].endnode_index;
	new_edge.distance=0.0;
	new_edge.endnode_index=endid;
	new_edge.speed_min=0;
	new_edge.speed_max=DEFAULT_ZONE_SPEED;
	path.push_back(new_edge);
      }
    */

  }



  // return number of way-points in Path
  unsigned size(void) const
  {
    return path.size();
  }

  // return whether Path is empty
  bool empty(void) const
  {
    return path.empty();
  }

  void print(const Graph& graph)
  {
    ROS_INFO("===============");
    for (unsigned i = 0; i < path.size(); i++)
      {
	if (i==0)
	  ROS_INFO_STREAM("route [" << i << "] is "
                          << graph.get_node_by_index(path.at(i).startnode_index)->id.name().str);

	ROS_INFO_STREAM("route [" << i+1 << "] is "
                        << graph.get_node_by_index(path.at(i).endnode_index)->id.name().str);
      }
    ROS_INFO("================");
  }


private:
  // hide the deque implementation
  std::deque<WayPointEdge> path;

};

#endif // __PATH_H__
