/*
 *  Base class for finite state machine events
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include "Blockage.h"

void Blockages::pop_oldest()
{
  double oldest_time;
  
  if (!empty())
    oldest_time=blocks.at(0).block_time;
  else return;
  
  uint index=0;
  for (uint i=1; i<blocks.size();i++)
    if (blocks.at(i).block_time < oldest_time)
      {
	oldest_time = blocks.at(i).block_time;
	index=i;
      }
  pop_block(index);
}

void Blockages::pop_newest()
{
  double newest_time;
  
  if (!empty())
    newest_time=blocks.at(0).block_time;
  else return;
  
  uint index=0;
  for (uint i=1; i<blocks.size();i++)
    if (blocks.at(i).block_time > newest_time)
      {
	newest_time = blocks.at(i).block_time;
	index=i;
      }
  pop_block(index);
}

void Blockages::pop_block(uint index){
  
  block oblock=blocks.at(index);
  
  ROS_INFO_STREAM("Unblocking block #" << index);

  for (uint i=0; i< oblock.added_blocks.size(); i++)
    for (uint j=0; j< b_graph->edges_size; j++)
      if ((b_graph->edges.at(j).startnode_index ==
	   oblock.added_blocks.at(i).startnode_index) &&
	  (b_graph->edges.at(j).endnode_index ==
	   oblock.added_blocks.at(i).endnode_index))
	{
	  ROS_INFO_STREAM("Unblocking edge between "
                          <<b_graph->nodes[b_graph->edges.at(j).startnode_index].id.name().str
                          <<" and "
                          <<b_graph->nodes[b_graph->edges.at(j).endnode_index].id.name().str);
	  b_graph->edges.at(j).blocked=false;
	  break;
	}

  for (uint i=0; i< oblock.added_edges.size(); i++)
    for (uint j=0; j< b_graph->edges_size; j++)
      if ((b_graph->edges.at(j).startnode_index ==
	   oblock.added_edges.at(i).startnode_index) &&
	  (b_graph->edges.at(j).endnode_index ==
	   oblock.added_edges.at(i).endnode_index))
	{
	  ROS_INFO_STREAM("Deleting edge between "
                          <<b_graph->nodes[b_graph->edges.at(j).startnode_index].id.name().str
                          <<" and "
                          <<b_graph->nodes[b_graph->edges.at(j).endnode_index].id.name().str);
	  b_graph->edges.erase(b_graph->edges.begin()+j);
	  b_graph->edges_size--;
	  break;
	}
  
  blocks.erase(blocks.begin()+index);
}


void Blockages::add_block(ElementID other_lane_way) 
{
  block new_block;
  
  timeval time;
  gettimeofday(&time, NULL);
  
  new_block.block_time=time.tv_sec + (time.tv_usec/1000000.0);
  new_block.count=1;
  
  int other_lane_index=-1;
  int other_lane_other_side_index=-1;
  
  for (uint i=0; i< b_graph->nodes_size; i++)
    {
      if (b_graph->nodes[i].id == other_lane_way)
	other_lane_index=i;
      if ((b_graph->nodes[i].id.seg == other_lane_way.seg) &&
	  (b_graph->nodes[i].id.lane == other_lane_way.lane) &&
	  (b_graph->nodes[i].id.pt == other_lane_way.pt-1))
	other_lane_other_side_index=i;
    }
	  
  if (other_lane_index == -1)
    return;

  WayPointEdge e1;
  
  if (!b_path->empty())
    e1=b_path->at(0);
  else return;

  bool exists_other_lane_connection=false;

  if (other_lane_other_side_index != -1)
    {
      for (uint i=0; i< b_graph->edges_size; i++)
	{

	  if (b_graph->edges.at(i).endnode_index == other_lane_index)
	    {
	      b_graph->edges.at(i).blocked=true;
	      new_block.added_blocks.push_back(b_graph->edges[i]);
	      ROS_INFO_STREAM("Blocking edge between "
                              <<b_graph->nodes[b_graph->edges.at(i).startnode_index].id.name().str
                              <<" and "
                              <<b_graph->nodes[b_graph->edges.at(i).endnode_index].id.name().str);
	      if (b_graph->edges.at(i).startnode_index == other_lane_other_side_index)
		exists_other_lane_connection=true;
	    }
	  if ((b_graph->edges.at(i).endnode_index == e1.endnode_index) &&
	      (b_graph->edges.at(i).startnode_index == e1.startnode_index))
	    {
	      ROS_INFO_STREAM("Blocking edge between "
                              <<b_graph->nodes[e1.startnode_index].id.name().str
                              <<" and "
                              <<b_graph->nodes[e1.endnode_index].id.name().str);
	      b_graph->edges.at(i).blocked=true;
	      new_block.added_blocks.push_back(b_graph->edges[i]);
	    }
	}
    }
  
  WayPointEdge new_edge;
  new_edge.startnode_index=e1.startnode_index;
  new_edge.endnode_index=other_lane_index;
  new_edge.distance=100;
  new_edge.speed_max=DEFAULT_ZONE_SPEED;
  new_edge.blocked=false;
  
  ROS_INFO_STREAM("Adding edge between "
                  <<b_graph->nodes[new_edge.startnode_index].id.name().str
                  <<" and "
                  <<b_graph->nodes[new_edge.endnode_index].id.name().str);

  new_block.added_edges.push_back(new_edge);
  b_graph->edges.push_back(new_edge);
  b_graph->edges_size++;

  if (exists_other_lane_connection)
    {
      new_edge.startnode_index=other_lane_other_side_index;
      new_edge.endnode_index=e1.endnode_index;
      new_block.added_edges.push_back(new_edge);
      ROS_INFO_STREAM("Adding edge between "
                      <<b_graph->nodes[new_edge.startnode_index].id.name().str
                      <<" and "
                      <<b_graph->nodes[new_edge.endnode_index].id.name().str);
      b_graph->edges.push_back(new_edge);
      b_graph->edges_size++;
    }
  blocks.push_back(new_block);
}
