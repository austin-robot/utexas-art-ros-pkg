/* -*- mode: C++ -*-
 *
 *  Base class for finite state machine events
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __BLOCKAGE_h__
#define __BLOCKAGE_h__

#include <art_map/Graph.h>
#include "Path.h"
#include <vector>

class block {
public:
  block() {};
  int count;
  double block_time;
  std::vector<WayPointEdge> added_blocks;
  std::vector<WayPointEdge> added_edges;
};

class Blockages {
public:
  Blockages(Graph* graph, Path* path) :
    b_graph(graph), b_path(path) {}

  bool empty() 
  {
    return blocks.empty();
  }

  void pop_oldest(); 
  void pop_newest(); 
  void pop_block(uint index); 
  void add_block(ElementID next);
  
private:
  Graph* b_graph;
  Path* b_path;
  
  std::vector<block> blocks;
};

#endif
