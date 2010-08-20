/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007, Mickey Ristroph
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 * 
 *  Graph search algorithms, for use by Commander, or whoever.
 */

#ifndef __GRAPHSEARCH_h__
#define __GRAPHSEARCH_h__

#include <art_map/Graph.h>
#include <art_map/types.h>
#include <queue>
#include <vector>
#include <map>
#include <iostream>

namespace GraphSearch {
  WayPointEdgeList astar_search(const Graph& graph,
				waypt_index_t start_id,
				waypt_index_t goal_id, 
				float speedlimit=1.0);

  WayPointNodeList edge_list_to_node_list(const Graph& graph,
					  WayPointEdgeList& edges);
};

#endif
