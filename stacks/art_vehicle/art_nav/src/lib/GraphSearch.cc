/*
 *  Copyright (C) 2007, Mickey Ristroph
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <art_nav/GraphSearch.h>
#include <art_map/euclidean_distance.h>
#include <float.h>

namespace GraphSearch {
  WayPointNodeList edge_list_to_node_list(const Graph& graph,
					  WayPointEdgeList& edges) {
    WayPointNodeList nodes;
    WayPointEdgeList::iterator i = edges.begin();

    if(i != edges.end()) {
      WayPointNode *start = graph.get_node_by_index(i->startnode_index);
      if(start != NULL)
	nodes.push_back(*start);
    }
    
    while(i != edges.end()) {
      WayPointNode *end = graph.get_node_by_index(i->endnode_index);
      if(end != NULL)
	nodes.push_back(*end);
      i++;
    }

    return nodes;
  }

  
  // A possible path includes the estimated total cost, the actual cost so far,
  // and a list of edges to follow
  
  typedef std::pair<std::pair<double,double>,WayPointEdgeList> PossiblePath;

  void print_edge_list(WayPointEdgeList& edges, const Graph& graph) {
    std::cout<<graph.get_node_by_index(edges.begin()->startnode_index)->id.name().str;
    for(WayPointEdgeList::iterator i = edges.begin(); i != edges.end(); i++) {
      std::cout<<" -> "<<graph.get_node_by_index(i->endnode_index)->id.name().str;
    }
    std::cout<<std::endl;
  }

  void print_possible_path(PossiblePath& pp, const Graph& graph) {
    std::cout<<"Estimated: "<<pp.first.first<<"So far: "<<pp.first.second<<std::endl;
    print_edge_list(pp.second, graph);
  }

  class PossiblePathComparision
  {
  public:
    PossiblePathComparision() {}
    bool operator() (const PossiblePath& lhs, const PossiblePath& rhs) const {
      return (lhs.first.first > rhs.first.first);
    }
  };
  typedef std::priority_queue<PossiblePath,std::vector<PossiblePath>,PossiblePathComparision> PathPriorityQueue;
  
  double time_between_nodes(const WayPointNode& start,
			    const WayPointNode& end,
			    float speedlimit) {
    float distance = 0;
    if(start.is_perimeter || end.is_perimeter ||
       start.is_spot || end.is_spot)
      distance = Infinite::distance;
    else
      distance = Euclidean::DistanceTo(start.map,end.map);
    
    float time = distance/speedlimit;

    // penalize for stopping
    if ((start.id.seg != end.id.seg) ||
	(start.id.lane != end.id.lane) ||
	(end.id.pt != start.id.pt+1))
      time+=10.0;

    return time;

  }

  double time_along_edge(const Graph& graph, const WayPointEdge& edge, float speedlimit) {
    WayPointNode* start = graph.get_node_by_index(edge.startnode_index);
    WayPointNode* end = graph.get_node_by_index(edge.endnode_index);

    // XXX: This means that the Graph is broken
    if(start == NULL || end == NULL) {
      std::cerr<<"ERROR: Graph edges have node indexes that don't exist!\n";
      return Infinite::distance;
    }
    
    if(start == NULL || end == NULL)
      return FLT_MAX;
    
    float distance = 0;
    
    if (start->is_perimeter || end->is_perimeter ||
	start->is_spot || end->is_spot)
      distance = Infinite::distance;
    else
      distance=edge.distance;
    
    float speed=fmin(edge.speed_max, speedlimit);

    float time;
    if (Epsilon::equal(speed,0.0))
      time=Infinite::distance;
    else
      time=distance/speed;

    // penalize for stopping
    if ((start->id.seg != end->id.seg) ||
	(start->id.lane != end->id.lane) ||
	(end->id.pt != start->id.pt+1))
      time+=10.0;
    
    return time;
  }

  double heuristic(const Graph& graph,
		   const waypt_index_t start_id,
		   const waypt_index_t goal_id,
		   float speedlimit)
  {
    //printf("START: %d END: %d\n", start_id, goal_id);
    WayPointNode* start = graph.get_node_by_index(start_id);
    WayPointNode* end = graph.get_node_by_index(goal_id);
    // XXX: This means that the Graph is broken
    if(start == NULL || end == NULL) {
      std::cerr<<"ERROR: Graph edges have node indexes that don't exist!\n";
      return FLT_MAX;
    }
    return time_between_nodes(*start, *end, speedlimit);
  }

  double cost(const Graph& graph,
	      const WayPointEdge& edge,
	      float speedlimit) {

    return time_along_edge(graph, edge, speedlimit);
  }


  void add_to_queue(const Graph& graph,
		    PathPriorityQueue* q,
		    waypt_index_t from_index,
		    PossiblePath& old_possible_path,
		    const waypt_index_t goal_id,
		    float speedlimit,
		    int prev_start_index) {
    WayPointEdgeList edges;
    WayPointNode *from_node = graph.get_node_by_index(from_index);
    WayPointNode *prev_node = graph.get_node_by_index(prev_start_index);

    if(from_node == NULL) {
      std::cerr<<"ERROR: From index ("<<from_index<<") doesn't exist in graph!!\n";
      return;      
    }

    edges = graph.edges_from(from_index);
    
    for(WayPointEdgeList::iterator i = edges.begin(); i != edges.end(); i++) {
      WayPointNode *next_node = graph.get_node_by_index(i->endnode_index);

      if(next_node == NULL) {
	std::cerr<<"ERROR: Next index ("<<i->endnode_index
		 <<") doesn't exist in graph!!\n";
	return;      
      }

      // Don't go into a zone and right back out just to turn around.
      if (prev_node!=NULL && 
	  prev_node->id.lane != 0 && 
	  from_node->id.lane == 0 &&
      	  next_node->id.lane != 0)
	if (!prev_node->is_spot &&
	    !next_node->is_spot)
      	continue;
      if (!i->blocked)
	{
	  //printf("Considering follow edge from %d to %d\n",
	  //	 i->startnode_index, i->endnode_index);
	  WayPointEdgeList new_path(old_possible_path.second);
	  new_path.push_back(*i);
	  PossiblePath pp;
	  
	  // Actual cost so far
	  pp.first.second = old_possible_path.first.second + cost(graph, *i, speedlimit);	
	  // Estimated total cost
	  pp.first.first = pp.first.second + heuristic(graph,
						       i->endnode_index,
						       goal_id, speedlimit);
	  
	  
	  pp.second = new_path;
	  //std::cout<<"ADDING POSSIBLE PATH: "<<std::endl;
	  //print_possible_path(pp, graph);
	  q->push(pp);
	}
    }
  }
  
  WayPointEdgeList astar_search(const Graph& graph,
				waypt_index_t start_id,
				waypt_index_t goal_id,
				float speedlimit) {

    std::map<waypt_index_t,bool> closed;
    closed[start_id] = true;
    PathPriorityQueue* q = new PathPriorityQueue(PossiblePathComparision());
    PossiblePath empty;
    empty.first.first = empty.first.second = 0;
    WayPointEdgeList empty_list;

    if (start_id==goal_id) {
      delete q;
      return empty_list;
    }

    // If checkpoint is parking spot, match on segment to get path
    // into zone
    WayPointNode *goal_node = graph.get_node_by_index(goal_id);

    if(goal_node == NULL) {
      std::cerr<<"ERROR: Goal index ("<<goal_id<<") doesn't exist in graph!!\n";
      delete q;
      return empty_list;      
    }

    // Seed the search....
    empty.second = empty_list;
    add_to_queue(graph, q, start_id, empty, goal_id, speedlimit, -1);
    
    // Main searching loop
    while(!q->empty()) {
      PossiblePath path = q->top();
      q->pop();
      WayPointEdge final_edge = path.second.back();
      if(closed[final_edge.endnode_index])
	continue;

      WayPointNode *final_node = graph.get_node_by_index(final_edge.endnode_index);
      if(final_node == NULL) {
	std::cerr<<"ERROR: Final index ("<<final_edge.endnode_index
		 <<") doesn't exist in graph!!\n";
	delete q;
	return empty_list;      
      }

      if(final_edge.endnode_index == goal_id) {
	delete q;
	return path.second;
      }
      closed[final_edge.endnode_index] = true;
      add_to_queue(graph, q, final_edge.endnode_index, path, goal_id, speedlimit, final_edge.startnode_index);
    }
    delete q;
    return empty_list;
  }

};
