/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007, Tarun Nimmagadda
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

//The University of Texas at Austin
//This file defines the Mission Data Structure

//Speeds are stored in the Graph object, so this Class will
//only store the Checkpoints that need to be hit. 

#ifndef __MISSION_h__
#define __MISSION_h__

#include <cstdlib>
#include <deque>
#include <fstream>
#include <art_map/RNDF.h>
#include <art_map/Graph.h>

class Mission{
 public:	
  Mission(const MDF& mdf);
  Mission(const Mission& that);
  ~Mission()
    {
      clear();
    };

  bool compare(const Mission &that);


  void print();
  //Removes the Current Checkpoint from the list
  bool nextPoint();
  int current_checkpoint_id();
  int next_checkpoint_id();
  ElementID current_checkpoint_elementid();
  ElementID next_checkpoint_elementid();
  int remaining_points(){return checkpoint_ids.size();}
  //Returns whether the MDF was valid match to the graph
  bool populate_elementid(const Graph& graph);

  // Hooks to save, reload Mission state
  void save(const char* fName);
  bool load(const char* fName, const Graph& graph);
  void clear();

  //NOTE: Checkpoints are stored in reverse order
  std::deque<int> checkpoint_ids;
  //Element ID is stored here. 
  std::deque<ElementID> checkpoint_elementid;
};

/*int parse_integer(std::string line, std::string token, bool& valid);*/
ElementID parse_elementid(std::string line, bool& valid);
#endif
