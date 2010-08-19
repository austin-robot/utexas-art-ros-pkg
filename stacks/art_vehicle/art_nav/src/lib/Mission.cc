/*
 *  Copyright (C) 2007, Tarun Nimmagadda
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

//The University of Texas, Austin

//Defines the Mission Data Structure

#include <art_nav/Mission.h>

//Copy Checkpoints in reverse order
Mission::Mission(const MDF& mdf){
  int previous_id = -1;
  for (int i = 0; i < (int)mdf.checkpoint_ids.size(); i++){
    int checkpoint_id = mdf.checkpoint_ids[i];
    if (previous_id != checkpoint_id) 
      checkpoint_ids.push_back(checkpoint_id);
    previous_id = checkpoint_id;
  }
};


Mission::Mission(const Mission& that){
  checkpoint_ids = std::deque<int>(that.checkpoint_ids);
  checkpoint_elementid = std::deque<ElementID>(that.checkpoint_elementid);
};

bool Mission::compare(const Mission &that){
  bool ids_match = checkpoint_ids == that.checkpoint_ids;
  bool eids_match = true;
  for (uint i = 0; i < (uint) checkpoint_elementid.size(); i++)
    eids_match = eids_match && checkpoint_elementid[i] == that.checkpoint_elementid[i];
  //printf("CHECKPOINT IDS %s\n", ids_match?"MATCH":"DO NOT MATCH");
  //printf("CHECKPOINT Element IDS %s\n", eids_match?"MATCH":"DO NOT MATCH");
  return (ids_match && eids_match);
};

//Populate checkpoint index field for convinience
bool Mission::populate_elementid(const Graph& graph)
{
  std::deque<int>::iterator itr;
  WayPointNode wpn;
  for(itr = checkpoint_ids.begin(); itr != checkpoint_ids.end(); itr++){
    for(uint i = 0; i < graph.nodes_size; i++){
      if (graph.nodes[i].checkpoint_id == *itr) {
	checkpoint_elementid.push_back(graph.nodes[i].id);
	break;
      }
    }
  }
  if (checkpoint_elementid.size() != checkpoint_ids.size()){
    printf("[%d] != [%d]\n", checkpoint_elementid.size(), 
	   checkpoint_ids.size());
    return false;
  }
  else
    return true;
};

bool Mission::nextPoint(){
  if (checkpoint_ids.empty())
    return false;

  checkpoint_ids.pop_front();
  checkpoint_elementid.pop_front();
  
  if (checkpoint_ids.empty())
    return false;

  return true;
};

int Mission::current_checkpoint_id(){
  if (checkpoint_ids.empty())
    return -1;
  else return checkpoint_ids.front();
};

int Mission::next_checkpoint_id(){
  int size = (int)checkpoint_ids.size();
  if ( size < 2)
    return -1;
  else return checkpoint_ids[1];
};

ElementID Mission::current_checkpoint_elementid(){
  if (checkpoint_elementid.empty())
    return ElementID();
  else return checkpoint_elementid.front();
};

ElementID Mission::next_checkpoint_elementid(){
  int size = (int)checkpoint_elementid.size();  
  if (size < 2)
    return ElementID();
  else return checkpoint_elementid[1];
};

void Mission::save(const char* fName){
  FILE* f = fopen(fName,"wb");
  fprintf(f, "MISSION-STATE\n");
  fprintf(f, "Number %d\n", checkpoint_ids.size());
  for (int i = 0; i < (int)checkpoint_ids.size(); i++)
    fprintf(f, "Id %d\n", checkpoint_ids[i]);
  for (int i = 0; i < (int)checkpoint_elementid.size(); i++){
    ElementID eid = checkpoint_elementid[i];
    fprintf(f, "ElementID %d.%d.%d\n", eid.seg, eid.lane, eid.pt);
  }
  fclose (f);
}

bool Mission::load(const char* fName, const Graph& graph){
  int number_of_checkpoints = 0;
  std::ifstream mission_file;
  mission_file.open(fName);
  if (!mission_file){
    printf("Error in opening Mission Log file\n");
    return false;
  }
  
  int line_number = 0;
  bool valid = true;
  std::string lineread;
  while(getline(mission_file, lineread) ) // Read line by line
    {
      line_number++;
      std::string token;
      char token_char [lineread.size()+1];
      //Read in one line
      sscanf(lineread.c_str(), "%s", token_char);
      token.assign(token_char);

      //      printf("Token: |%s|\n", token.c_str());
      
      if (line_number == 1){
	if (!(token.compare("MISSION-STATE") == 0)) return false;
      }
      
      else if (line_number == 2){
	if (!(token.compare("Number") == 0)) {return false;}
	else {
	  number_of_checkpoints = 
	    parse_integer(lineread, std::string("Number"), valid);
	}
	if (!valid) return false;
      }
      else if (token.compare("Id") == 0){
	int id = parse_integer(lineread, std::string("Id"), valid);
	if (!valid) return false;
	else checkpoint_ids.push_back(id);
      }
      else if (token.compare("ElementID") == 0){
	ElementID eid = parse_elementid(lineread, valid);
	if (!valid) return false;
	else checkpoint_elementid.push_back(eid);
      }
      else return false;
    }
  if (line_number < 4) return false;
  else if ((int)checkpoint_ids.size() != number_of_checkpoints) 
    return false;
  else if ((int)checkpoint_elementid.size() != number_of_checkpoints)
    return false;
  
  for (uint j=0; j<checkpoint_ids.size(); j++)
    for(uint i = 0; i < graph.nodes_size; i++){
      if (graph.nodes[i].checkpoint_id == checkpoint_ids.at(j)) 
	if (checkpoint_elementid.at(j)!=graph.nodes[i].id)
	  {
	    printf("Checkpoint has improper element ID\n");
	    return false;
	  }
    }
  
  return true;
}

void Mission::clear(){
  checkpoint_ids.clear();
  checkpoint_elementid.clear();
}

void Mission::print(){
  printf("\n MISSION: \n");
  std::deque<int>::iterator itr;
  std::deque<ElementID>::iterator eid_itr;
  if (checkpoint_ids.size() > 0){
    printf("Checkpoint IDs:");
    for(itr = checkpoint_ids.begin(); itr != checkpoint_ids.end()-1; itr++){
      printf("[%d]->", *itr); 
    }
    if (itr == checkpoint_ids.end() - 1){
      printf("[%d]", *itr); 
      printf("\n");
    }
  }
  else {
    printf("Checkpoint ID List is empty\n");  
  }
  
  if (checkpoint_elementid.size() > 0){
    printf("Checkpoint ElementID");
    for(eid_itr = checkpoint_elementid.begin(); 
	eid_itr != checkpoint_elementid.end()-1; eid_itr++){
      printf("[%d.%d.%d]->", eid_itr->seg, eid_itr->lane, eid_itr->pt); 
    }
    if (eid_itr == checkpoint_elementid.end() - 1){
      printf("[%d.%d.%d]", eid_itr->seg, eid_itr->lane, eid_itr->pt); 
      printf("\n");
    }
  }
  else {
    printf("Checkpoint Element ID List is empty\n");  
  }
};

ElementID parse_elementid(std::string line, bool& valid)
{
  int seg = -1;
  int lane = -1;
  int pt = -1;
  if (!sscanf(line.c_str(), "ElementID %d.%d.%d", &seg, &lane, &pt))
    {
      valid=false;
    }
  return ElementID(seg, lane, pt); 
};
