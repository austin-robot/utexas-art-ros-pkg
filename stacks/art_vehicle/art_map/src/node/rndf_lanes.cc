/*
 *  Utility to print RNDF lane information.
 *
 *  Copyright (C) 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>			// for sleep
#include <getopt.h>
#include <signal.h>
#include <string.h>

#include <iostream>
#include <fstream>
#include <iomanip>

#include <ros/ros.h>

#include <art_map/euclidean_distance.h>
#include <art_map/MapLanes.h>
#include <art_map/zones.h>

//#include <art/lanes.h>
//#include <art/ZoneOps.h>

// default parameters
char *pname;				// program name
bool print_polys = false;
bool output_polys = false;
bool make_image = false;
bool with_trans = false;
int verbose = 0;
char *rndf_name;
float poly_size=-1;

gps_info pos = {0, 0};
RNDF *rndf = NULL;
Graph* graph = NULL;

float centerx,centery;
#define CMD "rndf_lanes: "		// message prefix
  
bool build_RNDF()
{

  rndf = new RNDF(rndf_name);
  
  if (!rndf->is_valid) {
    std::cerr << "RNDF not valid\n";
    return false;;
  }

  graph = new Graph();

  rndf->populate_graph(*graph);

  // pos.gps_latitude=graph.nodes[0].ll.latitude;
  // pos.gps_longitude=graph.nodes[0].ll.longitude;

  if (graph->rndf_is_gps())
    {
      
      std::cout << "RNDF uses GPS waypoints\n";

      if (pos.gps_latitude==0 &&
	  pos.gps_longitude==0)
	{
	  pos.gps_latitude=graph->nodes[0].ll.latitude;
	  pos.gps_longitude=graph->nodes[0].ll.longitude;
	  graph->find_mapxy();

	  float min_x=FLT_MAX;
	  float min_y=FLT_MAX;
	  float max_x=-FLT_MAX;
	  float max_y=-FLT_MAX;
	  double initial_UTM_x;
	  double initial_UTM_y;
	  char zone[255];
	  
	  LLtoUTM(graph->nodes[0].ll.latitude, 
		  graph->nodes[0].ll.longitude, 
		  initial_UTM_y,	      
		  initial_UTM_x,
		  zone);
	  
	  ZonePerimeterList zones = ZoneOps::build_zone_list_from_rndf(*rndf, *graph);
	  
	  for(unsigned j = 0; j < zones.size(); j++) {
	    ZonePerimeter zzone = zones[j];
	    for(unsigned i = 0; i < zzone.perimeter_points.size(); i++) {
	      if (zzone.perimeter_points[i].map.x < min_x)
		min_x=zzone.perimeter_points[i].map.x;
	      if (zzone.perimeter_points[i].map.y < min_y)
		min_y=zzone.perimeter_points[i].map.y;
	      if (zzone.perimeter_points[i].map.x > max_x)
		max_x=zzone.perimeter_points[i].map.x;
	      if (zzone.perimeter_points[i].map.y > max_y)
		max_y=zzone.perimeter_points[i].map.y;
	    }
	  }
	  
	  for (uint i=0;i<graph->nodes_size;i++)
	    {
	      if (graph->nodes[i].map.x < min_x)
		min_x=graph->nodes[i].map.x;
	      if (graph->nodes[i].map.y < min_y)
		min_y=graph->nodes[i].map.y;
	      if (graph->nodes[i].map.x > max_x)
		max_x=graph->nodes[i].map.x;
	      if (graph->nodes[i].map.y > max_y)
		max_y=graph->nodes[i].map.y;
	    }
      
	  centerx=(max_x+min_x)/2 - graph->nodes[0].map.x + initial_UTM_x;
	  centery=(max_y+min_y)/2 - graph->nodes[0].map.y + initial_UTM_y;
	  
	  double centerlat, centerlong;
	  
	  UTMtoLL(centery,centerx,zone,
		  centerlat,centerlong);
	  
	  std::cout << "Center for RNDF is at lat/long: "<<
	    std::setprecision(8)<<
	    centerlat<<", "<<centerlong<<" "<<zone<<std::endl<<std::endl;
	  
	  pos.gps_latitude=centerlat;
	  pos.gps_longitude=centerlong;
	  
	}
      graph->find_mapxy();
      
    }
  else
    {
      std::cout << "RNDF does not use GPS waypoints\n";
      graph->xy_rndf();
    }
  
  
  return true;
}

void parse_args(int argc, char *argv[])
{
  bool print_usage = false;
  const char *options = "hiops:tx:y:v";
  int opt = 0;
  int option_index = 0;
  struct option long_options[] = 
    { 
      { "help", 0, 0, 'h' },
      { "image", 0, 0, 'i' },
      { "latitude", 1, 0, 'x' },
      { "longitude", 1, 0, 'y' },
      { "size", 1, 0, 's' },
      { "print", 0, 0, 'p' },
      { "output-points", 0, 0, 'o' },
      { "trans", 0, 0, 't' },
      { "verbose", 0, 0, 'v' },
      { 0, 0, 0, 0 }
    };

  /* basename $0 */
  pname = strrchr(argv[0], '/');
  if (pname == 0)
    pname = argv[0];
  else
    pname++;

  opterr = 0;
  while ((opt = getopt_long(argc, argv, options,
			    long_options, &option_index)) != EOF)
    {
      switch (opt)
	{
	case 'i':
	  make_image = true;
	  break;

	case 'p':
	  print_polys = true;
	  break;

	case 'o':
	  output_polys = true;
	  break;

	case 'v':
	  ++verbose;
	  break;

	case 'x':
	  pos.gps_latitude = atof(optarg);
	  break;

	case 'y':
	  pos.gps_longitude = atof(optarg);
	  break;
	 
	case 's':
	  poly_size = atof(optarg);
	  break;

	case 't':
	  with_trans = true;
	  break;

	default:
	  fprintf(stderr, "unknown option character %c\n",
		  optopt);
	  /*fallthru*/
	case 'h':
	  print_usage = true;
	}
    }

  if (print_usage || optind >= argc)
    {
      fprintf(stderr,
	      "usage: %s [options] RNDF_name\n\n"
	      "    Display RNDF lane information.  Possible options:\n"
	      "\t-h, --help\tprint this message\n"
	      "\t-i, --image\tmake .pgm image of polygons\n"
	      "\t-y, --latitude\tinitial pose latitude\n"
	      "\t-x, --longitude\tinitial pose longitude\n"
	      "\t-s, --size\tmax polygon size\n"
	      "\t-p, --print\tprint polygons to stdout\n"
	      "\t-o, --output-points\tprint polygon points to polys.points\n"
	      "\t-v, --verbose\tprint verbose messages\n",
	      pname);
      exit(9);
    }

  rndf_name = argv[optind];
}

void PrintPolygons(const lanes_state_msg_t &ldata)
{
  uint32_t count = ldata.poly_count;

  if (count > MAX_LANE_ELEMENTS)
    {
      std::cerr << "too many polygons received ("
		<< count << ")" << std::endl;
      return;
    }

  std::cout << "#: [start,end] <contains> {heading} Sv Tv (p1),(p2),(p3),(p4)"
	    << std::endl;

  for (unsigned i = 0; i < count; ++i)
    {
      std::cout << ldata.poly[i].poly_id << ": ";

      std::cout << "[" << ldata.poly[i].start_way.name().str
		<< "," << ldata.poly[i].end_way.name().str << "] <";

      if (ldata.poly[i].contains_way)
	{
	  std::cout << ldata.poly[i].start_way.name().str;
	}
      std::cout << std::fixed << std::setprecision(3)
		<< "> {" << ldata.poly[i].heading << "}"
		<< " S" << ldata.poly[i].is_stop
		<< " T" << ldata.poly[i].is_transition
		<< " ("
		<< ldata.poly[i].p1.x << "," << ldata.poly[i].p1.y
		<< "),("
		<< ldata.poly[i].p2.x << "," << ldata.poly[i].p2.y
		<< "),("
		<< ldata.poly[i].p3.x << "," << ldata.poly[i].p3.y
		<< "),("
		<< ldata.poly[i].p4.x << "," << ldata.poly[i].p4.y
		<< ")" << std::endl;
    }
}


void OutputPolygons(const lanes_state_msg_t &ldata)
{
  uint32_t count = ldata.poly_count;

  if (count > MAX_LANE_ELEMENTS)
    {
      std::cerr << "too many polygons received ("
		<< count << ")" << std::endl;
      return;
    }

  std::ofstream a_file ( "polys.points" );
  for (unsigned i = 0; i < count; ++i)
    {
      
      a_file << ldata.poly[i].p1.x << " " << ldata.poly[i].p1.y << " "
	     << ldata.poly[i].p2.x << " " << ldata.poly[i].p2.y << " "
	     << ldata.poly[i].p3.x << " " << ldata.poly[i].p3.y << " "
	     << ldata.poly[i].p4.x << " " << ldata.poly[i].p4.y
	     << std::endl;
    }
  a_file.close();
}


int main(int argc, char *argv[]) 
{
  int rc;

  parse_args(argc, argv);

  if (!build_RNDF())
    {
      std::cerr << "RNDF not valid\n";
      return 1;
    }

  MapLanes *mapl = new MapLanes(verbose);

  rc = mapl->MapRNDF(graph,poly_size);
  if (rc != 0)
    {
      std::cout << "cannot process RNDF! (error code " << rc <<")\n";
      return 1;
    }

  // Fill in lanedata with all polygons within range of our
  // current position.
  lanes_state_msg_t lanedata;
  rc = mapl->getAllLanes(&lanedata);
  if (rc < 0)
    {
      std::cout << "error getting all polygons!  "<<-rc << " too many polygons to fit in Player message)\n";
    }
  
  if (print_polys)
    PrintPolygons(lanedata);

  if (output_polys)
    OutputPolygons(lanedata);
  if (make_image) {
    ZonePerimeterList zones = ZoneOps::build_zone_list_from_rndf(*rndf, *graph);
    mapl->SetGPS(centerx,centery);
	  mapl->testDraw(with_trans, zones);
  }
  return rc;
}
