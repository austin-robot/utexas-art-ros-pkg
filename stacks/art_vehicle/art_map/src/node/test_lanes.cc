/*
 *  utility to print RNDF lane information
 *
 *  Copyright (C) 2005, 2010, Austin Robot Technology
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

#include <art_msgs/ArtLanes.h>
#include <art_map/euclidean_distance.h>
#include <art_map/MapLanes.h>
#include <art_map/zones.h>
#include <art_map/ZoneOps.h>

/** @file

 @brief utility to print RNDF lane information.

 @author Jack O'Quin, Patrick Beeson

*/

// TODO These should go in a header somewhere
static int32_t bottom_left  = art_msgs::ArtQuadrilateral::bottom_left;
static int32_t top_left     = art_msgs::ArtQuadrilateral::top_left;
static int32_t top_right    = art_msgs::ArtQuadrilateral::top_right;
static int32_t bottom_right = art_msgs::ArtQuadrilateral::bottom_right;


// default parameters
char *pname;				// program name
bool print_polys = false;
bool output_polys = false;
bool make_image = false;
bool with_trans = false;
int verbose = 0;
char *rndf_name;
float poly_size=-1;

double gps_latitude = 0.0;
double gps_longitude = 0.0;

RNDF *rndf = NULL;
Graph* graph = NULL;

float centerx,centery;
#define CMD "rndf_lanes: "		// message prefix

/** build road map graph from Road Network Definition File */  
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

      if (gps_latitude == 0 &&
	  gps_longitude == 0)
	{
	  gps_latitude = graph->nodes[0].ll.latitude;
	  gps_longitude = graph->nodes[0].ll.longitude;
	  graph->find_mapxy();

	  float min_x=FLT_MAX;
	  float min_y=FLT_MAX;
	  float max_x=-FLT_MAX;
	  float max_y=-FLT_MAX;
	  double initial_UTM_x;
	  double initial_UTM_y;
	  char zone[255];
	  
          UTM::LLtoUTM(graph->nodes[0].ll.latitude, 
                       graph->nodes[0].ll.longitude, 
                       initial_UTM_y,	      
                       initial_UTM_x,
                       zone);
	  
	  ZonePerimeterList zones =
            ZoneOps::build_zone_list_from_rndf(*rndf, *graph);
	  
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
	  
          UTM::UTMtoLL(centery,centerx,zone,
                       centerlat,centerlong);
	  
	  std::cout << "Center for RNDF is at lat/long: "<<
	    std::setprecision(8)<<
	    centerlat<<", "<<centerlong<<" "<<zone<<std::endl;

    std::cout << "X,Y coordiantes for map: \n"
              << std::setprecision(8)<< "  min=(" << min_x << "," << max_x << ")\n  max=(" << max_x << "," << max_y << ")\n  centre=(" << (max_x+min_x)/2  << "," << (max_y+min_y)/2  << ")\n";
    std::cout << std::setprecision(8) << "  Width=" << max_x-min_x << " m\n  Height=" << max_y-min_y << " m\n";

    
	  gps_latitude = centerlat;
	  gps_longitude = centerlong;
	  
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

/** parse command line arguments */
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
	  gps_latitude = atof(optarg);
	  break;

	case 'y':
	  gps_longitude = atof(optarg);
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

/** Print polygon data in human-readable format */
void PrintPolygons(const art_msgs::ArtLanes &ldata)
{
  uint32_t count = ldata.polygons.size();

  std::cout << "#: [start,end] <contains> {heading} Sv Tv (p1),(p2),(p3),(p4)"
	    << std::endl;

  for (unsigned i = 0; i < count; ++i)
    {
      std::cout << ldata.polygons[i].poly_id << ": ";

      std::cout << "[" << ldata.polygons[i].start_way.seg
                << "." << ldata.polygons[i].start_way.lane
                << "." << ldata.polygons[i].start_way.pt
		<< "," << ldata.polygons[i].end_way.seg
                << "." << ldata.polygons[i].end_way.lane
                << "." << ldata.polygons[i].end_way.pt
                << "] <";

      if (ldata.polygons[i].contains_way)
	{
	  std::cout << ldata.polygons[i].start_way.seg
                    << "." << ldata.polygons[i].start_way.lane
                    << "." << ldata.polygons[i].start_way.pt ;
	}
      std::cout << std::fixed << std::setprecision(3)
		<< "> {" << ldata.polygons[i].heading << "}"
		<< " S" << (bool) ldata.polygons[i].is_stop
		<< " T" << (bool) ldata.polygons[i].is_transition
		<< " ("
                << ldata.polygons[i].poly.points[bottom_left].x
                << "," << ldata.polygons[i].poly.points[bottom_left].y
		<< "),("
                << ldata.polygons[i].poly.points[top_left].x
                << "," << ldata.polygons[i].poly.points[top_left].y
		<< "),("
                << ldata.polygons[i].poly.points[top_right].x
                << "," << ldata.polygons[i].poly.points[top_right].y
		<< "),("
                << ldata.polygons[i].poly.points[bottom_right].x
                << "," << ldata.polygons[i].poly.points[bottom_right].y
		<< ")" << std::endl;
    }
}

/** write polygon data to space-delimited file */
void OutputPolygons(const art_msgs::ArtLanes &ldata)
{
  uint32_t count = ldata.polygons.size();

  std::ofstream a_file ( "polys.points" );
  for (unsigned i = 0; i < count; ++i)
    {
      a_file << ldata.polygons[i].poly.points[bottom_left].x << " "
             << ldata.polygons[i].poly.points[bottom_left].y << " "
             << ldata.polygons[i].poly.points[top_left].x << " "
             << ldata.polygons[i].poly.points[top_left].y << " "
             << ldata.polygons[i].poly.points[top_right].x << " "
             << ldata.polygons[i].poly.points[top_right].y << " "
             << ldata.polygons[i].poly.points[bottom_right].x << " "
             << ldata.polygons[i].poly.points[bottom_right].y << std::endl;
    }
  a_file.close();
}

/** main program */
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
  art_msgs::ArtLanes lanedata;
  rc = mapl->getAllLanes(&lanedata);
  if (rc < 0)
    {
      std::cout << "error getting all polygons!  " << -rc
                << " too many polygons to fit in Player message)\n";
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
