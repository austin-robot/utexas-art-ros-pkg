/* -*- mode: C++ -*-
 *
 *  Navigator zone controller
 *
 *  Copyright (C) 2007, Mickey Ristroph
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __VORONOI_ZONE_HH__
#define __VORONOI_ZONE_HH__
#include "parking.h"
#include "follow_lane.h"

class Safety;
class Halt;
class ZoneManager;

class VoronoiZone: public Controller
{
public:

  VoronoiZone(Navigator *navptr, int _verbose);
  ~VoronoiZone();
  void configure();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:
  // state
  bool in_fake_zone;

  // .cfg variables
  float zone_speed_limit;
  float zone_safety_radius;
  float zone_perimeter_sample;
  float fake_zone_border;
  float fake_zone_included_obstacle_radius;
  float zone_evg_thin_scale;
  int zone_grid_max_cells;
  bool zone_use_voronoi;
  bool zone_avoid_obstacles;
  bool zone_write_graph_to_tmp;
  bool zone_write_poly_to_tmp;
  bool zone_write_obstacles_to_tmp;
  float zone_aim_point;

  Safety *safety;
  Halt *halt;

  Controller::result_t set_heading(pilot_command_t &pcmd);

  rotate_translate_transform trans;

  PolyOps pops;
  ZoneManager *zmanager;

  PARK_control *park;

  
  FollowLane* follow_lane;
  float lastYaw;
  mapxy_list_t spot_points;
};

#endif // __VORONOI_ZONE_HH__
