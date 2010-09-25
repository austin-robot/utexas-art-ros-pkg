/*
 *  Navigator zone controller
 *
 *  Copyright (C) 2007, 2010, Mickey Ristroph
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <art/Graph.h>
#include <art/ZoneOps.h>
#include <art/euclidean_distance.h>

#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "obstacle.h"
#include "voronoi_zone.h"
#include <art/steering.h>
#include "safety.h"
#include "halt.h"

VoronoiZone::VoronoiZone(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose), in_fake_zone(false)
{
  zmanager = NULL;
  safety = new Safety(navptr, _verbose, 1);
  halt = new Halt(navptr, _verbose);
  park = new PARK_control(navptr, _verbose);
  follow_lane = new FollowLane(navptr, _verbose);
}

VoronoiZone::~VoronoiZone()
{
  delete safety;
  delete park;
  delete halt;
  delete follow_lane;
  if(zmanager != NULL)
    delete zmanager;
}

// configuration method
void VoronoiZone::configure(ConfigFile* cf, int section)
{
  // maximum speed when inside a zone
  zone_speed_limit = cf->ReadFloat(section, "zone_speed_limit", 1.0);
  ART_MSG(2, "\tspeed limit in a zone is %.1f m/s", zone_speed_limit);

  zone_safety_radius = cf->ReadFloat(section, "zone_safety_radius", 2.0);
  ART_MSG(2, "\tzone_safety_radius is %.1f m", zone_safety_radius);

  zone_perimeter_sample = cf->ReadFloat(section, "zone_perimeter_sample", 1.0);
  ART_MSG(2, "\tzone_perimeter_sample is %.1f m", zone_perimeter_sample);

  fake_zone_border = cf->ReadFloat(section, "fake_zone_border", 10.0);
  ART_MSG(2, "\tfake_zone_border is %.1f m", fake_zone_border);

  fake_zone_included_obstacle_radius =
    cf->ReadFloat(section, "fake_zone_included_obstacle_radius", 30.0);
  ART_MSG(2, "\tfake_zone_included_obstacle_radius is %.1f m",
	  fake_zone_included_obstacle_radius);

  zone_evg_thin_scale = cf->ReadFloat(section, "zone_evg_thin_scale", 1.0);
  ART_MSG(2, "\tzone_evg_thin_scale is %.1f m", zone_evg_thin_scale);

  zone_grid_max_cells = cf->ReadInt(section, "zone_grid_max_cells", 1000);
  ART_MSG(2, "\tzone_grid_max_cells is %d", zone_grid_max_cells);

  zone_use_voronoi = cf->ReadBool(section, "zone_use_voronoi", true);
  ART_MSG(2, "\tzone_use_voronoi is %d", zone_use_voronoi);

  zone_avoid_obstacles = cf->ReadBool(section, "zone_avoid_obstacles", true);
  ART_MSG(2, "\tzone_avoid_obstacles is %d", zone_avoid_obstacles);

  zone_write_graph_to_tmp = cf->ReadBool(section, "zone_write_graph_to_tmp", true);
  ART_MSG(2, "\tzone_write_graph_to_tmp is %d", zone_write_graph_to_tmp);

  zone_write_poly_to_tmp = cf->ReadBool(section, "zone_write_poly_to_tmp", false);
  ART_MSG(2, "\tzone_write_poly_to_tmp is %d", zone_write_poly_to_tmp);

  zone_write_obstacles_to_tmp = cf->ReadBool(section, "zone_write_obstacles_to_tmp", false);
  ART_MSG(2, "\tzone_write_obstacles_to_tmp is %d", zone_write_obstacles_to_tmp);

  zone_aim_point = cf->ReadFloat(section, "zone_aim_point", 8.0);
  ART_MSG(2, "\tzone_aim_point is %.3f", zone_aim_point);
  
  safety->configure(cf, section);
  halt->configure(cf, section);
  park->configure(cf, section);
  follow_lane->configure(cf, section);
}

// go to next zone way-point
//
// returns: OK.
//
Controller::result_t VoronoiZone::control(pilot_command_t &pcmd)
{
  result_t result = OK;

  ART_MSG(8, "ENTERING zone::control()");
  if (verbose >= 2)
    {
      ART_MSG(5, "Go to zone waypoint %s",
	      order->waypt[1].id.name().str);
    }
  
  // limit speed inside zones
  pcmd.velocity = fminf(pcmd.velocity, zone_speed_limit);
  
  navdata->reverse=false;
  
  // return desired heading for this cycle
  result=set_heading(pcmd);

  // TODO: implement obstacle evasion if Unsafe or Blocked
  
  pcmd.velocity=fminf(zone_speed_limit,pcmd.velocity);

  
  if (!Epsilon::equal(pcmd.yawRate,0.0) &&
      !Epsilon::equal(lastYaw,0.0))
    if (fabsf(pcmd.yawRate-lastYaw) > Steering::maximum_yaw/2
	&&
	((pcmd.yawRate < 0) != (lastYaw < 0)))
      pcmd.velocity=0;
  
  lastYaw=pcmd.yawRate;
  
  
  trace("voronoi_zone controller", pcmd, result);
  ART_MSG(8, "EXITING zone::control()");
  return result;
}

// reset all subordinate controllers
void VoronoiZone::reset(void)
{
  trace_reset("VoronoiZone");
  in_fake_zone = false;
  lastYaw=0.0;
  safety->reset();
  halt->reset();
  park->reset();
  follow_lane->reset();
  if(zmanager != NULL)
    delete zmanager;
  zmanager = NULL;

  spot_points.clear();
}

// set heading to next way-point
Controller::result_t VoronoiZone::set_heading(pilot_command_t &pcmd)
{
  static bool parking=false;

  if (order->waypt[1].id.lane!=0 &&
      !order->waypt[1].is_spot)
    if (course->lane_waypoint_reached())
      return Finished;
  
  posetype way_pose;
  
  if (!order->waypt[1].is_spot ||
      (order->waypt[1].is_spot && order->waypt[2].is_spot &&
       order->waypt[1].id.pt==1 && order->waypt[2].id.pt==2))
    // normal situation -- set pose at spot
    {
      way_pose=posetype(order->waypt[1].map.x,order->waypt[1].map.y,
			atan2f(order->waypt[2].map.y-order->waypt[1].map.y,
			       order->waypt[2].map.x-order->waypt[1].map.x));
// #ifdef NQE
//       if (order->waypt[1].id.seg==43 && 
// 	  order->waypt[2].id.seg==4) {
// 	way_pose=posetype(order->waypt[1].map.x,order->waypt[1].map.y,
// 			  atan2f(order->waypt[2].map.y-order->waypt[1].map.y,
// 				 order->waypt[2].map.x-order->waypt[1].map.x));
// 	way_pose.y-=8;
// 	//	way_pose.theta-=DTOR(45);
//       }
// #endif
    }
  
  else
    // pulling into spot, put point at front of spot
    if (order->waypt[1].is_spot && order->waypt[2].is_spot &&
	order->waypt[1].id.pt==2 && order->waypt[2].id.pt==1)
      way_pose = posetype(order->waypt[1].map.x,order->waypt[1].map.y,
			  atan2f(order->waypt[1].map.y-order->waypt[2].map.y,
				 order->waypt[1].map.x-order->waypt[2].map.x));
    else 
      // pulling out of spot, put point at front of spot
      way_pose=posetype(order->waypt[0].map.x,order->waypt[0].map.y,
			atan2f(order->waypt[0].map.y-order->waypt[1].map.y,
			       order->waypt[0].map.x-order->waypt[1].map.x));
  
  MapXY goal;

  trans.find_transform(posetype(0,0,0),way_pose);
  

  if (order->waypt[1].is_spot)
    goal=MapXY(trans.apply_transform(posetype(-zone_aim_point,0,0)));
  else
    if (order->waypt[1].id.lane==0)
      goal=MapXY(trans.apply_transform(posetype(-zone_aim_point,0,0)));
    else
      goal=MapXY(trans.apply_transform(posetype(zone_aim_point,0,0)));

  MapXY aim = goal;
  
  if (Euclidean::DistanceTo(goal,estimate->pos) < zone_aim_point ||
      (parking  && Euclidean::DistanceTo(goal,estimate->pos) < zone_aim_point+5))
    {
      if (order->waypt[1].is_spot)
	{
	  parking=true;
	  ART_MSG(3, "Near goal, entering Parking (%.3f %.3f, %.3f %.3f)",
		  estimate->pos.px, estimate->pos.py,
		  goal.x, goal.y);
	  result_t park_result=park->control(pcmd, obstacle->lasers->all_obstacle_list, false);
	  if (park_result==Finished)
	    {
	      park->reset();
	      parking=false;
	    }
	  return park_result;
	}
      else {
	ART_MSG(3, "Near goal, entering Follow lane (%.3f %.3f, %.3f %.3f)",
		estimate->pos.px, estimate->pos.py,
		goal.x, goal.y);
	pilot_command_t pcopy=pcmd;
	result_t res=follow_lane->control(pcopy);
	if (res < Unsafe)
	  {
	    pcmd=pcopy;
	    return res;
	  }
      }
    }

  course->no_waypoint_reached();

  if (course->spot_ahead())
    spot_points=course->calculate_spot_points();
  else if (!course->curr_spot())
    spot_points.clear();

  mapxy_list_t obs_points=obstacle->lasers->all_obstacle_list;

  for (uint i=0;i<spot_points.size();i++)
    obs_points.push_back(spot_points.at(i));
  
  if(zone_use_voronoi) {
    ART_MSG(8, "ENTERING zone::set_heading()");
    
    MapXY start(estimate->pos);
    MapXY end(aim);
   
    WayPointNodeList nodes;
    bool use_zone_manager = false;
    if(use_zone_manager) {
      bool is_a_zone = ZoneOps::is_a_zone_id(course->zones,
					     order->waypt[1].id.seg);
      if(zmanager == NULL ||
	 (is_a_zone && (order->waypt[1].id.seg != zmanager->starting_id.seg)) ||
	 (!is_a_zone && (order->waypt[0].id != zmanager->starting_id))) {
	if(zmanager != NULL) {
	  ART_MSG(1, "Deallocating old zone (seg=%d)",
		  zmanager->starting_id.seg);
	  delete zmanager;	
	}
	
	ZonePerimeter zone =
	  ZoneOps::get_zone_by_id(course->zones, order->waypt[1].id.seg);
	
	if(zone.perimeter_points.size() < 3) {
	  std::vector<MapXY> points_to_include;
	  points_to_include.push_back(start);
	  points_to_include.push_back(end);
	  
	  for(unsigned i = 0;
	      i < obstacle->lasers->all_obstacle_list.size(); i++) {
	    if(Euclidean::DistanceTo
	       (start, obstacle->lasers->all_obstacle_list.at(i)) <
	       fake_zone_included_obstacle_radius) {
	      points_to_include.push_back
		(obstacle->lasers->all_obstacle_list.at(i));
	    }
	  }
	  
	  zone = ZoneOps::build_fake_zone(points_to_include, fake_zone_border);
	
	  if(!in_fake_zone) {
	    ART_MSG(1, "Making a bogus zone around us and the goal!");
	    in_fake_zone = true;
	  }
	} else {
	  if(in_fake_zone) {
	    ART_MSG(1, "In a zone with at least three perimeter points.");
	    in_fake_zone = false;
	  }
	}
	
	MapXY ur = zone.perimeter_points[0].map;
	MapXY ll = ur;
	ZoneOps::expand_bounding_box_of_waypoints(zone.perimeter_points, ll, ur);
	float width = fabs(ur.x-ll.x);
	float height = fabs(ur.y-ll.y);
	
	ART_MSG(1, "Allocating a new zone (seg=%d)", order->waypt[0].id.seg);
	ART_MSG(1, "Zone is %.1f by %.1f meters.", width, height);
	
	zmanager = new ZoneManager(zone,
				   zone_safety_radius,
				   zone_evg_thin_scale,
				   zone_grid_max_cells,
				   zone_write_graph_to_tmp,
				   order->waypt[0].id,
				   ll,
				   ur);
      }
      
      ART_MSG(8, "Getting a path (%.3f, %.3f) -> (%.3f, %.3f)",
	      start.x, start.y, end.x, end.y);
      
      
      ART_MSG(8, "ENTERING path_through_zone()");
      nodes = zmanager->path_through_zone(obs_points,
					  start, end);
      
    }
    else {
      ZonePerimeter zone =
	ZoneOps::get_zone_by_id(course->zones, order->waypt[1].id.seg);
      
      bool use_lane_based_zones = false;
      if(use_lane_based_zones) {
	if(order->waypt[1].id.seg == 10 && order->waypt[0].id.seg == 10) {
	  std::vector<MapXY> p =
	    pops.getRoadPerimeterPoints(course->polygons,
					ElementID(10,1,6),
					ElementID(10,1,16));
	  ART_MSG(1, "Building a zone based on the lanes with %d points.",
		  p.size());
	  zone.perimeter_points.clear();
	  for(unsigned i = 0; i < p.size(); i++) {
	    WayPointNode node;
	    node.map = p.at(i);
	    zone.perimeter_points.push_back(node);
	  }
	}
      }
      
      if(zone.perimeter_points.size() < 3) {
	std::vector<MapXY> points_to_include;
	points_to_include.push_back(start);
	points_to_include.push_back(end);
	
	for(unsigned i = 0; i < obstacle->lasers->all_obstacle_list.size(); i++) {
	  if(Euclidean::DistanceTo(start,
				   obstacle->lasers->all_obstacle_list.at(i)) <
	     fake_zone_included_obstacle_radius) {
	    points_to_include.push_back(obstacle->lasers->all_obstacle_list.at(i));
	  }
	}
	zone = ZoneOps::build_fake_zone(points_to_include, fake_zone_border);
	MapXY lr, ul;
	float width = 0;
	float height = 0;
	
	if(zone.perimeter_points.size() > 3) {
	  lr = zone.perimeter_points[0].map;
	  ul = zone.perimeter_points[2].map;
	  width = fabs(lr.x-ul.x);
	  height = fabs(lr.y-ul.y);
	}
	
	if(!in_fake_zone) {
	  ART_MSG(1, "Making a bogus zone around us and the goal!");
	  in_fake_zone = true;
	  
	  ART_MSG(1, "Fake zone is %.1f by %.1f meters.", width, height);
	}
      } else {
	if(in_fake_zone) {
	  ART_MSG(1, "In a zone with at least three perimeter points.");
	  in_fake_zone = false;
	}
      }
      
      
      ART_MSG(8, "Getting a path (%.3f, %.3f) -> (%.3f, %.3f)",
	      start.x, start.y, end.x, end.y);
      
      ART_MSG(8, "ENTERING path_through_zone()");
      nodes =
	ZoneOps::path_through_zone(zone,
				   zone_perimeter_sample,
				   zone_safety_radius,
				   obs_points,
				   start,
				   end,
				   zone_write_graph_to_tmp,
				   zone_write_poly_to_tmp,
				   zone_write_obstacles_to_tmp,
				   zone_evg_thin_scale,
				   zone_grid_max_cells);
    }
    ART_MSG(8, "EXITING path_through_zone().");
    //    ART_MSG(1, "Current path size=%d", nodes.size());
    
    if(nodes.size() > 0) {
      WayPointNodeList::reverse_iterator i = nodes.rbegin();
      while(i < nodes.rend() &&
	    Euclidean::DistanceTo(start, i->map) > i->lane_width) {
	ART_MSG(8, "Can't yet go to point: (%.3f, %.3f) with radius %.3f."
		" Trying one closer...",
		i->map.x, i->map.y, i->lane_width);
	i++;
      }
      
      if(i < nodes.rend()) {
	aim = i->map;
	
	if(Euclidean::DistanceTo(end, i->map) < i->lane_width) {
	  ART_MSG(3, "Yippee! Goal is with radius, go for it!",
		  i->map.x, i->map.y, i->lane_width);
	  aim  = goal;
	} else {
	  ART_MSG(3, "Aiming for intermediate point (%.3f, %.3f)",
		  aim.x, aim.y);
	}
      } else {
	// XXX: TODO: MGR: Not sure what reaching this point means....
	ART_MSG(3, "We are within no point's safety radius! "
		"Shoot for goal...");
      }
    } else {
      ART_MSG(3, "No path through Voronoi graph! What to do? "
	      "Shoot for goal...");
    }    
  }
  
  // egocentric polar aim point
  Polar aim_polar =
    Coordinates::MapXY_to_Polar(aim, estimate->pos);
  
  if (verbose >= 3)
    {
      ART_MSG(8, "target, current positions: (%.3f, %.3f), (%.3f, %.3f)",
	      aim.x, aim.y,
	      estimate->pos.px, estimate->pos.py);
      ART_MSG(8, "relative heading: %.3f radians, distance: %.3f meters",
	      aim_polar.heading, aim_polar.range);
    }
  
  pcmd.yawRate = aim_polar.heading/fmaxf(1.0,estimate->vel.px/2);
  
  trace("voronoi_zone set_heading", pcmd);
  
  if (estimate->vel.px > fmaxf(3*zone_speed_limit,3)) {
    pcmd.velocity=0.0;
    return OK;
  }
  
  if (estimate->vel.px > 1.5*zone_speed_limit)
    pcmd.velocity=zone_speed_limit/2;
  
  result_t safe=safety->control(pcmd);
  if (safe>=Unsafe) {
    pcmd.velocity=zone_speed_limit;
    pcmd.yawRate/=2;
    while (fabsf(pcmd.yawRate) >= 0.1 && safety->control(pcmd) >=Unsafe)
      {
	pcmd.velocity=zone_speed_limit;
	pcmd.yawRate/=2;
      }
    ART_MSG(8, "Stuck.  Calling Parking");
    return park->control(pcmd,obstacle->lasers->all_obstacle_list);
  }
  
  ART_MSG(8, "EXITING zone::set_heading()");
 
  return safe;
}
