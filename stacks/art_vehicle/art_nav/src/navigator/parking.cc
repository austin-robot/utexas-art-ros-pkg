/*
 *  Navigator parking controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  Author: Patrick Beeson
 *
 *  $Id$
 */

#include "navigator_internal.h"
#include "obstacle.h"
#include "Controller.h"
#include "course.h"
#include "parking.h"
#include <art_map/rotate_translate_transform.h>
#include <art/steering.h>

#include "safety.h"
#include "stop.h"
#include "halt.h"

int find_spot(const std::vector<WayPointNode> &new_waypts)
{
  for (uint i=0; i < new_waypts.size()-1;i++)
    if (new_waypts[i].is_spot &&
	new_waypts[i+1].is_spot &&
	new_waypts[i].id.lane==
	new_waypts[i+1].id.lane)
      {
	return (int)i;
      }
  return -1;
}

poly adjust_spot_and_return_poly(WayPointNode &p1,
				 WayPointNode &p2,
				 float dx,
				 float dy) {
  poly spot;

  {
    posetype way_pose(p1.map.x, p1.map.y,
		      atan2f(p2.map.y-p1.map.y,
			     p2.map.x-p1.map.x));
    
    rotate_translate_transform trans;
    trans.find_transform(posetype(),way_pose);

    float dist=Euclidean::DistanceTo(p1.map,
				     p2.map);  
    
    p1.map = MapXY(trans.apply_transform(posetype(dx, dy, 0)));
    p2.map = MapXY(trans.apply_transform(posetype(dx + dist, dy, 0)));
  }

  posetype way_pose(p1.map.x, p1.map.y,
		    atan2f(p2.map.y-p1.map.y,
			   p2.map.x-p1.map.x));
  
  rotate_translate_transform trans;
  trans.find_transform(posetype(),way_pose);
  
  float dist=Euclidean::DistanceTo(p1.map,
				   p2.map);  
  
  // TODO: Make the + 1 a cfg option
  spot.p1 = MapXY(trans.apply_transform(posetype(0, p1.lane_width/2, 0)));
  spot.p2 = MapXY(trans.apply_transform(posetype(dist + 1, p2.lane_width/2, 0)));
  spot.p3 = MapXY(trans.apply_transform(posetype(dist + 1, -p2.lane_width/2, 0)));
  spot.p4 = MapXY(trans.apply_transform(posetype(0, -p1.lane_width/2, 0)));

  /*  
  fprintf(stderr, "p1=(%.3f, %.3f)\n", spot.p1.x, spot.p1.y);
  fprintf(stderr, "p2=(%.3f, %.3f)\n", spot.p2.x, spot.p2.y);
  fprintf(stderr, "p3=(%.3f, %.3f)\n", spot.p3.x, spot.p3.y);
  fprintf(stderr, "p4=(%.3f, %.3f)\n", spot.p4.x, spot.p4.y);
  */
  return spot;
}

int PARK_control::adjust_spot(std::vector<WayPointNode> &new_waypts,
			    ObstacleList obstacles,
			    float max_x_offset,
			    float max_y_offset,
			    float step_size) {
  PolyOps pops;
  int i = find_spot(new_waypts);
  if(i >= 0 && i < (int)new_waypts.size() - 1) {
    for(float dx = 0; dx <= max_x_offset; dx += step_size) {
      for(int x_sign = -1; x_sign <= 1; x_sign += 2) {
	for(float dy = 0; dy <= max_y_offset; dy += step_size) {
	  for(int y_sign = -1; y_sign <= 1; y_sign += 2) {
	    WayPointNode p1_new = new_waypts.at(i);
	    WayPointNode p2_new = new_waypts.at(i+1);
	    
	    poly spot = adjust_spot_and_return_poly(p1_new,
						    p2_new,
						    x_sign * dx,
						    y_sign * dy);
	    
	    int obstacles_in_spot = 0;
	    
	    for(unsigned j = 0; j < obstacles.size(); j++) {
	      if(pops.pointInPoly_ratio(obstacles[j], spot,1.1))
		obstacles_in_spot++;
	    }
	    
	    if(obstacles_in_spot <= min_obst) {
	      float distt=sqrtf(dx*dx+dy*dy);
	      if (distt <= min_adj_dist)
		{
		  min_adj_dist=distt;
		  min_obst=obstacles_in_spot;
		  new_waypts.at(i) = p1_new;
		  new_waypts.at(i+1) = p2_new;
		  //	      std::cerr<<"ADJUSTING SPOT BY X: "<<x_sign * dx
		  //		       <<" AND Y: "<<y_sign * dy<<"\n";
		  return 0;
		}
	    }
	  }
	}
      }
    }
  }
  return -1;
}

PARK_control::PARK_control(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  safety = new Safety(navptr, _verbose, 2);
  stop = new Stop(navptr, _verbose);
  halt = new Halt(navptr, _verbose);
  reset_me();

  state=hit_waypoint;
}

PARK_control::~PARK_control()
{
  delete safety;
  delete stop;
  delete halt;
}

// configuration method
void PARK_control::configure(ConfigFile* cf, int section)
{
  park_max_speed = cf->ReadFloat(section, "park_max_speed", 0.5);
  
  // maximum speed when inside a park_control
  speed_limit = cf->ReadFloat(section, "park_control_speed_limit", 1.0);
  ART_MSG(2, "\tspeed limit for park_control is %.1f m/s", speed_limit);
  
  min_distance = cf->ReadFloat(section, "park_min_distance", 0.5);
  ART_MSG(2, "\tmin_distance for park_control is %.1f m/s", min_distance);
  
  min_theta = cf->ReadFloat(section, "park_min_theta", 0.25);
  ART_MSG(2, "\tmin_theta for park_control is %.1f m/s", min_theta);
  
  park_turn_ratio = cf->ReadFloat(section, "park_turn_ratio", 1.0);
  ART_MSG(2, "\tturn_ratio for park_control is %.3f", park_turn_ratio);
  
  find_a_better_spot = cf->ReadBool(section, "find_a_better_spot", false);
  ART_MSG(2, "\tfind_a_better_spot is %d", find_a_better_spot);

  find_spot_max_x_offset = cf->ReadFloat(section,
					 "find_spot_max_x_offset", 1.0);
  ART_MSG(2, "\tfind_spot_max_x_offset is %.3f m", find_spot_max_x_offset);

  find_spot_max_y_offset = cf->ReadFloat(section,
					 "find_spot_max_y_offset", 1.0);
  ART_MSG(2, "\tfind_spot_max_y_offset is %.3f m", find_spot_max_y_offset);

  find_spot_step_size = cf->ReadFloat(section, "find_spot_step_size", 0.1);
  ART_MSG(2, "\tfind_spot_step_size is %.3f m", find_spot_step_size);

  safety->configure(cf, section);
}

inline float real_random(float multi=1.0){
  return float(random())/RAND_MAX*multi;
}


// go to next park_control way-point
//
// returns: OK.
//
Controller::result_t PARK_control::control(pilot_command_t &pcmd,
					 mapxy_list_t obs_list,
					 bool voronoi_stuck) {

  result_t result = OK;  
  if (voronoi_stuck)
    {
      pcmd.velocity=speed_limit;
      navdata->reverse=true;
      pcmd.yawRate=0.0;
      result_t safe=safety->control(pcmd);
      int count=1;
      bool neg=false;
      float rand=real_random(Steering::maximum_yaw/2);
      while (safe>=Unsafe && fabsf(pcmd.yawRate) <= Steering::maximum_yaw)
	{
	  if (!neg)
	    pcmd.yawRate=rand*count;
	  else
	    {
	      pcmd.yawRate=-rand*count;
	      count++;
	    }
	  neg=!neg;
	  pcmd.velocity=speed_limit;
	  safe=safety->control(pcmd);
	}
      if (safe >= Unsafe)
	{
	  pcmd.velocity=speed_limit;
	  navdata->reverse=false;
	  pcmd.yawRate=0;
	  result_t safe=safety->control(pcmd);
	  int count =1;
	  bool neg=false;
	  while (safe>=Unsafe && fabsf(pcmd.yawRate) <= Steering::maximum_yaw)
	    {
	      if (!neg)
		pcmd.yawRate=rand*count;
	      else
		{
		  pcmd.yawRate=-rand*count;
		  count++;
		}
	      neg=!neg;
	      pcmd.velocity=speed_limit;
	      safe=safety->control(pcmd);
	    }     
	}
      // must always run this on the final command
      result = safety->control(pcmd);      
      


      last_park_dist=pcmd.velocity;
      if (navdata->reverse)
	last_park_dist*=-1;
      last_park_turn=pcmd.yawRate;
      
      if (result >=Unsafe)
	{
	  pcmd.velocity=speed_limit;
	  navdata->reverse=true;
	}
      
      trace("park_control controller", pcmd, result);
      return result;
    }

  std::vector<WayPointNode> new_waypts;
  for(int i = 0; i < N_ORDER_WAYPTS; i++)
    new_waypts.push_back(order->waypt[i]);

  if(find_a_better_spot) {
    adjust_spot(new_waypts,
		obstacle->lasers->all_obstacle_list,
		find_spot_max_x_offset,
		find_spot_max_y_offset,
		find_spot_step_size);
  }

  if (!new_waypts[1].is_spot)
    state=hit_waypoint;
  else
    if ((new_waypts[1].is_spot && new_waypts[2].is_spot &&
	 new_waypts[1].id.pt==1 && new_waypts[2].id.pt==2))
      state=approach_spot;
    else
      if (new_waypts[1].is_spot && new_waypts[2].is_spot &&
	  new_waypts[1].id.pt==2 && new_waypts[2].id.pt==1)
	state=pull_in;
      else state=pull_out;

  if (course->spot_ahead())
    spot_points=course->calculate_spot_points(new_waypts);
  else if (!course->curr_spot())
    spot_points.clear();

  for (uint i=0; i<spot_points.size(); i++)
    obs_list.push_back(spot_points[i]);



	// Try going striaght back, then various ways of backwards and
	// turning.  Then forwards.
	
  
  switch (state)
    {
    case hit_waypoint:
    case approach_spot:
      {
	posetype way_pose(new_waypts[1].map.x,new_waypts[1].map.y,
			  atan2f(new_waypts[2].map.y-new_waypts[1].map.y,
				 new_waypts[2].map.x-new_waypts[1].map.x));
	
	rotate_translate_transform trans;
	
	trans.find_transform(posetype(ArtVehicle::front_bumper_px,0,0),
			     way_pose);
	posetype npose=trans.apply_transform(posetype(0,0,0));
	
	new_end_pose.px=npose.x;
	new_end_pose.py=npose.y;
	new_end_pose.pa=npose.theta;
      }
      break;
    case pull_in:
      {
	posetype way_pose(new_waypts[1].map.x,new_waypts[1].map.y,
			  atan2f(new_waypts[1].map.y-new_waypts[2].map.y,
				 new_waypts[1].map.x-new_waypts[2].map.x));
	
	rotate_translate_transform trans;
	
	trans.find_transform(posetype(ArtVehicle::front_bumper_px,0,0),
			     way_pose);
	posetype npose=trans.apply_transform(posetype());
	
	new_end_pose.px=npose.x;
	new_end_pose.py=npose.y;
	new_end_pose.pa=npose.theta;
      }
      break;
    case pull_out:
      {
	posetype way_pose(new_waypts[1].map.x,new_waypts[1].map.y,
			  atan2f(new_waypts[0].map.y-new_waypts[1].map.y,
				 new_waypts[0].map.x-new_waypts[1].map.x));
	rotate_translate_transform trans;
	
	trans.find_transform(posetype(ArtVehicle::front_bumper_px*2.0,0,0),
			     way_pose);
	posetype npose=trans.apply_transform(posetype());
	
	new_end_pose.px=npose.x;
	new_end_pose.py=npose.y;
	new_end_pose.pa=npose.theta;
      }
      break;
    }
  

  
  pcmd.velocity=speed_limit;
  navdata->reverse=false;
  
  if (Epsilon::equal(estimate->vel.px,0.0))
    halting=false;
  else if (Epsilon::gte(fabsf(estimate->vel.px),park_max_speed))
    halting=true;
  //halting=false;
  
  //Find Parking controller and run first step.
  result = initialize(pcmd, obs_list);
  
  if (result==Finished)
    {
      ART_MSG(3,"PARK Finished");
      halting=true;
      halt->control(pcmd);
      return result;
    }
  else 
    if (halting)
      {
	//	ART_MSG(1,"PARK Halting");
	pcmd.velocity=0.0;
	pcmd.yawRate=lastYaw;
	return OK;
      }
    else
      {
	if (result==OK)
	  {
	    
	    if (verbose >= 2)
	      {
		ART_MSG(5, "Go to pose %.3f,%.3f,%.3f",
			new_end_pose.px, new_end_pose.py, new_end_pose.pa);
	      } 
	  }
	else
	  {
	    // Try going striaght back, then various ways of backwards and
	    // turning.  Then forwards.
	    
	    ART_MSG(3,"Not App.");
	    pcmd.velocity=speed_limit;
	    navdata->reverse=true;
	    pcmd.yawRate=0;
	    result_t safe=safety->control(pcmd);
	    int count=1;
	    bool neg=false;
	    float rand=real_random(Steering::maximum_yaw/2);
	    while (safe>=Unsafe && fabsf(pcmd.yawRate) <= Steering::maximum_yaw)
	      {
		if (!neg)
		  pcmd.yawRate=rand*count;
		else
		  {
		    pcmd.yawRate=-rand*count;
		    count++;
		  }
		neg=!neg;
		pcmd.velocity=speed_limit;
		safe=safety->control(pcmd);
	      }
	    if (safe >= Unsafe)
	      {
		pcmd.velocity=speed_limit;
		navdata->reverse=false;
		pcmd.yawRate=0;
		result_t safe=safety->control(pcmd);
		int count =1;
		bool neg=false;
		while (safe>=Unsafe && fabsf(pcmd.yawRate) <= Steering::maximum_yaw)
		  {
		    if (!neg)
		      pcmd.yawRate=rand*count;
		    else
		      {
			pcmd.yawRate=-rand*count;
			count++;
		      }
		    neg=!neg;
		    pcmd.velocity=speed_limit;
		    safe=safety->control(pcmd);
		  }     
	      }
	  }
	
      }
  
  
  // must always run this on the final command
  result = safety->control(pcmd);      

  if (result >=Unsafe)
    {
      pcmd.velocity=speed_limit;
      navdata->reverse=true;
    }
  
  pcmd.velocity=fminf(speed_limit,pcmd.velocity);
  
  last_park_dist=pcmd.velocity;
  if (navdata->reverse)
    last_park_dist*=-1;
  last_park_turn=pcmd.yawRate;
  
  lastYaw=pcmd.yawRate;
  trace("park_control controller", pcmd, result);
  return result;
}

// reset all subordinate controllers
void PARK_control::reset(void)
{
  trace_reset("PARK_control");
  reset_me();
  halt->reset();
  safety->reset();
  stop->reset();
}


// reset controller
void PARK_control::reset_me(void)
{
  navdata->reverse = false;
  halting=true;
  park_distance=0.0;
  park_turn=0.0;
  last_park_turn=0.0;
  last_park_dist=0.0;
  min_adj_dist=Infinite::distance;
  min_obst=10000;
}


// initialize U-turn operation
Controller::result_t PARK_control::initialize(pilot_command_t& pcmd,
					    const mapxy_list_t& obs_list)
{
  int pcase=1;

  // Bearing to goal
  park_turn =Coordinates::bearing(estimate->pos,MapXY(new_end_pose));
  // Distance to goal
  park_distance = Euclidean::DistanceTo(new_end_pose,estimate->pos);
  
  // heading offset
  float dheading=Coordinates::normalize(new_end_pose.pa-estimate->pos.pa);
  


  if (hit_way_point(park_distance, park_turn, dheading) &&
      Epsilon::equal(estimate->vel.px,0.0))
    return Finished;  
  
  if (halting)
    return OK;

  float b=fabsf(park_turn);
  float h=fabsf(dheading);



  if (b < DTOR(45))
    if (h < DTOR(45))
      pcase=1;
    else if (h <DTOR(135) && dheading > 0)
      pcase=5;
    else if (h <DTOR(135) && dheading < 0)
      pcase=7;
    else pcase=6;
  else if (b < DTOR(135) && park_turn >0)
    if (h < DTOR(45))
      pcase=8;
    else if (h <DTOR(135) && dheading > 0)
      pcase=2;
    else if (h <DTOR(135) && dheading < 0)
      pcase=10;
    else pcase=9;
  else if (b < DTOR(135) && park_turn < 0)
    if (h < DTOR(45))
      pcase=14;
    else if (h <DTOR(135) && dheading > 0)
      pcase=15;
    else if (h <DTOR(135) && dheading < 0)
      pcase=4;
    else pcase=16;
  else
    if (h < DTOR(45))
      pcase=11;
    else if (h <DTOR(135) && dheading > 0)
      pcase=12;
    else if (h <DTOR(135) && dheading < 0)
      pcase=13;
    else pcase=3;

  std::cerr << "Bearing: "<<RTOD(park_turn)<<"\t dhead: "<<RTOD(dheading)<<"\t distance "<<park_distance<<std::endl;
  std::cerr << "Case: "<<pcase<<"\n";

  switch(pcase)
    {
    case 1:
      if (fabsf(sinf(park_turn)*park_distance) <
	  fabsf(sinf(dheading)*(park_distance+ 3.5)))
	park_turn=dheading;
      break;
    case 2:
    case 4:
      if (fabsf(dheading) > fabsf(park_turn))
	park_turn = dheading;
      break;
    case 3:
      {
	float new_turn=Coordinates::normalize(park_turn+M_PI);
	float new_head=Coordinates::normalize(dheading+M_PI);
	if (fabsf(sinf(new_turn)*park_distance) <
	    fabsf(sinf(new_head)*(park_distance+ 3.5)))
	  park_turn=dheading;
	park_distance*=-1;
	navdata->reverse=true;
      }
      break;
    case 5:
    case 6:
    case 7:
    case 9:
    case 16:
      break;
    case 8:
    case 10:
    case 14:
    case 15:
      park_turn*=1;
      park_distance*=-1;
      navdata->reverse=true;
      break;
    case 11:
      park_distance*=-1;
      navdata->reverse=true;
      park_turn=0;
      break;
    case 12:
    case 13:
      park_turn=dheading;
      park_distance*=-1;
      navdata->reverse=true;
      break;
    }

  if (fabsf(park_distance) < fmaxf(Steering::steer_speed_min,pcmd.velocity) && fabsf(park_turn) < Steering::maximum_yaw)
    pcmd.yawRate=park_turn;
  else
    pcmd.yawRate= fmaxf(Steering::steer_speed_min, pcmd.velocity)/
      fabs(park_distance)* park_turn * park_turn_ratio;
  pcmd.yawRate=fmaxf(-Steering::maximum_yaw/2,
		     fminf(Steering::maximum_yaw/2,pcmd.yawRate));
  
  if (safety->control(pcmd) < Unsafe)
    {
      return OK;
    }
  pcmd.velocity=speed_limit;
  navdata->reverse=true;
  pcmd.yawRate=dheading;

  if (safety->control(pcmd) < Unsafe)
    {
      return OK;
    }

  navdata->reverse=!navdata->reverse;
  pcmd.yawRate*=1;
  pcmd.velocity=speed_limit;
  
  if (safety->control(pcmd) < Unsafe)
    {
      return OK;
    }  


  return NotApplicable;

}

bool PARK_control::hit_way_point(float goal_distance, float goal_turn,
			       float dheading) {
  bool done=false;
  
  float car_bearing=Coordinates::bearing(new_end_pose,MapXY(estimate->pos));
  
  switch(state)
    {
    case hit_waypoint:
    case approach_spot:
      {
	if (fabsf(dheading) < DTOR(30))
	  if (goal_distance < 1.0 ||
	      (fabsf(car_bearing) < HALFPI))
	    done=true;
      }
      break;
    case pull_in:
      {
	//	if (fabsf(dheading) < DTOR(20))
	if (goal_distance < 0.75 ||
	    fabsf(car_bearing) < HALFPI)
	  done=true;
      }
      break;
    case pull_out:
      {
	if (fabsf(dheading) > DTOR(45) ||
	    fabsf(car_bearing) > HALFPI ||
	    goal_distance < 1.0)
	  //	    fabsf(goal_turn) < HALFPI)
	  done=true;
      }
      break;
    }      

  course->no_waypoint_reached();

  if (done)
    {
      navdata->last_waypt=order->waypt[1].id;
    }

  return done;
  
}

bool PARK_control::small_segment(float distance, float turn) {
  return (fabsf(distance) < min_distance &&
	  fabsf(turn) < min_theta);
}

