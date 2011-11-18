/*
 *  Navigator course planning class
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <angles/angles.h>

#include <art/DARPA_rules.h>

#include "navigator_internal.h"
#include "course.h"
#include <art/steering.h>
#include <art_map/coordinates.h>
#include <art_nav/estimate.h>
using art_msgs::ArtVehicle;
using Coordinates::bearing;
using Coordinates::normalize;

/** @todo Use ROS tf instead of art_map/rotate_translate_transform */
#include <art_map/rotate_translate_transform.h>

// Constructor
Course::Course(Navigator *_nav, int _verbose)
{
  verbose = _verbose;
  nav = _nav;

  // copy convenience pointers to Navigator class data
  estimate = &nav->estimate;
  navdata = &nav->navdata;
  odom = nav->odometry;
  order = &nav->order;
  pops = nav->pops;
  config_ = &nav->config_;

  // initialize polygon vectors
  plan.clear();
  polygons.clear();

  for (unsigned i = 0; i < 2; ++i)
    adj_polys[i].clear();
  passing_lane = -1;
  passed_lane.clear();

  last_error=0;

  reset();
}

// Course class initialization for run state cycle.
//
// exit:
//	navdata->cur_poly updated
//	order->waypt array reflects last_waypt
//
void Course::begin_run_cycle(void)
{
  waypoint_checked = false;

  // Finding the current polygon is easy in a travel lane, but
  // more difficult in intersections.  They have many overlapping
  // transition lanes, and getContainingPoly() picks the first one
  // in the polygons vector, not necessarily the correct one.

  // So, first check whether vehicle is in the planned travel lane
  if (0 <= (poly_index = pops->getContainingPoly(plan,
                                                 MapPose(estimate->pose.pose))))
    {
      // This is the normal case.  Re-resolve poly_index relative to
      // polygons vector.
      poly_index = pops->getPolyIndex(polygons, plan.at(poly_index));
    }
  else
    {
      // Not in the planned travel lane, check the whole road network.
      poly_index = pops->getContainingPoly(polygons,
                                           MapPose(estimate->pose.pose));
    }

  // set cur_poly ID in navdata for Commander (no longer used)
  if (poly_index < 0)			// no polygon found?
    navdata->cur_poly = -1;		// outside the road network
  else
    navdata->cur_poly = polygons.at(poly_index).poly_id;

  // This order may have been issued before Commander saw the
  // last_waypt Navigator returned in a previous cycle.  Make sure the
  // order reflects the current situation.
  int limit = art_msgs::Order::N_WAYPTS; // search limit
  while (ElementID(order->waypt[0].id) != ElementID(navdata->last_waypt)
         && --limit > 0)
    {
      ROS_DEBUG_STREAM("waypoint " << ElementID(order->waypt[1].id).name().str
                       << " already reached, advance order->waypt[] array");
      // advance order->waypt array by one
      for (unsigned i = 1; i < art_msgs::Order::N_WAYPTS; ++i)
        order->waypt[i-1] = order->waypt[i];
    }

  // log current order attributes
  for (unsigned i = 0; i < art_msgs::Order::N_WAYPTS; ++i)
    ROS_DEBUG("waypt[%u] %s (%.3f,%.3f), E%d G%d L%d P%d S%d X%d Z%d",
              i, ElementID(order->waypt[i].id).name().str,
              order->waypt[i].mapxy.x,
              order->waypt[i].mapxy.y,
              (bool) order->waypt[i].is_entry,
              (bool) order->waypt[i].is_goal,
              (bool) order->waypt[i].is_lane_change,
              (bool) order->waypt[i].is_spot,
              (bool) order->waypt[i].is_stop,
              (bool) order->waypt[i].is_exit,
              (bool) order->waypt[i].is_perimeter);
}

void Course::configure()
{
  // dynamic reconfigure parameters
  lane_change_secs = config_->lane_change_secs;
  //lane_steer_time = config_->lane_steer_time;
  heading_change_ratio = config_->heading_change_ratio;
  turning_latency = config_->turning_latency;
  k_error = config_->turning_offset_tune;
  k_theta = config_->turning_heading_tune;
  //yaw_ratio = config_->yaw_ratio;
  k_int = config_->turning_int_tune;
  //min_lane_change_dist = config_->min_lane_change_dist;
  min_lane_steer_dist = config_->min_lane_steer_dist;
  max_speed_for_sharp = config_->max_speed_for_sharp;
  spring_lookahead = config_->spring_lookahead;
  max_yaw_rate = config_->real_max_yaw_rate;
  zone_waypoint_radius = config_->zone_waypoint_radius;
  //zone_perimeter_radius = config_->zone_perimeter_radius;
  spot_waypoint_radius = config_->spot_waypoint_radius;

#if 0
  ros::NodeHandle nh("~");

  // how far away (in seconds) we aim when changing lanes
  nh.param("lane_change_secs", lane_change_secs, 2.0);
  ROS_INFO("lane change target is %.3f seconds ahead",
          lane_change_secs);

  // Look-ahead time for steering towards a polygon.
  nh.param("lane_steer_time", lane_steer_time, 2.0);
  ROS_INFO("lane steering time is %.3f seconds", lane_steer_time);

  nh.param("heading_change_ratio", heading_change_ratio, 0.75);
  ROS_INFO("heading change ratio is %.3f", heading_change_ratio);

  nh.param("turning_latency", turning_latency, 1.0);
  ROS_INFO("turning latency time is %.3f seconds", 1.0);

  // Look-ahead time for steering towards a polygon.
  nh.param("turning_offset_tune", k_error, 0.5);
  ROS_INFO("yaw tuning parameter (offset) is %.3f", k_error);

  // Look-ahead time for steering towards a polygon.
  nh.param("turning_heading_tune", k_theta, sqrt(k_error/2));
  ROS_INFO("yaw tuning parameter (heading) is %.3f", k_theta);

  // Look-ahead time for steering towards a polygon.
  nh.param("yaw_ratio", yaw_ratio, 0.75);
  ROS_INFO("yaw ratio is %.3f", yaw_ratio);

  // Look-ahead time for steering towards a polygon.
  nh.param("turning_int_tune", k_int, 1.25);
  ROS_INFO("yaw tuning parameter (integral) is %.3f", k_int);

  // Minimum distance to aim for when changing lanes.
  // Should at least include front bumper offset and minimum separation.
  nh.param("min_lane_change_dist", min_lane_change_dist,
           (double) (DARPA_rules::min_forw_sep_travel
                     + ArtVehicle::front_bumper_px));
  ROS_INFO("minimum lane change distance is %.3f meters",
          min_lane_change_dist);

  // Minimum look-ahead distance for steering towards a polygon.
  // Should at least include front bumper offset.
  nh.param("min_lane_steer_dist", min_lane_steer_dist,
           (double) ArtVehicle::front_bumper_px);
  ROS_INFO("minimum lane steering distance is %.3f meters",
          min_lane_steer_dist);

  // how fast the maximum steer can be done
  nh.param("max_speed_for_sharp", max_speed_for_sharp ,3.0); 
  ROS_INFO("maximum speed to go full yaw is %.3f m/s", max_speed_for_sharp);

  // how far in future to estimate for reactive steering
  nh.param("spring_lookahead", spring_lookahead, 0.0);
  ROS_INFO("spring lookahead distance is %.3f m", spring_lookahead);

  nh.param("real_max_yaw_rate", max_yaw_rate, Steering::maximum_yaw);
  ROS_INFO("real maximum yaw rate is %.3f (radians/s)", max_yaw_rate);

  nh.param("zone_waypoint_radius", zone_waypoint_radius, 1.0);
  ROS_INFO("zone waypoint radius is %.3f m", zone_waypoint_radius);

  nh.param("zone_perimeter_radius", zone_perimeter_radius, 2.0);
  ROS_INFO("zone perimeter radius is %.3f m", zone_perimeter_radius);

  nh.param("spot_waypoint_radius", spot_waypoint_radius, 0.5);
  ROS_INFO("spot waypoint radius is %.3f m", spot_waypoint_radius);
#endif
}

/** Set heading for desired course.
 *
 * @pre
 *	plan contains desired polygon path to follow
 *
 * @param pcmd[in,out] pilot command to be published
 * @param offset_ratio 1.0 pushes the left side of the car to the left
 *		lane boundary, 0.0 to the center, -1.0 to the right.
 *		Larger offsets push the car outside the lane.
 */
void Course::desired_heading(pilot_command_t &pcmd, float offset_ratio)
{
  if (Epsilon::equal(pcmd.velocity, 0.0))
    return;

  Polar aim_polar;			// egocentric polar aim point
  //  float aim_abs_heading = 0;
  float aim_next_heading = 0;
  float aim_distance = 0;
  bool aim_in_plan = false;
  int aim_index = -1;
  float used_velocity = estimate->twist.twist.linear.x;
  float target_dist = min_lane_steer_dist;

  if (plan.empty())
    {
      // no plan available: a big problem, but must do something
      ROS_WARN_THROTTLE(40, "no lane data available, steer using waypoints.");
      aim_polar = head_for_waypt(target_dist);
      aim_distance = aim_polar.range;
      aim_next_heading =
        normalize(MapPose(estimate->pose.pose).yaw + aim_polar.heading);
    }
  else 
    {
      // Look in plan
      aim_index = pops->getPolyIndex(plan, aim_poly);

      poly_list_t edge;
      pops->add_polys_for_waypts(plan,edge,order->waypt[0].id,
                                 order->waypt[1].id);

      // get closest polygon to estimated position
      int nearby_poly =
        pops->getClosestPoly(edge, MapPose(estimate->pose.pose));
      if (nearby_poly >= 0)
	nearby_poly = pops->getPolyIndex(plan,edge.at(nearby_poly));
      else
        nearby_poly = pops->getClosestPoly(plan, MapPose(estimate->pose.pose));

      if (aim_poly.poly_id != -1
          && aim_index >= 0
          && aim_index < (int) plan.size() - 1)
	{
	  if (nearby_poly >= 0)
	    {
	      int aim_index2 = pops->index_of_downstream_poly
		(plan,nearby_poly,target_dist);	      
	  
	      if (aim_index2 > aim_index && aim_index2 < (int) plan.size()-1) 
		{
		  aim_index = aim_index2;
		  aim_poly.poly_id = -1; // no aim polygon defined
		}
	    }

	  aim_distance = Euclidean::DistanceTo(plan.at(aim_index+1).midpoint,
					       plan.at(aim_index).midpoint);
	  aim_next_heading = atan2f(plan.at(aim_index+1).midpoint.y-
				    plan.at(aim_index).midpoint.y,
				    plan.at(aim_index+1).midpoint.x-
				    plan.at(aim_index).midpoint.x);

	  aim_in_plan = true;
	  
          ROS_DEBUG("steering down the lane toward polygon %d",
                    plan.at(aim_index).poly_id);
	}
      else
	{
	  if (nearby_poly >= 0)
	    {
              ROS_DEBUG("nearby_poly in desired_heading() is %d",
                        plan.at(nearby_poly).poly_id);
	      
	      // set aim_polar to the closest polygon at least target_dist
	      // meters away from the estimated position.
	      aim_index =
                pops->index_of_downstream_poly(plan, nearby_poly,
                                               target_dist);
	      if (aim_index >= 0 && aim_index < (int)plan.size()-1)
		{
		  // Polygon at target distance
		  aim_distance =
                    Euclidean::DistanceTo(plan.at(aim_index+1).midpoint,
                                          plan.at(aim_index).midpoint);

		  aim_next_heading = atan2f(plan.at(aim_index+1).midpoint.y-
					    plan.at(aim_index).midpoint.y,
					    plan.at(aim_index+1).midpoint.x-
					    plan.at(aim_index).midpoint.x);

		  aim_in_plan = true;
		  
                  ROS_DEBUG("steering at least %.3fm "
                            "down the lane toward polygon %d",
                            target_dist, plan.at(aim_index).poly_id);
		}
	      else
		{
                  // No polygon in target distance.  Head to next way-point
		  ROS_WARN_THROTTLE(40, "no polygon at least %.3fm away, "
                                     "steer using waypoints", target_dist);
		  aim_polar = head_for_waypt(target_dist);
		  
		  aim_distance = aim_polar.range;
		  aim_next_heading =
                    normalize(MapPose(estimate->pose.pose).yaw
                              + aim_polar.heading);
		}
	    }
	  else
	    {
	      // no plan available: a big problem, but must do
	      // something.  Go to next waypoint.
	      
              ROS_WARN_THROTTLE(40, "no lane data available, "
                                "steer using waypoints.");
	      aim_polar = head_for_waypt(target_dist);
	      aim_distance = aim_polar.range;
	      aim_next_heading =
                normalize (MapPose(estimate->pose.pose).yaw
                           + aim_polar.heading);
	    }
	}
    }
  
  
  ROS_DEBUG("desired, current positions: (%.3f, %.3f), (%.3f, %.3f, %.3f)",
            order->waypt[1].mapxy.x, order->waypt[1].mapxy.y,
            estimate->pose.pose.position.x,
            estimate->pose.pose.position.y,
            MapPose(estimate->pose.pose).yaw);
  
  float full_heading_change =
    fabs(normalize(aim_next_heading-MapPose(estimate->pose.pose).yaw));
  
  float max_speed_to_hit_aim = 
    max_speed_for_change_in_heading(full_heading_change,
				    aim_distance,
				    pcmd.velocity,
				    max_yaw_rate);

  pcmd.velocity = fminf(pcmd.velocity,
			max_speed_to_hit_aim);
  


#if 0
  if (pcmd.velocity>used_velocity)
    used_velocity += fminf(pcmd.velocity-used_velocity,1.0*turning_latency);
  else if (pcmd.velocity < used_velocity)
    used_velocity-=fminf(used_velocity-pcmd.velocity,1.0*turning_latency);
#endif

#if 1
  used_velocity = fmaxf(pcmd.velocity,used_velocity);
#endif
  
#if 0
  used_velocity = fminf(used_velocity,pcmd.velocity);
#endif

  ROS_DEBUG("Thresholding speed to %.3f m/s", used_velocity);

  float spring_yaw;
  if (aim_in_plan)
    spring_yaw = get_yaw_spring_system(aim_polar, aim_index, 
				     aim_next_heading,
				     max_yaw_rate, used_velocity,
				     offset_ratio);
  else
    spring_yaw = get_yaw_spring_system(aim_polar, -1, aim_next_heading,
				     max_yaw_rate, used_velocity);
  
  pcmd.yawRate = spring_yaw;

#if 0
  if (Epsilon::equal(pcmd.yawRate,max_yaw_rate))
    pcmd.velocity = fminf(pcmd.velocity,Steering::steer_speed_min);
#endif  

  nav->trace_controller("desired_heading", pcmd);
}

// return distance in the plan to a way-point
float Course::distance_in_plan(const MapPose &from,
			       const WayPointNode &wp) const
{
  if (plan.empty())
    return Euclidean::DistanceToWaypt(from, wp);
  else return pops->distanceAlongLane(plan, from.map, wp.map);
}

// return distance in plan to a pose
float Course::distance_in_plan(const MapPose &from,
			       const MapPose &to) const
{
  if (plan.empty())
    return Euclidean::DistanceTo(from, to);
  else return pops->distanceAlongLane(plan, from.map, to.map);
}

float Course::distance_in_plan(const MapPose &from,
			       const MapXY &to) const
{
  if (plan.empty())
    return Euclidean::DistanceTo(from.map, to);
  else return pops->distanceAlongLane(plan, from.map,to);
}


// Course class termination for run state cycle.
//
// entry:
//	waypoint_checked true if any controller has checked that a new
//	way-point has been reached.
//
void Course::end_run_cycle()
{
  if (!waypoint_checked)
    ROS_ERROR("failed to check for way-point reached!");
}

// find an aim polygon ahead of the car in lane
//
//  The selected polygon is at least min_lane_change_dist away.  The
//  exact choice depends on the distance of the car from the lane, and
//  its velocity.
//
// returns: index of aim polygon, -1 if none
//
int Course::find_aim_polygon(poly_list_t &lane)
{

  poly_list_t edge;
  pops->add_polys_for_waypts(lane,edge,order->waypt[0].id,
			     order->waypt[1].id);
  
  // get closest polygon to estimated position
  int nearby_poly = pops->getClosestPoly(edge, MapPose(estimate->pose.pose));
  if (nearby_poly < 0)
    nearby_poly = pops->getClosestPoly(lane, MapPose(estimate->pose.pose));
  else
    nearby_poly = pops->getPolyIndex(lane,edge.at(nearby_poly));

  if (nearby_poly < 0)
    return -1;
  
#if 1  
  float aim_distance = min_lane_steer_dist;
  
  ROS_DEBUG("aim point %.3fm ahead", aim_distance);
  
  return pops->index_of_downstream_poly(lane, nearby_poly, aim_distance);

#else

  // increase aim_distance if the lane is far away
  float lane_distance =
    pops->getShortestDistToPoly(MapPose(estimate->pose.pose), lane.at(nearby_poly));
  
  if (Epsilon::equal(lane_distance,0.0))
    return -1;
   
  float lane_change_distance_ratio = estimate->vel.x;
  float aim_distance;

  if (req_max_dist < Infinite::distance)
    aim_distance = req_max_dist;
  else 
    {  
      // Try based on speed -- may be too far or 0 (if not moving)
      aim_distance = fminf(lane_distance * lane_change_distance_ratio,
			   estimate->vel.x * lane_change_secs);
    }

  float max_pass_distance = ArtVehicle::length*4;

  // Threshold by maximum distance to get over.
  aim_distance = fminf(max_pass_distance,aim_distance);

  if (order->waypt[1].is_goal)
    {
      float way_dist = distance_in_plan(MapPose(estimate->pose.pose),
                                        order->waypt[1]);
      aim_distance = fminf(aim_distance,way_dist);
    }

  // At least look as far ahead as bumper
  aim_distance = fmaxf(aim_distance, ArtVehicle::front_bumper_px);

  ROS_DEBUG("lane is %.3fm away, aim point %.3fm ahead",
	    lane_distance, aim_distance);
  
  return pops->index_of_downstream_poly(lane, nearby_poly, aim_distance);
  
#endif
}

// Find an appropriate polygon path for passing an obstacle blocking
// the current travel lane.
//
// Note that an obstacle could be on a checkpoint.  DARPA said that
// checkpoints would be in driveable locations.  But, another car
// could still stop there.  Our implementation will pass it anyway and
// consider the checkpoint reached when the car passes it.  That does
// not meet the requirement that the front bumper pass over the
// checkpoint, but follows the DARPA Technical FAQ recommendation.
//
// exit:
//	sets adj_lane[passing_lane], adj_polys[passing_lane]
//	sets passing_left true iff passing to the left
//	leaves Course::plan alone
//	returns true, if alternate lane found
//
bool Course::find_passing_lane(void)
{
  ROS_DEBUG("find passing lane around waypoint %s",
            ElementID(order->waypt[1].id).name().str);

  // generate adjacent lane IDs
  adj_lane[0] = order->waypt[1].id;
  --adj_lane[0].lane;			// next lower lane number

  adj_lane[1] = order->waypt[1].id;
  ++adj_lane[1].lane;			// next higher lane number

#if 1 // more general implementation, experimental

  int cur_index = pops->getClosestPoly(plan, MapPose(estimate->pose.pose));
  if (cur_index == -1)
    {
      ROS_WARN("no polygon nearby in plan");
      return false;
    }
  poly cur_poly = plan.at(cur_index);
  
  // collect polygons for any adjacent lanes and determine their
  // relative position and direction.
  int left_lane = -1;			// no left lane
  int right_lane = -1;			// no right lane
  bool adj_forw[2];			// true if going forward

  for (unsigned i = 0; i < 2; ++i)
    {
      adj_lane[i].pt = 0;		// lane ID, not way-point
      adj_polys[i].clear();
      if (adj_lane[i].lane == 0)	// ID not valid?
	continue;

      // collect lane polygons
      pops->AddLanePolys(polygons, adj_polys[i], adj_lane[i]);
      int this_index =
        pops->getClosestPoly(adj_polys[i],
                             MapXY(order->waypt[1].mapxy));
      if (this_index < 0)		// no polygon found?
	continue;

      // see if it is right or left of current lane
      poly this_poly = adj_polys[i].at(this_index);
      if (pops->left_of_poly(this_poly, cur_poly))
	left_lane = i;
      else
	right_lane = i;

      // see if it goes forward or backward
      adj_forw[i] = pops->same_direction(cur_poly, this_poly, HALFPI);
      if (!adj_forw[i])
	{
	  // collect polygons in reverse direction, instead
	  adj_polys[i].clear();
	  pops->AddReverseLanePolys(polygons, adj_polys[i], adj_lane[i]);
	}

      if (verbose >= 4)
	log(adj_lane[i].lane_name().str, adj_polys[i]);
    }

  // pick the preferred lane and direction
  if (right_lane >= 0
      && adj_forw[right_lane])
    passing_lane = right_lane;		// right lane, forward
  else if (left_lane >= 0
           && adj_forw[left_lane])
    passing_lane = left_lane;		// left lane, forward
  else if (right_lane >= 0)
    passing_lane = right_lane;		// right lane, backward
  else if (left_lane >= 0)
    passing_lane = left_lane;		// left lane, backward
  else
    {
      passing_lane = -1;
      ROS_WARN("no passing lane available for waypoint %s",
               ElementID(order->waypt[1].id).name().str);
      return false;
    }

  // save direction for turn signals
  passing_left = (passing_lane == left_lane);

  ROS_INFO("passing lane %s selected, to %s going %s",
           adj_lane[passing_lane].lane_name().str,
           (passing_left? "left": "right"),
           (adj_forw[passing_lane]? "forward": "backward"));

#else // old version

  for (unsigned i = 0; i < 2; ++i)
    {
      // TODO: use pops->PolyHeading() to determine direction of
      // passing lane.  (This code reproduces site visit capability,
      // so it is always reversed.)
      adj_lane[i].pt = 0;		// lane ID, not way-point
      adj_polys[i].clear();
      if (adj_lane[i].lane > 0)	// ID in valid range?
	{
	  pops->AddReverseLanePolys(polygons, adj_polys[i], adj_lane[i]);
	}
      if (verbose >= 4)
	log(adj_lane[i].lane_name().str, adj_polys[i]);
    }

  // find a non-empty lane
  passing_lane = 1;
  while (passing_lane >= 0 && adj_polys[passing_lane].empty())
    {
      --passing_lane;
    }

  // if there are no polygons at all, or no adjacent lane in this
  // segment, return failure
  if (passing_lane < 0)
    {
      ROS_WARN("no passing lane available for waypoint %s",
               order->waypt[1].id.name().str);
      return false;
    }

  passing_left = true;			// always left for now

  ROS_DEBUG("passing lane %s selected",
	    adj_lane[passing_lane].lane_name().str);

#endif

  return true;
}

// Find a path in the travel lane to the next few way-points.
//
// We want the sequence of polygons that will take us from our current
// position to the order->waypt[].id polygons, avoiding wrong paths
// through any intersection.
//
// entry: rejoin is true when the car is currently outside the lane
//
void Course::find_travel_lane(bool rejoin)
{
  if (plan_valid())
    {
      ROS_DEBUG("find_travel_lane() plan still valid");
    }
  else
    {
      // make a new plan
      plan.clear();
      aim_poly.poly_id = -1;		// no aim polygon defined
      set_plan_waypts();
    
      if (polygons.size() == 0)		// no lane data available?
	{
          ROS_WARN("find_travel_lane() has no polygons");
	  return;
	}

      // push waypt[0] polygon onto the plan
      pops->add_polys_for_waypts(polygons, plan,
				 order->waypt[0].id, order->waypt[0].id);
      if (verbose >= 6)
        log("debug plan", plan);

      // add polygons leading to the target waypt entries
      for (int i = 1; i < Order::N_WAYPTS; ++i)
	{
	  // Do not repeat polygons for repeated way-points in the order.
	  if (ElementID(order->waypt[i-1].id)
              != ElementID(order->waypt[i].id))
	    // Collect all polygons from previous waypt to this one and
	    // also the polygon containing this one.
	    pops->add_polys_for_waypts(polygons, plan,
				       order->waypt[i-1].id,
				       order->waypt[i].id);
	  // don't plan past a zone entry
	  if (order->waypt[i].is_perimeter)
	    break;
	}

      if (plan.size() > 1)
	{
	  ROS_DEBUG("plan[0] start, end waypoints are %s, %s, poly_id = %d",
                    plan.at(0).start_way.name().str,
                    plan.at(0).end_way.name().str,
                    plan.at(0).poly_id);
	  ROS_DEBUG("plan[1] start, end waypoints are %s, %s, poly_id = %d",
                    plan.at(1).start_way.name().str,
                    plan.at(1).end_way.name().str,
                    plan.at(1).poly_id);
	}
      log("find_travel_lane() plan", plan);
    }
  
  new_plan_lanes = false;		// plan reflects current lanes
  aim_poly.poly_id = -1;		// no aim polygon defined

  if (rejoin)
    {
      // If the car is outside its lane, select appropriate polygon to
      // rejoin it.  Otherwise, the car may overshoot and circle back,
      // which would be very bad.  This also prevents the follow
      // safely controller from getting confused after passing an
      // obstacle in the target lane.

      // Beware: when approaching the site visit intersection the loop
      // in segment one causes find_aim_polygon(lane_polys) to pick
      // the wrong end of the lane, causing the car to turn the wrong
      // way.  So, first look it up in plan, then find the
      // corresponding index in lane_polys.

      // find a polygon slightly ahead of the car
      int aim_index = find_aim_polygon(plan);
      if (aim_index >= 0)
	{
	  // set aim polygon for obstacle avoidance
	  aim_poly = plan.at(aim_index);
          ROS_DEBUG("aim polygon is %d", aim_poly.poly_id);
	}
    }
}

// Head directly for next reachable way-point.
//
// This is trouble: the plan stops too soon for navigating
// by polygons.  Have to do something, so head directly for
// the next way-point, but make sure it's far enough away
// that the car does not double back to it.
Polar Course::head_for_waypt(float target_dist)
{
  using Coordinates::MapXY_to_Polar;
  Polar aim_polar = MapXY_to_Polar(MapXY(order->waypt[1].mapxy),
                                   *estimate);
  if (aim_polar.range < target_dist)
    {
      if (special_waypt(1))
	{
	  // If the next way-point is a stop or U-turn, go straight
	  // and try to reach it.
	  ART_MSG(8, "waypt[1] is a special way-point, keep current heading");
	  aim_polar.heading=0.0;
	}
      else if (order->waypt[1].is_perimeter)
	{
	  ART_MSG(8, "waypt[1] is a perimeter point");
	  aim_polar = MapXY_to_Polar(MapXY(order->waypt[1].mapxy),
                                     *estimate);
	  if (fabsf(bearing(MapPose(estimate->pose.pose),
                            MapXY(order->waypt[1].mapxy)))
              > HALFPI)
	    new_waypoint_reached(order->waypt[1].id);
	}
      else
	{
	  // waypt[1] is too close, steer for waypt[2] instead
	  aim_polar = MapXY_to_Polar(MapXY(order->waypt[2].mapxy),
                                     *estimate);
	  ART_MSG(8, "waypt[1] less than %.3fm away, using waypt[2] instead",
		  target_dist);
	  // claim we got there (we're at least close)
	  new_waypoint_reached(order->waypt[1].id);
	}
    }
  return aim_polar;
}

// return lane change direction
Course::direction_t Course::lane_change_direction(void)
{
  int w0_index = pops->get_waypoint_index(polygons, order->waypt[0].id);
  int w1_index = pops->get_waypoint_index(polygons, order->waypt[1].id);

  // give up unless both polygons are available
  if (w0_index < 0 || w1_index < 0)
    return Straight;

  if (pops->left_of_poly(polygons.at(w1_index), polygons.at(w0_index)))
    return Left;
  else
    return Right;
}

// check if lane way-point reached
//
// Considers a way-point reached when the car is in front of the pose
// formed by the way-point and the heading of its containing polygon.
//
// exit: navdata->last_waypt updated
// returns: true if order->waypt[1] reached (unless a special way-point)
//
// bugs: Does not work for zone perimeter way-points because they do
//	 not have a containing polygon.  Those are detected by the
//	 stop_line controller, instead.
//
bool Course::lane_waypoint_reached(void)
{
  // Mark the way-point checked, even if it is a special one.
  waypoint_checked = true;

  if (order->waypt[1].is_perimeter)
    return zone_perimeter_reached();
  
  // Special way-points (stop, U-turn) are handled explicitly
  // elsewhere by their state-specific controllers.  They cause state
  // transitions, so they must be ignored here and only considered
  // "reached" when the requirements of those specific controllers are
  // fully met.
  if (special_waypt(1))
    return false;

#ifdef USE_PATS
  ElementID last_way = pops->updateLaneLocation(polygons,
						odom->pose,
						order->waypt[0],
						order->waypt[1]);
  if (last_way == order->waypt[1].id)
    {
      navdata->last_way = last_way;
      return true;
    }
  return false;
#endif

  bool found = false;

  // Instead of checking a circle about the way-point, see if the
  // car has reached a line through the way-point perpendicular to
  // the direction of its lane.

  // get polygon index of waypt[1] (TODO: save somewhere)
  int w1_index = -1;

  w1_index = pops->get_waypoint_index(polygons, order->waypt[1].id);
  
  if (w1_index >= 0)
    {
      // form way-point pose using polygon heading
      // TODO: save somewhere
      MapPose w1_pose(MapXY(order->waypt[1].mapxy),
		      pops->PolyHeading(polygons.at(w1_index)));
      
      // Is the bearing of the car from that pose within 90
      // degrees of the polygon heading?
      float bearing_from_w1 = bearing(w1_pose,
                                      MapXY(odom->pose.pose.position));
      if (fabsf(bearing_from_w1) < angles::from_degrees(90))
	{
	  // The car is "in front" of this way-point's pose.
          ROS_INFO("reached waypoint %s, bearing %.3f radians",
                   ElementID(order->waypt[1].id).name().str,
                   bearing_from_w1);
	  navdata->last_waypt = order->waypt[1].id;
	  found = true;
	}
    }

  
  if (!found)
    ROS_DEBUG("cur_poly = %d, last_waypt = %s",
              navdata->cur_poly,
              ElementID(navdata->last_waypt).name().str);
  return found;
}

/** Handle lanes message.
 *
 *  Called from the topic subscription callback when new lanes data
 *  arrive.
 */
void Course::lanes_message(const art_msgs::ArtLanes &lanes)
{
  // copy polygons from message
  polygons.resize(lanes.polygons.size());
  for (unsigned num = 0; num < lanes.polygons.size(); num++)
    polygons.at(num) = lanes.polygons[num];

  if (polygons.empty())
    ROS_WARN("empty lanes polygon list received!");

  // force plan to be recomputed
  new_plan_lanes = true;

  log("lanes input:", polygons);
};

// log a vector of polygons
void Course::log(const char *str, const poly_list_t &polys)
{
  unsigned npolys = polys.size();
  if (npolys > 0)
    {
      for (unsigned i = 0; i < npolys; ++i)
        {
          ROS_DEBUG("polygon[%u] = %d", i, polys.at(i).poly_id);
          unsigned start_seq = i;
          while (i+1 < npolys
                 && abs(polys.at(i+1).poly_id - polys.at(i).poly_id) == 1)
            {
              ++i;
            }
          if (start_seq == i)
            ROS_DEBUG("%s polygon at %d", str, polys.at(i).poly_id);
          else
            ROS_DEBUG("%s polygons from %d to %d",
                      str, polys.at(start_seq).poly_id, polys.at(i).poly_id);
        }
    }
  else
    {
      ROS_INFO("%s no polygons at all", str);
    }
}

// return true if current order does not match saved way-points
bool Course::new_waypts(void)
{
  if (saved_replan_num!=order->replan_num)
    return true;

  for (unsigned i = 0; i < art_msgs::Order::N_WAYPTS; ++i)
    if (saved_waypt_id[i] != order->waypt[i].id)
      return true;

  // still the same
  return false;
}

// reset course class
void Course::reset(void)
{
  ROS_INFO("Course class reset()");

  // TODO: figure out when this needs to happen and what to do
  start_pass_location = MapPose();

  // clear the previous plan
  plan.clear();
  aim_poly.poly_id = -1;
}

// replan after road block
ElementID Course::replan_roadblock(void)
{
  saved_replan_num=order->replan_num;

  // save current order way-points
  for (unsigned i = 0; i < art_msgs::Order::N_WAYPTS; ++i)
    {
      saved_waypt_id[i] = order->waypt[i].id;
      ROS_DEBUG("saved_waypt_id[%u] = %s",
		i, saved_waypt_id[i].name().str);
    }

  // Get closest polygon in current plan.
  int uturn_exit_index = 
    pops->getClosestPoly(plan,MapPose(estimate->pose.pose));

  MapPose exit_pose;
  exit_pose.map.x=plan.at(uturn_exit_index).midpoint.x;
  exit_pose.map.y=plan.at(uturn_exit_index).midpoint.y;

  // Should get lane left of current position.  If in transition, the
  // lane should be left of previous lane left.
  ElementID reverse_lane =
    pops->getReverseLane(polygons,exit_pose);

  ROS_INFO("Replan from lane %s", reverse_lane.lane_name().str);

  return reverse_lane;
}

// direction for crossing an intersection
Course::direction_t Course::intersection_direction(void)
{
  int w0_index =
    pops->getContainingPoly(polygons,
                            MapXY(order->waypt[0].mapxy));
  int w1_index =
    pops->getContainingPoly(polygons,
                            MapXY(order->waypt[1].mapxy));

  // give up unless both polygons are available
  if (w0_index < 0 || w1_index < 0)
    return Straight;

  float w0_heading = pops->PolyHeading(polygons.at(w0_index));
  float w1_heading = pops->PolyHeading(polygons.at(w1_index));
  float heading_change = normalize(w1_heading - w0_heading);
					    
  ROS_DEBUG("heading change from waypoint %s to %s is %.3f radians",
	    ElementID(order->waypt[0].id).name().str,
	    ElementID(order->waypt[1].id).name().str,
	    heading_change);

  if (fabsf(heading_change) < angles::from_degrees(30))
    return Straight;
  else if (heading_change > 0.0)
    return Left;
  else
    return Right;
}

// true if order has an upcoming stop way-point
//
// exit: sets stop_waypt, stop_poly if found
//
float Course::stop_waypt_distance(bool same_lane)
{
  for (unsigned i = 1; i < art_msgs::Order::N_WAYPTS; ++i)
    {
      // only consider way-points in the current lane
      if (same_lane
	  && !ElementID(order->waypt[i].id).same_lane(order->waypt[0].id))
	break;

      if (order->waypt[i].is_stop)
	{
	  // find stop way-point polygon
	  int stop_index =
            pops->getContainingPoly(polygons,
                                    MapXY(order->waypt[i].mapxy));
	  if (stop_index < 0)		// none found?
	    continue;			// keep looking

	  stop_poly = polygons.at(stop_index);
	  stop_waypt = WayPointNode(order->waypt[i]);
	  float wayptdist = distance_in_plan(MapPose(estimate->pose.pose),
                                             stop_waypt);
          ROS_DEBUG("Stop at waypoint %s is %.3fm away",
		    stop_waypt.id.name().str, wayptdist);
	  return wayptdist;
	}
    }
  return Infinite::distance;
}

// switch to previously selected passing lane
//
// entry:
//	adj_lane[passing_lane].id is the passing lane ID
//	adj_polys[passing_lane] contains its polygons
//	obstacle is polar coordinate of nearest obstacle in plan
// exit:
//	plan contains polygons to follow
//	passed_lane contains previous plan polygons
//	start_pass_location set to current pose, with polygon heading
//	returns true if successful
//
bool Course::switch_to_passing_lane()
{
  // find a polygon slightly ahead of the car
  int aim_index = find_aim_polygon(adj_polys[passing_lane]);
  if (aim_index == -1)
    {
      ROS_WARN("unable to pass, no polygon near the aiming point");
      return false;
    }

  // save original plan for checking when it is safe to return
  passed_lane = plan;

  // collect all the polygons from aim_index to end of passing lane
  plan.clear();
  pops->CollectPolys(adj_polys[passing_lane], plan, aim_index);
  
  log("switch_to_passing_lane() plan", plan);
  if (plan.empty())
    {
      ROS_WARN("no polygons in passing lane past aiming point");
      return false;
    }

  aim_poly=plan.at(0);
  MapXY aim_poly_midpt = pops->getPolyEdgeMidpoint(aim_poly);
  ROS_DEBUG("aiming at polygon %d, midpoint (%.3f, %.3f)",
           aim_poly.poly_id, aim_poly_midpt.x, aim_poly_midpt.y);

  MapXY start_point =
    pops->GetClosestPointToLine (pops->midpoint(aim_poly.p1,aim_poly.p4),
                                 pops->midpoint(aim_poly.p2,aim_poly.p3),
                                 MapXY(estimate->pose.pose.position),
                                 true);

  start_pass_location.map=start_point;
  start_pass_location.yaw=aim_poly.heading;

  ROS_INFO("passing starts at (%.3f, %.3f)",
           start_pass_location.map.x, start_pass_location.map.y);

  return true;
}

// return distance to upcoming U-turn way-point, Infinite::distance if none.
float Course::uturn_distance(void)
{
  int i = uturn_order_index();
  if (i < 0)
    return Infinite::distance;

  // find stop way-point polygon
  int stop_index =
    pops->getContainingPoly(polygons,
                            MapXY(order->waypt[i].mapxy));
  if (stop_index < 0)		// none found?
    return Infinite::distance;

  // save way-point and polygon for stop_line controller
  stop_poly = polygons.at(stop_index);
  stop_waypt = WayPointNode(order->waypt[i]);

  // compute distance remaining
  float wayptdist = distance_in_plan(MapPose(estimate->pose.pose), stop_waypt);
  ROS_DEBUG("U-turn at waypoint %s, %.3fm away",
	    stop_waypt.id.name().str, wayptdist);
  return wayptdist;
}

// return index of upcoming U-turn transition in order->waypt array
//
// A U-turn is represented in the RNDF by an exit way-point in one
// lane pointing to a matching entry way-point in an adjacent lane.
//
int Course::uturn_order_index(void)
{
  for (unsigned i = 1; i < art_msgs::Order::N_WAYPTS-1; ++i)
    {
      // only consider way-points in the current lane
      if (!ElementID(order->waypt[i].id).same_lane(ElementID(order->waypt[0].id)))
	break;
      
      if (uturn_waypt(i))
	return i;
    }
  return -1;
}

// return true if waypt[windex] and waypt[windex+1] are a U-turn pair
bool Course::uturn_waypt(unsigned windex)
{
  if (order->next_uturn < 0)
    return false;
    
  return (windex==(unsigned)order->next_uturn);
}

// check if zone way-point reached
//
// Considers a way-point reached when the front of the car is within
// zone_waypoint_radius of the way-point.
//
// exit: navdata->last_waypt updated
// returns: true if order->waypt[1] reached
//
bool Course::zone_waypoint_reached(void)
{
  bool found = false;
  waypoint_checked = true;
	  
  // polar coordinate of front bumper from estimated position
  Polar bumper_polar(0.0, ArtVehicle::front_bumper_px);
  float distance =
    Euclidean::DistanceToWaypt(bumper_polar,
                               MapPose(estimate->pose.pose),
                               WayPointNode(order->waypt[1]));

  if (distance <= zone_waypoint_radius)
    {
      // The car is near this way-point.
      ROS_DEBUG("reached zone waypoint %s, distance %.3fm",
                ElementID(order->waypt[1].id).name().str, distance);
      navdata->last_waypt = order->waypt[1].id;
      found = true;
    }
  else
    {
      ROS_DEBUG("distance to zone waypoint %s is %.3fm",
                ElementID(order->waypt[1].id).name().str, distance);
    }
  
  return found;
}

bool Course::zone_perimeter_reached(void)
{
  bool found = false;
  waypoint_checked = true;
  
  int w1_index =
    pops->getClosestPoly(polygons,
                         MapXY(order->waypt[1].mapxy));
  if (w1_index >= 0)
    {
      // form way-point pose using polygon heading
      // TODO: save somewhere
      MapPose w1_pose(order->waypt[1].mapxy, 
		      pops->PolyHeading(polygons.at(w1_index)));
      
#if 1
      // Is the bearing of the car from that pose within 90
      // degrees of the polygon heading?
      float bearing_from_w1 =
        bearing(w1_pose, MapXY(odom->pose.pose.position));
#else // experimental code -- not working right yet
      // Is the bearing of a point slightly ahead of the front bumper
      // from that pose within 90 degrees of the polygon heading?
      Polar bumper_polar(0.0,
			 (ArtVehicle::front_bumper_px
			  + DARPA_rules::stop_line_to_bumper));
      MapXY bumper_pos = Polar_to_MapXY(bumper_polar, odom->pos);
      float bearing_from_w1 = bearing(w1_pose, bumper_pos);
#endif
      if (fabsf(bearing_from_w1) < angles::from_degrees(90))
	{
	  // The car is "in front" of this way-point's pose.
          ROS_DEBUG("reached waypoint %s, bearing %.3f radians",
		    ElementID(order->waypt[1].id).name().str,
                    bearing_from_w1);
	  navdata->last_waypt = order->waypt[1].id;
	  found = true;
	}
    }
  
  return found;
}


bool Course::spot_waypoint_reached(void)
{
  bool found = false;
  waypoint_checked = true;
	  
  // polar coordinate of front bumper from estimated position
  Polar bumper_polar(0.0, ArtVehicle::front_bumper_px);
  float distance =
    Euclidean::DistanceToWaypt(bumper_polar, MapPose(estimate->pose.pose),
                               WayPointNode(order->waypt[1]));

  if (distance <= spot_waypoint_radius)
    {
      // The car is near this way-point.
      ROS_DEBUG("reached spot waypoint %s, distance %.3fm",
                ElementID(order->waypt[1].id).name().str, distance);
      navdata->last_waypt = order->waypt[1].id;
      found = true;
    }
  else
    {
      ROS_DEBUG("distance to spot waypoint %s is %.3fm",
                ElementID(order->waypt[1].id).name().str, distance);
    }

  return found;
}

float Course::max_speed_for_slow_down(const float& final_speed,
				      const float& distance,
				      const float& max,
				      const float& max_deceleration) {
  // This function answers the question:
  //
  // What is the fastest I could be going right now such that I can be
  // travelling at <final_speed> in <distance> without exceeding 
  // <max_deceleration> between now and then?
  //
  // It uses one of the basic kinematic equations:
  // Vf^2 = Vi^2 + 2 * a * (Xf - Xi)
  
  
  float vf2 = final_speed * final_speed;
  float tax = 2 * (-max_deceleration) * distance;
  
  // Return 0 if it's impossible to stop in time!
  if(tax > vf2)
    return 0.0;
  
  return fminf(max, sqrtf(vf2 - tax));
}

// This function answers the question:
//
// What is the fastest I could be going right now such that my
// heading changes <dheading> over the next <distance>, but I never
// exceed <maximum_yaw_rate>?
float Course::max_speed_for_change_in_heading(const float& dheading,
					      const float& distance,
					      const float& max,
					      const float& maximum_yaw_rate) 
{
  if (Epsilon::equal(dheading,0))
    {
      return max;
    }
  else
    {
      float new_speed=fminf(max,
                            fmaxf(max_speed_for_sharp,
                                  fabsf(heading_change_ratio *
                                        (maximum_yaw_rate / dheading)))); 
      ROS_DEBUG("slow for heading: distance: %.3f, dheading: %.3f, "
                "maximum_yaw_rate: %.3f, max_speed: %.3f, final: %.3f",
                distance, dheading, maximum_yaw_rate, max,new_speed); 
      return new_speed;
    }
}


float Course::get_yaw_spring_system(const Polar& aim_polar, 
				    int poly_id,
				    float poly_heading,
				    float max_yaw,
				    float curr_velocity,
				    float offset_ratio)
{
  float error = 0;
  float theta=-aim_polar.heading;
  float velocity = fmaxf(curr_velocity, Steering::steer_speed_min);
  nav_msgs::Odometry front_est;  
  Estimate::front_axle_pose(*estimate, front_est);
  ros::Duration frequency(1.0 / art_msgs::ArtHertz::NAVIGATOR);
  ros::Time time_in_future = (ros::Time::now()
                              + frequency
                              + ros::Duration(velocity * spring_lookahead));
  nav_msgs::Odometry pos_est;
  Estimate::control_pose(front_est, time_in_future, pos_est);
 
  if (poly_id >=0)
    {
      poly current_poly=plan.at(poly_id);
      posetype origin;			// (0, 0, 0)
      posetype cpoly(current_poly.midpoint.x, current_poly.midpoint.y,
		     poly_heading);
      rotate_translate_transform trans;
      trans.find_transform(cpoly,origin);
      posetype car(pos_est.pose.pose.position.x,
                   pos_est.pose.pose.position.y,
                   0.0);
      posetype car_rel=trans.apply_transform(car);

      float width=Euclidean::DistanceTo(current_poly.p2,current_poly.p3);

      // transverse offset error, positive if left of center (push right)
      error=car_rel.y;
      //      ART_MSG(1,"STEER error = %lf\n", error);

#if 1 // still experimental:

      if (!Epsilon::equal(offset_ratio, 0.0))
	{
	  // To steer for an offset from lane center, adjust error by
	  // subtracting offset from polygon midpoint to middle of
	  // left lane boundary minus width of car.  

	  MapXY mid_left_side =
	    pops->midpoint(current_poly.p1, current_poly.p2);
	  float half_lane_width =
	    Euclidean::DistanceTo(current_poly.midpoint, mid_left_side);
	  float lane_space = half_lane_width - ArtVehicle::halfwidth;
	  float error_offset = 0.0;
	  if (lane_space > 0.0)		// any room in this lane?
	    error_offset = offset_ratio * lane_space;
          ROS_DEBUG("error offset %.3f, half lane width %.3f, ratio %.3f",
		    error_offset, half_lane_width, offset_ratio);

	  // Increasing error term pushes right, decreasing left.
	  error -= error_offset;
	}
#endif
      error=fminf(fmaxf(-width,error),width);
      // heading error
      theta = normalize(MapPose(pos_est.pose.pose).yaw - poly_heading);
    }


  float cth = cosf(theta);

  float vcth = velocity*cth;

  if (fabsf(theta) >= HALFPI ||
      Epsilon::equal(cth,0.0) ||
      Epsilon::equal(vcth,0.0))
    {
      ART_MSG(8,"Spring system does not apply: heading offset %.3f", theta);
      if (Epsilon::equal(error,0)) {
	if (theta < 0)
	  return max_yaw;
	else return -max_yaw;
      }
      else {
	if (error > 0)
	  return max_yaw;
	else return -max_yaw;
      }
    }
  
  float d2=-k_theta*sinf(theta)/cth;
  float d1=-k_error*error/vcth;  
  
  // #ifdef NQE
  //   if (order->waypt[0].id==ElementID(2,1,3) &&
  //       order->waypt[1].id==ElementID(1,1,2))
  //     {
  //       d2=-0.7*sinf(theta)/cth;
  //       ART_MSG(1,"Taking special turn");
  //     }
  // #endif

  if ((Coordinates::sign(error) == Coordinates::sign(last_error)) &&
      (fabsf(error) > fabs(last_error)))
    d1*=k_int;

  last_error=error;
  float yaw=d1+d2;

  ROS_DEBUG("Heading spring systems values: error %.3f, dtheta %.3f, "
            "d1 %.3f, d2 %.3f, d1+d2 %.3f", error, theta, d1, d2, yaw);
  
  if (yaw < 0)
    return fmaxf(-max_yaw, yaw);
  return fminf(max_yaw, yaw);
}


bool Course::spot_ahead()
{
  for (uint i=0; i<art_msgs::Order::N_WAYPTS-1;i++)
    if (order->waypt[i].is_spot &&
	order->waypt[i+1].is_spot &&
	order->waypt[i].id.pt==1 &&
	order->waypt[i+1].id.pt==2)
      return true;

  return false;

}

bool Course::curr_spot()
{
  return order->waypt[0].is_spot;
}

#if 0
mapxy_list_t Course::calculate_zone_barrier_points() 
{
  mapxy_list_t spot_points;
  
  return spot_points;

  if (order->waypt[1].is_spot)
    return spot_points;

  posetype way_pose(order->waypt[1].mapxy.x,
                    order->waypt[1].mapxy.y,
		    atan2f(order->waypt[2].mapxy.y-order->waypt[1].mapxy.y,
			   order->waypt[2].mapxy.x-order->waypt[1].mapxy.x));
  
  rotate_translate_transform trans;
  trans.find_transform(posetype(),way_pose);
  
  posetype npose;
  npose=trans.apply_transform(posetype(1,order->waypt[1].lane_width,0));
  spot_points.push_back(npose);

  npose=trans.apply_transform(posetype(1,order->waypt[1].lane_width/2,0));
  spot_points.push_back(npose);
  
  npose=trans.apply_transform(posetype(1,0,0));
  spot_points.push_back(npose);
  
  npose=trans.apply_transform(posetype(1,-order->waypt[1].lane_width/2,0));
  spot_points.push_back(npose);
  
  npose=trans.apply_transform(posetype(1,-order->waypt[1].lane_width,0));
  spot_points.push_back(npose);

  return spot_points;
}

mapxy_list_t Course::calculate_spot_points(const std::vector<WayPointNode>& new_waypts) 
{
  mapxy_list_t spot_points;

  for (uint i=0; i<art_msgs::Order::N_WAYPTS-1;i++)
    if (new_waypts[i].is_spot &&
	new_waypts[i+1].is_spot &&
	new_waypts[i].id.pt==1 &&
	new_waypts[i+1].id.pt==2)
      {
	posetype way_pose(new_waypts[i].map.x,new_waypts[i].map.y,
			  atan2f(new_waypts[i+1].map.y-new_waypts[i].map.y,
				 new_waypts[i+1].map.x-new_waypts[i].map.x));

	float dist=Euclidean::DistanceTo(new_waypts[i+1].map,
					 new_waypts[i].map);
	rotate_translate_transform trans;
	trans.find_transform(posetype(),way_pose);
	
	posetype npose;

	npose=trans.apply_transform(posetype(0,new_waypts[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist,new_waypts[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,new_waypts[i].lane_width,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,new_waypts[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,0,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,-new_waypts[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,new_waypts[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist,-new_waypts[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(0,-new_waypts[i].lane_width/2,0));
	spot_points.push_back(npose);

      }
  return spot_points;
}

mapxy_list_t Course::calculate_spot_points() 
{
  mapxy_list_t spot_points;

  for (uint i=0; i<art_msgs::Order::N_WAYPTS-1;i++)
    if (order->waypt[i].is_spot &&
	order->waypt[i+1].is_spot &&
	order->waypt[i].id.pt==1 &&
	order->waypt[i+1].id.pt==2)
      {
	posetype way_pose(order->waypt[i].mapxy.x,
                          order->waypt[i].mapxy.y,
			  atan2f(order->waypt[i+1].mapxy.y-order->waypt[i].mapxy.y,
				 order->waypt[i+1].mapxy.x-order->waypt[i].mapxy.x));

	float dist=Euclidean::DistanceTo(order->waypt[i+1].mapxy,
					 order->waypt[i].mapxy);
	rotate_translate_transform trans;
	trans.find_transform(posetype(),way_pose);
	
	posetype npose;

	npose=trans.apply_transform(posetype(0,order->waypt[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist,order->waypt[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,order->waypt[i].lane_width,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,order->waypt[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,0,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,-order->waypt[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist+2,order->waypt[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(dist,-order->waypt[i].lane_width/2,0));
	spot_points.push_back(npose);

	npose=trans.apply_transform(posetype(0,-order->waypt[i].lane_width/2,0));
	spot_points.push_back(npose);

      }
  return spot_points;
}
#endif

bool Course::nqe_special(int i, int j)
{
#ifdef NQE
  
  typedef struct {
    ElementID start;
    ElementID end;
  } id_pair;
  
  static int num_pair=8;
  
  static id_pair id_table[]=
    {
      //AREA A
      {ElementID(1,1,6),ElementID(41,1,1)},
      {ElementID(1,2,5),ElementID(41,2,1)},
      {ElementID(41,1,7),ElementID(1,1,1)},
      {ElementID(41,2,7),ElementID(1,2,1)},
      //AREA B
      {ElementID(6,1,10),ElementID(5,1,1)},
      {ElementID(5,1,7),ElementID(6,1,1)},
      {ElementID(6,1,4),ElementID(8,2,1)},
      {ElementID(6,1,6),ElementID(7,1,1)},
    };
  
  for (int k=0; k< num_pair; k++)
    if (order->waypt[i].id==id_table[k].start &&
	order->waypt[j].id==id_table[k].end)
      return true;
#endif

  return false;
  
}
