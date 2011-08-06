/*
 *  Navigator U-turn controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <angles/angles.h>

#include "navigator_internal.h"
#include "Controller.h"
#include "course.h"
#include "obstacle.h"
#include "uturn.h"
//#include "safety.h"
#include "stop.h"

#include <art_map/coordinates.h>
using namespace Coordinates;
#include <art_map/rotate_translate_transform.h>
#include <art_msgs/ArtVehicle.h>
using art_msgs::ArtVehicle;

// PFB: TODO: get all left lanes and take first one going other
// direction.

// PFB TODO: Pad end of polygons until we get 15 meters on both side.
// Then define "inside" or "outside" of lanes for points on robot we
// care about (tires, bumper, etc.): if line segment connecting point
// to closest point on right intersects left, then outside (and vice
// versa).  

// TODO Cont'd: If inside use first intersection point on circle.  If
// outside, use second.  Only turn M_PI forward or backwards (u-turn
// is theorectically M_PI turnaround (this *may* cause one more
// backup, but prevents over shooting in other cases).

static const char *state_name[] =
  {
    "Backward",
    "Forward",
    "Wait",
  };

Uturn::Uturn(Navigator *navptr, int _verbose):
  Controller(navptr, _verbose)
{
  ROS_INFO("vehicle turn radius %.3fm, front, rear wheel radii %.3fm, %.3f",
           ArtVehicle::turn_radius,
           ArtVehicle::front_outer_wheel_turn_radius,
           ArtVehicle::rear_outer_wheel_turn_radius);
  //safety = new Safety(navptr, _verbose);
  stop = new Stop(navptr, _verbose);
  reset_me();
}

Uturn::~Uturn()
{
  //delete safety;
  delete stop;
}

// returns true if circle and line segment intersect
//
// exit: sets first meeting point, if they do
//
// In 2D geometry, a circle and a line intersect at zero, one or two
// points.  Those are the zeroes of a real-valued quadratic equation,
// so their number is determined by its discriminant.  Since p1 and p2
// describe a line segment, only solutions between the two are of
// interest.  P1 is the first of the two in the direction of the lane.
//
// Let D(p,q) represent the Euclidean distance from point p to q.  Let
// L(p1,p2) be the line segment from p1 to p2, the set of all points q
// such that D(p1,q) + D(q,p2) = D(p1,p2).  Let C(c,r) be the circle
// of radius r around center point c, the set of all points q such
// that D(c,q) = r.  Transforming all these points from MapXY into a
// frame of reference with origin at the center of the circle
// simplifies the equations for points (x,y) belonging to the line
// through L and the circle C:
//
//	y = s*x + k			(line L)
//	y^2 + x^2 = r^2			(circle C)
//
// The slope (s) of the line is (p2.y-p1.y)/(p2.x-p1.x), and its
// intercept (k) is p1.y-(s*p1.x).
//
// Beware the special case of L being vertical: s would be infinite
// and x is a constant.  This requires special handling.  So does the
// special case of p1 = p2, where the line segment is a point.
//
// In all other cases, intersection points must satisfy both
// equations, so substitute the first equation for y in the second:
//
//	(s*x + k)^2 + x^2 = r^2
//
// Grouping this into a quadratic equation on x, we get:
//
//	(s^2+1)*x^2 + (2*s*k)*x + k^2 - r^2 = 0
//
// The canonical form is a*x^2 + b*x + c = 0, so we have:
//
//	a = s^2 + 1
//	b = 2*s*k
//	c = k^2 - r^2
//
// The discriminant, D, of this equation (b^2-4*a*c) is:
//
//	 D = (2*s*k)^2 - 4*(s^2+1)*(k^2-r^2)
//	 D = 4 * (s^2*r^2 + r^2 - k^2))
//
// When D > 0 the equation has two roots (intersection points), when D
// is 0 there is one, and D < 0 means there are none.  As noted above,
// any roots outside the line segment L(p1,p2) do not count and must be
// ignored.
//
bool Uturn::circle_and_line_intersect(MapXY c, float r,
				      MapXY point1, MapXY point2,
				      MapXY &meetpt)
{
  bool intersects = false;
  bool have_roots = false;
  MapXY root1, root2;			// roots relative to center

  // transformed coordinates relative to center of circle:
  MapXY p1 = point1 - c;
  MapXY p2 = point2 - c;

  // Check for the special case of L being vertical.
  if (Epsilon::equal(p1.x, p2.x))
    {
      if (Epsilon::equal(p1.y, p2.y))
	{
	  // the two points are the same
	  if (verbose >= 4)
	    ART_MSG(8, "line segment is a point");
	  if (Epsilon::equal(r*r, p1.x*p1.x + p1.y*p1.y))
	    {
	      have_roots = true;
	      root1 = p1;
	      root2 = p1;
	    }
	}
      else
	{
	  // For a vertical line, x is a constant.
	  if (verbose >= 4)
	    ART_MSG(8, "line segment is vertical");

	  root1.x = root2.x = p1.x;

	  // Compute y^2 from the formula for the circle.
	  float y2 = r*r - root1.x*root1.x;
	  if (y2 >= 0.0)
	    {
	      have_roots = true;
	      root1.y = sqrtf(y2);
	      root2.y = -root1.y;
	    }
	}
    }
  else
    {
      // normal quadratic equation:
      float s = (p2.y - p1.y) / (p2.x - p1.x);
      float k = p1.y - s*p1.x;
      float discriminant = 4.0 * (s*s * r*r + r*r - k*k);

      if (verbose >= 6)
	ART_MSG(8, "discriminant of circle and line y = (%.3f * x + %.3f)"
		" is %.3f", s, k, discriminant);

      // does the equation have any roots?
      if (discriminant >= 0.0)
	{
	  // compute quadratic roots
	  float a = s*s + 1.0f;
	  float b = 2.0f * s * k;
	  float c = k*k - r*r;

	  if (verbose >= 6)
	    ART_MSG(8, "y = (%.3f * x^2 + %.3f * x + %.3f)"
		    " has at least one root", a, b, c);

	  if (discriminant >= Epsilon::float_value
	      && fabsf(b) >= Epsilon::float_value)
	    {
	      // Compute roots of the quadratic using the floating
	      // point implementation trick in the Wikipedia
	      // "Quadratic Equation" article.  It ensures that the
	      // quantities added are of the same sign, avoiding
	      // catastrophic cancellation.  The root2 computation
	      // uses the fact that the product of the roots is c/a.
	      float t = (-(1.0f/2.0f) * (b + sign(b) * sqrtf(discriminant)));
	      root1.x = t/a;
	      root2.x = c/t;
	    }
	  else
	    {
	      // apply algebraic formula directly
	      root1.x = (-b - sqrtf(discriminant)) / 2 * a;
	      root2.x = (-b + sqrtf(discriminant)) / 2 * a;
	    }

	  // compute the y coordinates from the equation for the line
	  root1.y = s*root1.x + k;
	  root2.y = s*root2.x + k;
	  have_roots = true;
	}
    }

  if (have_roots)
    {
      if (verbose >= 6)
	ART_MSG(8, "circle intersects (%.3f,%.3f), (%.3f,%.3f) at (%.3f,%.3f)"
		" and (%.3f,%.3f)",
		p1.x, p1.y, p2.x, p2.y,
		root1.x, root1.y, root2.x, root2.y);

      // see if the roots are contained in the line segment
      bool r1_valid = Euclidean::point_in_line_segment(root1, p1, p2);
      bool r2_valid = Euclidean::point_in_line_segment(root2, p1, p2);

      intersects = (r1_valid || r2_valid);

      // return the appropriate meeting point
      if (r1_valid && r2_valid)
	{
	  // see which of root1 and root2 comes first in the lane
	  if (Euclidean::DistanceTo(root1, p1)
	      <= Euclidean::DistanceTo(root2, p1))
	    meetpt = root1 + c;
	  else
	    meetpt = root2 + c;
	}
      else if (r1_valid)
	{
	  meetpt = root1 + c;
	}
      else if (r2_valid)
	{
	  meetpt = root2 + c;
	}
    }

  if (intersects)
    {
      if (verbose >= 4)
	ART_MSG(8, "U-turn intersects (%.3f,%.3f), (%.3f,%.3f) at (%.3f,%.3f)",
		point1.x, point1.y, point2.x, point2.y,
		meetpt.x, meetpt.y);
    }
  else if (verbose >= 5)
    ART_MSG(8, "U-turn does not intersect (%.3f,%.3f), (%.3f,%.3f)",
	    point1.x, point1.y, point2.x, point2.y);

  return intersects;
}

// perform U-turn
//
// All these calculations are done using the absolute value of the
// speed.  The run controller will convert them later if the
// navdata->reverse flag gets set.
//
// entry:
//	navdata->reverse indicates current direction of travel
// exit:
//	navdata->reverse indicates desired direction of travel
// result:
//	OK, if able to continue or waiting for obstacle
//	Finished, if U-turn completed
//	NotApplicable, if no U-turn in order or no polygons available
//	safety controller results, if applicable
//
Controller::result_t Uturn::control(pilot_command_t &pcmd)
{
  // This controller never marks a way-point officially reached.
  // Don't want run controller to get upset about that.
  course->no_waypoint_reached();

  result_t result = OK;
  if (do_init)
    {
      result = initialize();
      if (result == NotApplicable)
	return result;
    }

  if (verbose >= 2)
    {
      // car may not be at a U-turn way-point.
      ART_MSG(5, "U-turn from near waypoint %s to %s, state is %s",
	      uturn_exit.start_way.name().str, 
	      uturn_entry.start_way.name().str,
	      state_name[state]);
    }

  // set Pilot command for slow speed, hard left turn
  pcmd.velocity = fminf(pcmd.velocity, config_->uturn_speed);
  pcmd.yawRate = config_->uturn_yaw_rate;

  // The car's heading monotonically approaches that of the goal
  // way-point from right to left.  Due to lane heading differences,
  // the remaining angle could be a little more than 180 degrees.
  // Convert a normalized angle to the right rear into a larger left
  // turn, ranging from -90 to 270 degrees.  When it goes negative, we
  // have turned too far.


  float remaining_angle = normalize(goal_heading
                                    - MapPose(estimate->pose.pose).yaw);
  if (remaining_angle < -HALFPI)
    remaining_angle += TWOPI;

  float desired_arc_length = remaining_angle * ArtVehicle::turn_radius;
  ROS_DEBUG("%.3f radians (%.f degrees) remain for U-turn, %.3fm arc",
	    remaining_angle, angles::to_degrees(remaining_angle),
            desired_arc_length);

  switch (state)
    {
    case Wait:
      // wait until left lane is clear.
      if (obstacle->observer_clear(Observation::Adjacent_left))
	{
	  if (verbose)
	    ART_MSG(1, "U-turn entry lane clear");
	  set_state(Forward);
	}
      else
	{
	  // TODO: check a timer.  What if it expires?  Return Blocked?
	  if (verbose)
	    ART_MSG(2, "U-turn entry lane blocked");
	}
      pcmd.velocity = 0.0;		// wait for lane to clear
      break;

    case Forward:
      {
	if (outside_lanes_front())	// gone too far forward?
	  {
	    if (navdata->stopped)
	      {
		set_state(Backward);
		navdata->reverse = true;
	      }
	    pcmd.velocity = 0.0;
	    break;
	  }
	
	float arc_length = estimate_uturn_distance(true, desired_arc_length);
#if 0
 	if (remaining_angle <= config_->uturn_stop_heading
	    || arc_length > desired_arc_length)
#else   // Pat's way (Pat want to make sure we are catching this on
	// both sides, so normalize the heading):
	if (fabs(Coordinates::normalize(remaining_angle)) <= config_->uturn_stop_heading)
#endif
	  {
	    // The car has a clear path to the goal heading or the
	    // remaining angle is close enough for follow_lane to
	    // handle.  Since the car is still moving, it's good to
	    // finish a little early to avoid turning too far.
	    result = Finished;
	    break;
	  }

	if (stop->control(pcmd, arc_length,
			  config_->uturn_threshold, config_->uturn_speed) == Finished)
	  {
	    set_state(Backward);
	    navdata->reverse = true;
	  }
      }
      break;

    case Backward:
      if (outside_lanes_rear()	// gone too far backward?
	  || (!outside_lanes_front()
	      && (estimate_uturn_distance(true, desired_arc_length)
		  > desired_arc_length)))
	{
	  // The car has either gone too far backwards or there is a
	  // clear path to goal heading going forward.  Stop and go
	  // forward again.
	  if (navdata->stopped)
	    {
	      set_state(Forward);
	      navdata->reverse = false;
	    }
	  pcmd.velocity = 0.0;
	}
      else  // continue backing
	{
	  // estimate distance to lane boundary
	  float arc_length = estimate_uturn_distance(false,
						     desired_arc_length);

	  // continue backing
	  if (stop->control(pcmd, arc_length, config_->uturn_threshold) == Finished)
	    {
	      // this is far enough
	      set_state(Forward);
	      navdata->reverse = false;
	    }
	}
      break;
    };

#if 0 // not using safety controller yet
  // must always run this on the final command
  result_t sresult = safety->control(pcmd);
  if (result != Finished)
    result = sresult;			// return safety result

  // TODO: implement obstacle evasion if Unsafe or Blocked
  if (sresult == Blocked)
    switch (state) {
    case Forward:
      set_state(Backward);
      navdata->reverse = true;
      break;
    case Backward:
      set_state(Forward);
      navdata->reverse = false;
      break;
    default:
      break;
    }
#endif // not using safety controller yet

  trace("uturn controller", pcmd, result);

  return result;
};

float Uturn::calculate_arc_length(bool forward, 
				  const MapXY& center,
				  float safety_radius,
				  const MapXY& p1,
				  const MapXY& p2)
{
  MapXY curb_point;			// point where path meets curb
  
  float arc_length=Infinite::distance;
  
  // TODO: (1) Return both intersection points, it is possible
  // (though unlikely) for the second one to have a shorter arc
  // length.  (2) Compute the safety radii of both outside wheels,
  // and use the shorter arc distance of the two.
  
  // compute intersection of circle with polygon, set curb_point
  if (circle_and_line_intersect(center, safety_radius, p1, p2, curb_point))
    {
      // bearings of car and meeting point from center of circle
      float curb_bearing = bearing(center, curb_point);
      float car_bearing;
      if (forward)
	{
	  // going forward, use the front axle's bearing
	  Polar front_axle(0.0, ArtVehicle::wheelbase);
	  car_bearing =
            bearing(center,
                    Polar_to_MapXY(front_axle,
                                   MapPose(estimate->pose.pose)));
	}
      else
	{
	  // in reverse, use the rear axle's bearing
	  car_bearing = bearing(center, MapXY(estimate->pose.pose.position));
	}
      
      // determine angle from car to meeting point
      // (an arc to the left is non-negative)
      float arc = curb_bearing - car_bearing;
      while (arc < 0.0)
	arc += TWOPI;
      
      // distance from car bumper to meeting point around the circle
      arc_length = arc * ArtVehicle::turn_radius;
      
      if (verbose >= 5)
	ART_MSG(5, "U-turn arc length %.3fm, bearing radians [%.3f,%.3f]",
		arc_length, car_bearing, curb_bearing);
    }
  return arc_length;
}

// returns distance to where estimated path intersects a curb
//
// This estimate assumes the car follows a circular path which may
// intersect the lane boundary at one or two points in each polygon,
// and determines the first intersection point along that path.
//
float Uturn::estimate_uturn_distance(bool forward, float desired_arc_length)
{
  // compute center point of circle
  MapXY center;				// center of curvature
  float safety_radius;
  if (forward)
    {
      // circle to left of the car
      center = Polar_to_MapXY(Polar(HALFPI, ArtVehicle::turn_radius),
			      MapPose(estimate->pose.pose));
      safety_radius = ArtVehicle::front_outer_wheel_turn_radius;
    }
  else
    {
      // circle to right of the car
      center = Polar_to_MapXY(Polar(-HALFPI, ArtVehicle::turn_radius),
			      MapPose(estimate->pose.pose));
      safety_radius = ArtVehicle::rear_outer_wheel_turn_radius;
    }
  
  if (verbose >= 2)
    ART_MSG(5, "U-turn circle center going %s is (%.3f,%.3f)",
	    (forward? "forward": "backward"), center.x, center.y);
  
  float min_arc_length = Infinite::distance;
  
  // check lane polygon right boundaries in the direction of lane heading
  for (unsigned pidx = 0; pidx < uturn_polys.size(); ++pidx)
    {
      MapXY p1 = uturn_polys.at(pidx).p4; // bottom right
      MapXY p2 = uturn_polys.at(pidx).p3; // top right

      float arc_length =
	calculate_arc_length(forward,center,safety_radius,p1,p2);      
      min_arc_length = fminf(arc_length, min_arc_length);
    }

  if (min_arc_length < desired_arc_length)
    {
      if (verbose >= 4)
	ART_MSG(5, "U-turn %s arc intersects in %.3fm",
		(forward? "forward": "backward"), min_arc_length);
      return min_arc_length;
    }
  
  int index=pops->getClosestPoly(uturn_polys,uturn_entry.midpoint);
  if (index >= 0)
    {
      MapXY p1 = uturn_polys.at(index).p4; // bottom right
      MapXY p2 = uturn_polys.at(index).p3; // top right

      MapXY mid=pops->midpoint(p1,p2);
      posetype origin(0,0,0);
      posetype origin_abs(mid.x,mid.y,bearing(p1,p2));
      
      posetype far(300,0,0);
      
      rotate_translate_transform long_trans;
      long_trans.find_transform(origin, origin_abs);
      posetype far_abs=long_trans.apply_transform(far);
      far.x*=-1;
      posetype nfar_abs=long_trans.apply_transform(far);
      
      p2.x=far_abs.x;
      p2.y=far_abs.y;
      p1.x=nfar_abs.x;
      p1.y=nfar_abs.y;

  
      float arc_length =
	calculate_arc_length(forward,center,safety_radius,p1,p2);      
      min_arc_length = fminf(arc_length, min_arc_length);
    }
  
  index=pops->getClosestPoly(uturn_polys,uturn_exit.midpoint);
  if (index >= 0)
    {
      MapXY p1 = uturn_polys.at(index).p4; // bottom right
      MapXY p2 = uturn_polys.at(index).p3; // top right
      
      MapXY mid=pops->midpoint(p1,p2);
      posetype origin(0,0,0);
      posetype origin_abs(mid.x,mid.y,bearing(p1,p2));
      
      posetype far(300,0,0);
      
      rotate_translate_transform long_trans;
      long_trans.find_transform(origin, origin_abs);
      posetype far_abs=long_trans.apply_transform(far);
      far.x*=-1;
      posetype nfar_abs=long_trans.apply_transform(far);
      
      p2.x=far_abs.x;
      p2.y=far_abs.y;
      p1.x=nfar_abs.x;
      p1.y=nfar_abs.y;

      float arc_length =
	calculate_arc_length(forward,center,safety_radius,p1,p2);      
      min_arc_length = fminf(arc_length, min_arc_length);
    }
      
  if (min_arc_length < Infinite::distance)
    {
      if (verbose >= 4)
	ART_MSG(5, "U-turn %s arc intersects in %.3fm",
		(forward? "forward": "backward"), min_arc_length);
    }
  else
    {
      if (verbose >= 5)
	ART_MSG(2, "U-turn %s arc does not intersect curb locally",
		(forward? "forward": "backward"));
    }
  
  return min_arc_length;
}

// initialize U-turn operation
Controller::result_t Uturn::initialize(void)
{
  poly_list_t current_lane_polys, left_lane_polys;


  pops->AddLanePolys(course->polygons, current_lane_polys,
		     order->waypt[0].id);

  int uturn_exit_index = 
    pops->getClosestPoly(current_lane_polys,
                         MapPose(estimate->pose.pose));

  int uturn_entry_index=-1;

  if (uturn_exit_index >= 0)
    {
      MapXY exit_pose;
      exit_pose = current_lane_polys.at(uturn_exit_index).midpoint;
      pops->AddLanePolys(course->polygons, left_lane_polys,
                         order->waypt[1].id);
      uturn_entry_index = pops->getClosestPoly(left_lane_polys, exit_pose);
    }
  
  if (uturn_exit_index < 0 || uturn_entry_index < 0)
    {
      ROS_ERROR("no U-turn according to polygons: %d %d", 
		uturn_exit_index, uturn_entry_index);
      return NotApplicable;
    }
  
  do_init = false;

  uturn_exit = current_lane_polys.at(uturn_exit_index);
  uturn_entry = left_lane_polys.at(uturn_entry_index);

  for (uint i=0; i<current_lane_polys.size();i++)
    if (Euclidean::DistanceTo(current_lane_polys[i].midpoint,
			      uturn_exit.midpoint) <= 15.0)
      uturn_polys.push_back(current_lane_polys[i]);

  for (uint i=0; i<left_lane_polys.size();i++)
    if (Euclidean::DistanceTo(left_lane_polys[i].midpoint,
			      uturn_entry.midpoint) <= 15.0)
      uturn_polys.push_back(left_lane_polys[i]);

  
  // TODO: make sure all four wheels are inside the uturn_polys.

  course->log("U-turn", uturn_polys);

  if (uturn_polys.empty())
    return NotApplicable;

  // get polygon index of uturn_entry, the goal way-point
  int p_index = pops->getContainingPoly(uturn_polys, uturn_entry.midpoint);
  if (p_index < 0)
    {
      ART_ERROR("no polygon for U-turn entry waypoint.");
      return NotApplicable;
    }

  // save heading of the goal polygon
  goal_heading = pops->PolyHeading(uturn_polys.at(p_index));

  return OK;
}

// return true when either front wheel is outside the U-turn lanes.
bool Uturn::outside_lanes_front(void)
{
  if (point_outside_lanes(wheel_location(ArtVehicle::front_right_wheel_px,
					 ArtVehicle::front_right_wheel_py)))
    {
      if (verbose >= 3)
	ART_MSG(6, "right front wheel is outside the U-turn lanes");
      return true;
    }

  if (point_outside_lanes(wheel_location(ArtVehicle::front_left_wheel_px,
					 ArtVehicle::front_left_wheel_py)))
    {
      if (verbose >= 3)
	ART_MSG(6, "left front wheel is outside the U-turn lanes");
      return true;
    }

  return false;
}

// return true when either rear wheel is outside the U-turn lanes.
bool Uturn::outside_lanes_rear(void)
{
  if (point_outside_lanes(wheel_location(ArtVehicle::rear_right_wheel_px,
					 ArtVehicle::rear_right_wheel_py)))
    {
      if (verbose >= 3)
	ART_MSG(6, "right rear wheel is outside the U-turn lanes");
      return true;
    }

  if (point_outside_lanes(wheel_location(ArtVehicle::rear_left_wheel_px,
					 ArtVehicle::rear_left_wheel_py)))
    {
      if (verbose >= 3)
	ART_MSG(6, "left rear wheel is outside the U-turn lanes");
      return true;
    }

  return false;
}

// return true if point is outside the U-turn lanes
bool Uturn::point_outside_lanes(MapXY point)
{
  int pindex = pops->getClosestPoly(uturn_polys, point);
  if (pindex < 0)			// no polygon found?
    return true;			// must be outside lane

  poly nearest = uturn_polys.at(pindex);

  // get the bearing of the point from right side of polygon
  float point_bearing = normalize(bearing(nearest.p4, point)
				  - bearing(nearest.p4, nearest.p3));

  if (point_bearing >= 0.0)
    return false;			// point is left of right curb

  if (verbose >= 4)
    ART_MSG(7, "point (%.3f %.3f) is outside nearest polygon, %d",
	    point.x, point.y, nearest.poly_id);

  return true;
}

// reset all subordinate controllers
void Uturn::reset(void)
{
  trace_reset("Uturn");
  reset_me();
  //safety->reset();
  stop->reset();
}

// reset controller
void Uturn::reset_me(void)
{
  navdata->reverse = false;
  state = Wait;
  do_init = true;
  uturn_polys.clear();
}

// set new state
void Uturn::set_state(state_t newstate)
{
  if (state != newstate)
    {
      if (verbose)
	ART_MSG(4, "U-turn state changing from %s to %s",
		state_name[state], state_name[newstate]);
      state = newstate;
    }
}

// return MapXY position of egocentric point
MapXY Uturn::wheel_location(float x, float y)
{
  posetype vehicle_relative(0, 0, 0);
  posetype wheel_relative(x, y, 0);
  MapPose pose2d(estimate->pose.pose);
  posetype vehicle_map(pose2d.map.x, pose2d.map.y, pose2d.yaw);
  rotate_translate_transform trans;
  trans.find_transform(vehicle_relative, vehicle_map);
  posetype wheel_map = trans.apply_transform(wheel_relative);
  return MapXY(wheel_map.x, wheel_map.y);
}
