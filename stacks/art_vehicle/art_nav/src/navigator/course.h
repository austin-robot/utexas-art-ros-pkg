/* -*- mode: C++ -*-
 *
 *  Navigator course planning class
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _COURSE_HH_
#define _COURSE_HH_

#include <vector>

#include <art/infinity.h>
#include <art_msgs/ArtLanes.h>
#include <art_map/coordinates.h>
#include <art_map/zones.h>

#include "Controller.h"

/** @brief Navigator course planning class. */
class Course
{
 public:

  // intersection crossing directions
  typedef enum {
    Right = -1,
    Straight = 0,
    Left = 1
  } direction_t;

  /** @brief Constructor */
  Course(Navigator *_nav, int _verbose);

  /** @brief Destructor */
  ~Course() {};

  /** @brief Course class initialization for run state cycle. */
  void begin_run_cycle(void);

  /** @brief set configuration variables. */
  void configure();

  /** @brief set heading for desired course */
  void desired_heading(pilot_command_t &pcmd, float offset_ratio = 0.0);

  /** return distance in a lane to a way-point */
  float distance_in_plan(const MapPose &from,
			 const WayPointNode &wp) const;

  /** return distance in a lane to a pose */
  float distance_in_plan(const MapPose &from,
			 const MapPose &to) const;

  /** return distance in a lane to a pose */
  float distance_in_plan(const MapPose &from,
			 const MapXY &to) const;

  /** @brief Course class termination for run state cycle. */
  void end_run_cycle(void);

  /** find the index in lane of a polygon to aim for ahead of the car
   *  (returns -1 if none)
   */
  int find_aim_polygon(poly_list_t &lane);

  /** @brief Find a passing lane through the polygons. */
  bool find_passing_lane(void);

  /** @brief Find a path in the travel lane to the next few way-points. */
  void find_travel_lane(bool rejoin);

  /** @brief return true if pose is in the current travel lane */
  bool in_lane(const MapPose &pose) const
  {
    return in_poly_list(plan, pose);
  }

  /** @brief return true if pose is in the polys list */
  bool in_poly_list(const poly_list_t &polys, const MapPose &pose) const
  {
    return (pops->getContainingPoly(polys, pose) >= 0);
  }

  /** @brief return intersection crossing direction */
  direction_t intersection_direction(void);

  /** @brief return lane change direction */
  direction_t lane_change_direction(void);

  /** @brief return true if lane way-point reached */
  bool lane_waypoint_reached(void);

  /** @brief handle lanes message
   *
   *  Called from the driver ProcessMessage() handler when new lanes
   *  data arrive.
   *
   * @param lanes pointer to the lanes message in the player message
   * queue.  Must copy the data before returning.
   */
  void lanes_message(const art_msgs::ArtLanes &lanes);

  /** @brief log a vector of polygons */
  void log(const char *str, const poly_list_t &polys);

  /** @brief confirm that the next way-point was reached */
  void new_waypoint_reached(ElementID new_way)
  {
    waypoint_checked = true;
    navdata->last_waypt = new_way.toMapID();
    ROS_DEBUG_STREAM("reached waypoint %s" << new_way.name().str);
  };

  /** @brief return true if current order does not match saved
   *  way-points */
  bool new_waypts(void);

  /** @brief confirm that no way-point was reached */
  void no_waypoint_reached(void)
  {
    waypoint_checked = true;
  };

  /** @brief return whether the current plan is still valid
   *
   * Technically, a plan is no longer valid after we receive new
   * polygons, but that is currently ignored for simplicity.
   */
  bool plan_valid(void)
  {
    // check that in the current order waypts match
    for (int i = 0; i < Order::N_WAYPTS; ++i)
      {
	if (plan_waypt[i] != order->waypt[i].id)
	  return false;			// need a new plan
      }
    return (!plan.empty()		// valid if plan exists
	    && !new_plan_lanes);	//   and no new lane polygons
  }

  /** @brief replan after road block
   *
   * exit: saves current order way-points
   * returns: ElementID of way-point from which to replan
   *		(null ElementID if unable to find alternate)
   */
  ElementID replan_roadblock(void);

  /** @brief reset course */
  void reset(void);

  /** @brief are id1 and id2 in the same lane?
   *
   *  Beware of a segment that loops back to itself.  In that case,
   *  the lane is the same, but the way-point numbers decrease.
   */
  bool same_lane(ElementID id1, ElementID id2)
  {
    return (id1.same_lane(id2) && id1.pt <= id2.pt);
  }

  /** @brief return distance to stop way-point, Infinite::distance if none 
   *
   * exit: sets stop_waypt, if found
   */
  float stop_waypt_distance(bool same_lane);

  /** @brief set turn signal for passing */
  void signal_pass(void)
  {
    turn_signal_on(passing_left);
  }

  /** @brief set turn signal for return from passing */
  void signal_pass_return(void)
  {
    turn_signal_on(!passing_left);
  }

  /** @brief special way-point predicate
   *
   * @param windex an index in the order->waypt[] array.
   * @return true for a special way-point (stop, or U-turn)
   */
  bool special_waypt(unsigned windex)
  {
    return (order->waypt[windex].is_stop
	    || uturn_waypt(windex));
  }

  /** @brief set turn signal based on direction */
  void signal_for_direction(direction_t direction)
  {
    if (direction == Straight)
      {
	turn_signals_off();
      }
    else
      {
	// turning: true if Left
	turn_signal_on(direction == Left);
      }
  }

  /** @brief switch to previously selected passing lane */
  bool switch_to_passing_lane();

  /** @brief set both turn signals on */
  void turn_signals_both_on(void)
  {
    if (navdata->signal_left || navdata->signal_right)
      {
	navdata->signal_left = true;
	navdata->signal_right = true;
        ROS_DEBUG("setting both turn signals on");
      }
  }

  /** @brief set both turn signals off */
  void turn_signals_off(void)
  {
    if (navdata->signal_left || navdata->signal_right)
      {
	navdata->signal_left = false;
	navdata->signal_right = false;
        ROS_DEBUG("setting turn signals off");
      }
  }

  /** request a turn signal on (direction true for left turns) */
  void turn_signal_on(bool direction)
  {
    if (navdata->signal_left != direction
	|| navdata->signal_right != !direction)
      {
	navdata->signal_left = direction;
	navdata->signal_right = !direction;
        ROS_DEBUG("signalling %s", (direction? "left": "right"));
      }
  }

  /** @brief return distance to U-turn way-point, Infinite::distance if none.
   *
   * exit: sets stop_waypt to U-turn exit point, if found
   */
  float uturn_distance(void);

  /** @brief return index of U-turn exit in order->waypt array */
  int uturn_order_index(void);

  /** @brief return true if waypt[windex] and waypt[windex+1] are a
   *  U-turn pair */
  bool uturn_waypt(unsigned windex);

  /** @brief return true if zone way-point reached */
  bool zone_waypoint_reached(void);
  bool zone_perimeter_reached(void);
  bool spot_waypoint_reached(void);

  // public class data
  poly_list_t polygons;			//< all polygons for local area
  poly_list_t plan;			//< planned course

  poly_list_t passed_lane;		//< original lane being passed
  bool passing_left;			//< when passing, true if to left
  MapPose start_pass_location;          //< pose where passing started

  WayPointNode stop_waypt;		//< coming stop or U-turn way-point
  poly stop_poly;			//< polygon containing stop waypt
  poly aim_poly;			//< aim polygon for rejoining plan
					//  (none if its poly_id = -1)
  ZonePerimeterList zones;

  
  float max_speed_for_slow_down(const float& final_speed,
				const float& distance,
				const float& max, 
				const float& max_deceleration);
  
  float max_speed_for_change_in_heading(const float& dheading,
					const float& distance,
					const float& max,
					const float& max_yaw_rate);

  float get_yaw_spring_system(const Polar& aim_polar, int poly_id,
			      float poly_heading,
			      float max_yaw, float curr_velocity,
			      float offset_ratio = 0.0);

  bool spot_ahead();
  mapxy_list_t calculate_spot_points(const std::vector<WayPointNode>
                                     &new_waypts);
  mapxy_list_t calculate_spot_points();
  mapxy_list_t calculate_zone_barrier_points();
  bool curr_spot();

  bool nqe_special(int i, int j);

 private:

  // Internal state.  Some of these vectors are class variables to
  // minimize dynamic memory allocation, instead of making them
  // automatic.
  ElementID plan_waypt[art_msgs::Order::N_WAYPTS]; //< waypts in the plan
  bool new_plan_lanes;			//< new lanes since plan made
  bool waypoint_checked;
  int poly_index;			// index in polygons of odom pose

  // Passing lane selection data.
  ElementID adj_lane[2];		// adjacent lane IDs
  poly_list_t adj_polys[2];		// adjacent lanes in segment
  int passing_lane;			// index of passing lane (or -1)

  // saved order way-points for road block
  ElementID saved_waypt_id[art_msgs::Order::N_WAYPTS];
  int saved_replan_num;

  // .cfg variables
  double heading_change_ratio;
  double k_error;
  double k_int;
  double k_theta;
  double lane_change_secs;
  //double lane_steer_time;
  double last_error;
  double max_speed_for_sharp;
  double max_yaw_rate;
  //double min_lane_change_dist;
  double min_lane_steer_dist;
  double spot_waypoint_radius;
  double spring_lookahead;
  double turning_latency;
  //double yaw_ratio;
  //double zone_perimeter_radius;
  double zone_waypoint_radius;

  // constructor parameters
  int verbose;				// message verbosity level
  Navigator *nav;			// internal navigator class

  // convenience pointers to Navigator class data
  PolyOps* pops;			// polygon operations class
  art_msgs::Order *order;               // current commander order
  art_msgs::NavigatorState *navdata;   // current navigator state data
  nav_msgs::Odometry *odom;             // current odometry position
  nav_msgs::Odometry *estimate;         // estimated control position
  const Config *config_;

  /** @brief head directly for next reachable way-point */
  Polar head_for_waypt(float target_dist);

  /** @brief set new plan way-points */
  void set_plan_waypts(void)
  {
    for (int i = 0; i < Order::N_WAYPTS; ++i)
      plan_waypt[i] = order->waypt[i].id;
  }


};

#endif // _COURSE_HH_
