/* -*- mode: C++ -*-
 *
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

#ifndef __PARKING_HH__
#define __PARKING_HH__


class Safety;
class Stop;
class Halt;

class PARK_control: public Controller
{
public:

  PARK_control(Navigator *navptr, int _verbose);
  ~PARK_control();
  void configure(ConfigFile* cf, int section);
  result_t control(pilot_command_t &pcmd, mapxy_list_t, bool voronoi_stuck=true);
  void reset(void);
  
private:
  // simple state machine

  typedef enum
    {
      hit_waypoint,
      approach_spot,
      pull_in,
      pull_out
    } state_t;


  state_t state;			// current FSM state


  Controller::result_t initialize(pilot_command_t& pcmd,
				  const mapxy_list_t&);

  // .cfg variables
  float parking_speed_limit;

  bool find_a_better_spot;
  float find_spot_max_x_offset;
  float find_spot_max_y_offset;
  float find_spot_step_size;


  // subordinate controllers
  Safety *safety;
  Stop	*stop;
  Halt *halt;

  int adjust_spot(std::vector<WayPointNode> &new_waypts,
		  ObstacleList obstacles,
		  float max_x_offset,
		  float max_y_offset,
		  float step_size);


  float speed_limit;

  float park_distance, park_turn;

  void reset_me();
  bool hit_way_point(float park_distance, float park_turn,
				 float dheading);
  player_pose2d_t new_end_pose;

  float min_distance,min_theta;

  bool small_segment(float distance, float turn);

  float park_turn_ratio;
  float park_max_speed;
  bool halting;

  pilot_command_t last_pcmd;

  float last_park_dist;
  float last_park_turn;
  float min_adj_dist;
  float lastYaw;
  std::vector<MapXY> spot_points;
  int min_obst;
};

#endif // __REED_SHEPP_HH__
