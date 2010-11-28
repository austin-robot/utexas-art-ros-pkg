/* -*- mode: C++ -*-
 *
 *  Navigator U-turn controller
 *
 *  Copyright (C) 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __UTURN_HH__
#define __UTURN_HH__

class Safety;
class Stop;

class Uturn: public Controller
{
public:

  Uturn(Navigator *navptr, int _verbose);
  ~Uturn();
  void configure();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:

  // simple state machine
  typedef enum
    {
      Backward,				// moving backward
      Forward,				// moving forward
      Wait,				// wait until lane clear
    } state_t;

  state_t state;			// current FSM state
  bool do_init;

  // subordinate controllers
  //Safety *safety;
  Stop	*stop;

  // private data
  float goal_heading;			// heading of goal way-point
  poly uturn_exit;		        // current U-turn exit poly
  poly uturn_entry;		        // U-turn entry (goal) poly
  poly_list_t  uturn_polys;		// polygons for U-turn lanes

  float calculate_arc_length(bool forward, const MapXY& center, 
			     float safety_radius,
			     const MapXY& p1, const MapXY& p2);
  bool circle_and_line_intersect(MapXY center, float radius,
				 MapXY p1, MapXY p2,
				 MapXY &meet_point);
  float estimate_uturn_distance(bool forward, float desired_arc_length);
  Controller::result_t initialize(void);
  bool outside_lanes_front(void);
  bool outside_lanes_rear(void);
  bool point_outside_lanes(MapXY point);
  void reset_me(void);
  void set_state(state_t newstate);
  MapXY wheel_location(float x, float y);
};

#endif // __UTURN_HH__
