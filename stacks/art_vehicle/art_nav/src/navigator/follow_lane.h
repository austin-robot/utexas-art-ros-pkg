/* -*- mode: C++ -*-
 *
 *  Navigator follow lane controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __FOLLOW_LANE_HH__
#define __FOLLOW_LANE_HH__

//class Avoid;
class FollowSafely;
class SlowForCurves;
class StopArea;
class StopLine;

class FollowLane: public Controller
{
public:

  FollowLane(Navigator *navptr, int _verbose);
  ~FollowLane();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

private:

  // way-point types
  typedef enum
    {
      Lane,
      LaneChange,
      Merge,
      Stop,
      Uturn,
      Zone,
      ZoneExit,
    } way_type_t;

  //Avoid       *avoid;
  FollowSafely	*follow_safely;
  SlowForCurves *slow_for_curves;
  StopArea	*stop_area;
  StopLine	*stop_line;

  way_type_t approaching_waypoint_type(WayPointNode &stop_point);
  void reset_me(void);
};

#endif // __FOLLOW_LANE_HH__
