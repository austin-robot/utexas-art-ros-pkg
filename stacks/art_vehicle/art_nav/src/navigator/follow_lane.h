//
// Navigator follow lane controller
//
//  Copyright (C) 2007 Austin Robot Technology
//  All Rights Reserved. Licensed Software.
//
//  This is unpublished proprietary source code of Austin Robot
//  Technology, Inc.  The copyright notice above does not evidence any
//  actual or intended publication of such source code.
//
//  PROPRIETARY INFORMATION, PROPERTY OF AUSTIN ROBOT TECHNOLOGY
//
//  $Id$
//
//  Author: Jack O'Quin
//

#ifndef __FOLLOW_LANE_HH__
#define __FOLLOW_LANE_HH__

class Avoid;
class FollowSafely;
class LaneHeading;
class SlowForCurves;
class StopArea;
class StopLine;

class FollowLane: public Controller
{
public:

  FollowLane(Navigator *navptr, int _verbose);
  ~FollowLane();
  void configure(ConfigFile* cf, int section);
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

  // .cfg variables
  float lost_speed;

  Avoid		*avoid;
  FollowSafely	*follow_safely;
  LaneHeading	*lane_heading;
  StopArea	*stop_area;
  StopLine	*stop_line;
  SlowForCurves *slow_for_curves;

  way_type_t approaching_waypoint_type(WayPointNode &stop_point);
  void reset_me(void);
};

#endif // __FOLLOW_LANE_HH__
