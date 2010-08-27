/* -*- mode: C++ -*-
 *
 *  Navigator lane heading controller
 *
 *  Copyright (C) 2007, 2010, Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef __LANE_HEADING_HH__
#define __LANE_HEADING_HH__


class LaneHeading: public Controller
{
public:

  LaneHeading(Navigator *navptr, int _verbose);
  ~LaneHeading();
  void configure();
  result_t control(pilot_command_t &pcmd);
  void reset(void);

};

#endif // __LANE_HEADING_HH__
