/* -*- mode: C++ -*-
 *
 *  DARPA Urban Challenge rules description
 *
 *  Copyright (C) 2009 Austin Robot Technology                    
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _DARPA_rules_H_
#define _DARPA_rules_H_

/**  @file
   
     @brief DARPA Urban Challenge rules description

     This class encapsulates constants and methods required or implied
     by the rules of the DARPA Urban Challenge.  All distances are in
     meters, times in seconds.  Section numbers refer to the official
     DARPA Technical Evaluation Criteria document.
 
     http://www.darpa.mil/grandchallenge/rules.asp/Technical_Evaluation_Criteria_031607.pdf
 */

#include <algorithm>
#include <math.h>
#include <art/conversions.h>

#include <art_msgs/ArtVehicle.h>

namespace DARPA_rules
{
  // specific to our vehicle
  const float vehicle_length = art_msgs::ArtVehicle::length;
  const float vehicle_width = art_msgs::ArtVehicle::width;

  // A.4. Stay in lane
  // Stay at least this far away from the road's center line.
  const float lane_center_offset = 1.0f;

  // A.6. Excess delay
  const float excess_delay = 10.0f;

  // A.8. Stop line
  const float stop_line_to_bumper = 1.0f;

  // A.9. Vehicle separation
  const float stop_line_safety_area = 30.0f;
  const float min_forw_sep_safety = 2.0f;
  const float min_forw_sep_travel = vehicle_length;

  const float min_standoff_dist = 1.0f;

  static inline float forw_sep_travel(float speed)
  {
    float speed_mph = mps2mph(speed);
    return fmaxf((speed_mph/10.0f)*vehicle_length, min_forw_sep_travel);
  }

  // A.10. Leaving lane to pass
  const float min_forw_sep_to_pass = vehicle_length;

  // A.11. Returning to lane after pass
  const float min_rear_sep_after_pass = vehicle_length;
  const float front_limit_after_pass = vehicle_length * 4.0f;

  // A.12. U-turn
  const float max_dist_from_entry_exit = 15.0f;

  // D.4. Left turn
  const float max_left_turn_delay = 10.0f;

  // D.5. Vehicle separation during left turn
  const float min_left_turn_sep = vehicle_length * 2.0f;
};

#endif
