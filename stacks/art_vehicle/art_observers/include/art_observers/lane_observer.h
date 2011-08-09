/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id: accel.h 1539 2011-05-09 04:09:20Z jack.oquin $
 */

/**  @file
 
     ART lane observers class interface.

     @author Michael Quinlan

 */

#ifndef _LANE_OBSERVER_H_
#define _LANE_OBSERVER_H_

#include <vector>
#include <art_observers/filter.h>
#include <art_observers/observer.h>


class LaneObserver: public Observer 
{
public:
  LaneObserver();
  ~LaneObserver();

  virtual art_msgs::Observation
    update(int root_poly_id,
           art_msgs::ArtLanes &lane_quads,
           art_msgs::ArtLanes &obstacle_quads);

private:
  std::vector<float> distance_;

  MedianFilter distance_filter_;
  MeanFilter velocity_filter_;

  ros::Time prev_update_;
  ros::Time current_update_;
};


#endif // _LANE_OBSERVER_H_
