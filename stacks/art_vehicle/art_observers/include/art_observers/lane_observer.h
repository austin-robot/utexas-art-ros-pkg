/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART lane observers class interface.

     @author Michael Quinlan

 */

#ifndef _LANE_OBSERVER_H_
#define _LANE_OBSERVER_H_

#include <art_observers/filter.h>
#include <art_observers/observer.h>


/** @brief Generic lane observer class (to be replaced). */
class LaneObserver: public observers::Observer 
{
public:
  LaneObserver(Oid_t id, const std::string &name);
  ~LaneObserver();

  virtual art_msgs::Observation
    update(int root_poly_id,
           const art_msgs::ArtLanes &lane_quads,
           const art_msgs::ArtLanes &obstacle_quads);
  virtual art_msgs::Observation    
    updateAdj(int adj_poly_id, 
           art_msgs::ArtLanes &adj_lane_quads,
           art_msgs::ArtLanes &adj_obstacle_quads);

private:
  std::vector<float> distance_;

  MedianFilter distance_filter_;
  MeanFilter velocity_filter_;

  ros::Time prev_update_;
  ros::Time current_update_;
  art_msgs::Observation  
  updateAdjHelper(int adj_poly_id, 
                             art_msgs::ArtLanes &adj_lane_quads,
                             art_msgs::ArtLanes &adj_obstacle_quads);
};


#endif // _LANE_OBSERVER_H_
