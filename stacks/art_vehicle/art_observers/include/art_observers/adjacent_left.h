/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id: adjacent_left.h 1744 2011-10-27 16:07:00Z jack.oquin $
 */

/**  @file

     adjacent right left interface.

     @author Michael Quinlan, Jack O'Quin, Corbyn Salisbury

 */

#ifndef _ADJACENT_LEFT_OBSERVER_H_
#define _ADJACENT_LEFT_OBSERVER_H_

#include <art_observers/filter.h>
#include <art_observers/observer.h>
#include <art_map/PolyOps.h>
#include <art_observers/QuadrilateralOps.h>

namespace observers
{

/** @brief Adjacent left observer class. */
class AdjacentLeft: public Observer 
{
public:
  AdjacentLeft(art_observers::ObserversConfig &config);
  ~AdjacentLeft();

  virtual art_msgs::Observation
    update(const art_msgs::ArtLanes &local_map,
           const art_msgs::ArtLanes &obstacles,
	   MapPose pose_);

private:


  std::vector<float> distance_;

  MedianFilter distance_filter_;
  MeanFilter velocity_filter_;

  ros::Time prev_update_;
};

}; // namespace observers

#endif // _ADJACENT_LEFT_H_
