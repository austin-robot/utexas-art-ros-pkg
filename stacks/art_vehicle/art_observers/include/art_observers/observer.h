/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id: accel.h 1539 2011-05-09 04:09:20Z jack.oquin $
 */

/**  @file
 
     ART lane observer base class interface.

     @author Michael Quinlan

 */

#ifndef _ART_OBSERVER_H_
#define _ART_OBSERVER_H_

#include <vector>

#include <art_msgs/ArtLanes.h>
#include <art_msgs/Observation.h>
#include <art_msgs/ArtQuadrilateral.h>
#include <art_msgs/ArtLanes.h>

class Observer 
{
public:

  /// Shorter typedef for observer ID
  typedef art_msgs::Observation::_oid_type Oid_t;

  Observer(Oid_t id, const std::string &name)
  {
    observation_.oid = id;
    observation_.name = name;
  }
  ~Observer();

  virtual art_msgs::Observation
    update(int root_poly_id,
           art_msgs::ArtLanes &lane_quads,
           art_msgs::ArtLanes &obstacle_quads) = 0;

protected:
  art_msgs::Observation observation_;
};

#endif // _ART_OBSERVER_H_
