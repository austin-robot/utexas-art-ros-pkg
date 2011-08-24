/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART lane observer base class interface.

     @author Michael Quinlan

 */

#ifndef _ART_OBSERVER_H_
#define _ART_OBSERVER_H_

#include <limits>
#include <art_msgs/ArtLanes.h>
#include <art_msgs/Observation.h>

/** @brief Observers base class. */
class Observer 
{
public:

  /** Shorter typedef for observer ID */
  typedef art_msgs::Observation::_oid_type Oid_t;

  /** Constructor
   *
   *  @param id observer ID
   *  @param name observer name
   */
  Observer(Oid_t id, const std::string &name)
  {
    observation_.oid = id;
    observation_.name = name;
    observation_.applicable = false;
    observation_.clear = false;
    observation_.time = std::numeric_limits<float>::signaling_NaN();
    observation_.distance = std::numeric_limits<float>::signaling_NaN();
    observation_.velocity = std::numeric_limits<float>::signaling_NaN();
    observation_.nobjects = 0;
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
