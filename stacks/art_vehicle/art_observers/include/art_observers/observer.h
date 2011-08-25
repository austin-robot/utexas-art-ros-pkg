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

  /** Generic observer update function.
   *
   *  Called whenever there are new obstacle data, assuming the
   *  local_map is also available.
   *
   *  @param robot_quad    quadrilateral containing the robot
   *  @param local_map     road map lanes within range of the robot
   *  @param obstacles     local map quads currently containing obstacles
   *
   *  @todo Make pure virtual once deprecated version is deleted. 
   */
  virtual art_msgs::Observation
    update(const art_msgs::ArtQuadrilateral &robot_quad,
	   const art_msgs::ArtLanes &local_map,
           const art_msgs::ArtLanes &obstacles);

  /** Deprecated update interface. */
  virtual art_msgs::Observation
    update(int robot_poly_id,
	   const art_msgs::ArtLanes &local_map,
           const art_msgs::ArtLanes &obstacles);

protected:
  art_msgs::Observation observation_;
};

#endif // _ART_OBSERVER_H_
