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
#include <art_observers/ObserversConfig.h>
#include <art_map/PolyOps.h>

namespace observers
{

/** @brief Observers base class. */
class Observer 
{
public:

  /** Shorter typedef for observer ID */
  typedef art_msgs::Observation::_oid_type Oid_t;

  /** Constructor.
   *
   *  @param config configuration structure
   *  @param id observer ID
   *  @param name observer name
   */
  Observer(art_observers::ObserversConfig &config,
	   Oid_t id, const std::string &name):
    config_(config)
  {
    observation_.oid = id;
    observation_.name = name;
    observation_.applicable = false;
    observation_.clear = false;
    observation_.time = std::numeric_limits<float>::infinity();
    observation_.distance = std::numeric_limits<float>::infinity();
    observation_.velocity = std::numeric_limits<float>::quiet_NaN();
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
   *  @param pose          current pose of robot
   *
   *  @todo Make pure virtual once deprecated version is deleted. 
   */
  virtual art_msgs::Observation
    update(const art_msgs::ArtLanes &local_map,
           const art_msgs::ArtLanes &obstacles,
	   MapPose pose) = 0;

  /** Used by all observers to get obstacles in polygons of interest
   *
   *  @todo move these out of this pure virtual base class.
   */
  bool pointInLane(float x, float y, art_msgs::ArtLanes lane);
  art_msgs::ArtLanes getObstaclesInLane(art_msgs::ArtLanes obstacles,
                                        art_msgs::ArtLanes lane_quads);

protected:
  art_msgs::Observation observation_;
  art_observers::ObserversConfig config_;
};

}; // namespace observers

#endif // _ART_OBSERVER_H_
