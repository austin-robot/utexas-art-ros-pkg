/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology, Michael Quinlan
 *  License: Modified BSD Software License 
 */

/**  @file
 
     ART lane observer base class implementation.

     @author Michael Quinlan

 */

#include <art_observers/observer.h>

Observer::~Observer() {}

// Define stub for non-pure virtual function method to avoid linker
// error.  Can be removed once it is made pure virtual.
art_msgs::Observation
  Observer::update(const art_msgs::ArtQuadrilateral &robot_quad,
		   const art_msgs::ArtLanes &local_map,
		   const art_msgs::ArtLanes &obstacles)
{
  return art_msgs::Observation();
}

// Define stub for non-pure virtual function method to avoid linker
// error.  Can be removed once this deprecated function is deleted
art_msgs::Observation
  Observer::update(int robot_poly_id,
		   const art_msgs::ArtLanes &local_map,
		   const art_msgs::ArtLanes &obstacles)
{
  return art_msgs::Observation();
}
