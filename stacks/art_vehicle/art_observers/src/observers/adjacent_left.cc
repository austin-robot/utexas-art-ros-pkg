/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology, Michael Quinlan
 *  Copyright (C) 2011 UT-Austin & Austin Robot Technology
 *  License: Modified BSD Software License 
 */

/**  @file

     Nearest forward observer implementation.

     @author Michael Quinlan, Jack O'Quin, Corbyn Salisbury

 */

#include <art_observers/QuadrilateralOps.h>
#include <art_observers/adjacent_left.h>

namespace observers
{

AdjacentLeft::AdjacentLeft(art_observers::ObserversConfig &config):
  Observer(config,
	   art_msgs::Observation::Adjacent_left,
	   std::string("Adjacent Left"))
{
  distance_filter_.configure();
  velocity_filter_.configure();
}

AdjacentLeft::~AdjacentLeft()
{
}

art_msgs::Observation
  AdjacentLeft::update(const art_msgs::ArtQuadrilateral &robot_quad,
			 const art_msgs::ArtLanes &local_map,
			 const art_msgs::ArtLanes &obstacles)
{
  observation_.distance = 0;
  observation_.velocity = 0.0;
  observation_.time = 0.0;
  observation_.clear = false;
  observation_.applicable = false;
                   
  return observation_;
}


}; // namespace observers
