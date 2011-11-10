/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology, Michael Quinlan
 *  License: Modified BSD Software License 
 */

/**  @file
 
     ART lane observer base class implementation.

     @author Michael Quinlan

 */

#include <art_observers/observer.h>
#include <art_observers/QuadrilateralOps.h>

namespace observers
{

Observer::~Observer() {}

// Define stub for non-pure virtual function method to avoid linker
// error.  Can be removed once it is made pure virtual.
art_msgs::Observation
  Observer::update(const art_msgs::ArtQuadrilateral &robot_quad,
		   const art_msgs::ArtLanes &local_map,
		   const art_msgs::ArtLanes &obstacles,
		   MapPose pose_)
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

art_msgs::ArtLanes Observer::getObstaclesInLane(art_msgs::ArtLanes obstacles, art_msgs::ArtLanes lane_quads) {
  int counter = 0;
  art_msgs::ArtLanes obstaclesInLane;
  obstaclesInLane.polygons.resize(obstacles.polygons.size());
  for(unsigned i = 0; i < obstacles.polygons.size(); i++) {
    float x = obstacles.polygons[i].midpoint.x;
    float y = obstacles.polygons[i].midpoint.y;
    
    // Want to do the following in pointInLane, but for some reason can't
    size_t num_polys = lane_quads.polygons.size();

    bool inside = false;
  
    for (size_t i=0; i<num_polys; i++) {
      art_msgs::ArtQuadrilateral *p= &(lane_quads.polygons[i]);
      float dist= ((p->midpoint.x-x)*(p->midpoint.x-x)
                   + (p->midpoint.y-y)*(p->midpoint.y-y));

      if (dist > 16)         // quick check: are we near the polygon?
        continue;

      inside = quad_ops::quickPointInPolyRatio(x,y,*p,0.6);

      if(inside)
	break;
   }

    if(inside) {
      obstaclesInLane.polygons.push_back(obstacles.polygons[i]);
      counter++;
    }
  }
  obstaclesInLane.polygons.resize(counter);
  return obstaclesInLane;
}

/*bool pointInLane(float x, float y, art_msgs::ArtLanes lane) {
  
  size_t num_polys = lane.polygons.size();
  
  bool inside = false;
  
  for (size_t i=0; i<num_polys; i++)
    {
      art_msgs::ArtQuadrilateral *p= &(lane.polygons[i]);
      float dist= ((p->midpoint.x-x)*(p->midpoint.x-x)
                   + (p->midpoint.y-y)*(p->midpoint.y-y));

      if (dist > 16)         // quick check: are we near the polygon?
        continue;

      inside = quad_ops::quickPointInPolyRatio(x,y,*p,0.6);

      if(inside)
	break;
    }

  return inside;
}*/

}; // namespace observers
