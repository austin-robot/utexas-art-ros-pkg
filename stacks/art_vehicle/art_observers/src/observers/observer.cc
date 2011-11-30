/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology, Michael Quinlan
 *  License: Modified BSD Software License 
 */

/**  @file
 
     ART lane observer base class implementation.  Used as a framework to all other observers

     @author Michael Quinlan, Corbyn Salisbury

 */

#include <art_observers/observer.h>
#include <art_observers/QuadrilateralOps.h>

namespace observers
{

Observer::~Observer() {}

// \brief returns all obstacles located in wanted lane
art_msgs::ArtLanes 
  Observer::getObstaclesInLane(art_msgs::ArtLanes obstacles,
                               art_msgs::ArtLanes lane_quads) 
{
  int counter = 0;
  art_msgs::ArtLanes obstaclesInLane;
  obstaclesInLane.polygons.resize(obstacles.polygons.size());
  for(unsigned i = 0; i < obstacles.polygons.size(); i++) {
    // implementation of pointsInLane from lane_observations here on out.
    float x = obstacles.polygons[i].midpoint.x;
    float y = obstacles.polygons[i].midpoint.y;
    
    // Want to do the following in pointInLane, but for some reason can't
    size_t num_polys = lane_quads.polygons.size();

    bool inside = false;
  
    for (size_t j=0; j<num_polys; j++) {
      art_msgs::ArtQuadrilateral *p= &(lane_quads.polygons[j]);

      float dist= ((p->midpoint.x-x)*(p->midpoint.x-x)
                   + (p->midpoint.y-y)*(p->midpoint.y-y));

      if (dist > 16)         // quick check: are we near the polygon?
        continue;

      inside = quad_ops::quickPointInPolyRatio(x,y,*p,0.6);

      if(inside)
	break;
   }

    if(inside) {
      obstaclesInLane.polygons[counter] = obstacles.polygons[i];
      counter++;
    }
  }
  obstaclesInLane.polygons.resize(counter);
  return obstaclesInLane;
}

#if 0
bool pointInLane(float x, float y, art_msgs::ArtLanes lane) {
  
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
}
#endif

}; // namespace observers
