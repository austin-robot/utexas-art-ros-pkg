/* -*- mode: C++ -*- */
/*
 *  Description: Euclidean distance functions
 *
 *  Copyright (C) 2009 Austin Robot Technology                    
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _EUCLIDEAN_DISTANCE_H_
#define _EUCLIDEAN_DISTANCE_H_

/**  @file
   
     @brief ART Euclidean distance functions.
 */

#include <art/epsilon.h>
#include <art/infinity.h>
#include <art/Position.h>
#include <art_map/types.h>

namespace Euclidean
{

  /** find the distance from the point (cx,cy) to the line determined
   * by the points (ax,ay) and (bx,by)
   */
  void inline DistanceFromLine(float cx, float cy,
			       float ax, float ay ,
			       float bx, float by,
			       float &distanceSegment,
			       float &distanceLine)
    {
      // find the distance from the point (cx,cy) to the line
      // determined by the points (ax,ay) and (bx,by)
      //
      // distanceSegment = distance from the point to the line segment
      // distanceLine = distance from the point to the line
      //		    (assuming infinite extent in both directions)
      //
      // copied from http://www.codeguru.com/forum/printthread.php?t=194400

      float r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
      float r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
      float r = r_numerator / r_denomenator;

      float px = ax + r*(bx-ax);
      float py = ay + r*(by-ay);

      float s =  ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay) ) / r_denomenator;

      distanceLine = fabs(s)*sqrtf(r_denomenator);

      //
      // (xx,yy) is the point on the lineSegment closest to (cx,cy)
      //
      float xx = px;
      float yy = py;

      if ( (r >= 0) && (r <= 1) )
	{
	  distanceSegment = distanceLine;
	}
      else
	{

	  float dist1 = (cx-ax)*(cx-ax) + (cy-ay)*(cy-ay);
	  float dist2 = (cx-bx)*(cx-bx) + (cy-by)*(cy-by);
	  if (dist1 < dist2)
	    {
	      xx = ax;
	      yy = ay;
	      distanceSegment = sqrtf(dist1);
	    }
	  else
	    {
	      xx = bx;
	      yy = by;
	      distanceSegment = sqrtf(dist2);
	    }
	}
    }

  void inline DistanceFromLine(const MapXY& c,
			       const MapXY& a,
			       const MapXY& b,
			       float &distanceSegment,
			       float &distanceLine)
  {
    return DistanceFromLine(c.x,c.y,a.x,a.y,b.x,b.y,distanceSegment, 
			    distanceLine);
  }

  /** find the Euclidean distance between poses p1 and p2 */
  float inline DistanceTo(Position::Pose3D p1, Position::Pose3D p2)
    {
      float x_dist = p1.x - p2.x;
      float y_dist = p1.y - p2.y;
      return sqrtf(x_dist*x_dist + y_dist*y_dist);
    }

  /** find the Euclidean distance between poses p1 and p2 */
  float inline DistanceTo(float p1x, float p1y, float p2x, float p2y)
    {
      float x_dist = p1x - p2x;
      float y_dist = p1y - p2y;
      return sqrtf(x_dist*x_dist + y_dist*y_dist);
    }

  /** find the Euclidean distance between MapXY coordinates p1 and p2 */
  float inline DistanceTo(MapXY p1, MapXY p2)
    {
      MapXY dist = p1 - p2;
      return sqrtf(dist.x*dist.x + dist.y*dist.y);
    }

  /** return how many seconds it takes to move a distance at a given
   *  speed.
   */
  float inline DistanceToTime(float distance, float speed)
    {
      float abs_speed = fabs(speed);
      if (abs_speed < Epsilon::speed)
	return (Infinite::time);
      else
	return (distance / abs_speed);
    }

  /** find the Euclidean distance between pose and way-point */
  float inline DistanceToWaypt(const Position::Pose3D &pose,
			       const WayPointNode &waypt)
    {
      return DistanceTo(MapXY(pose), waypt.map);
    }

  /** find the Euclidean distance between MapXY point and way-point */
  float inline DistanceToWaypt(MapXY point, const WayPointNode &waypt)
    {
      return DistanceTo(point, waypt.map);
    }

  /** find the Euclidean distance between Polar coordinate (relative
   *  to origin) and way-point */
  float inline DistanceToWaypt(Polar polar, const Position::Pose3D &origin,
			       const WayPointNode &waypt)
    {
      return DistanceTo(Coordinates::Polar_to_MapXY(polar, origin),
			waypt.map);
    }

  /** return true if point is in the line segment between lp1 and lp2 */
  bool inline point_in_line_segment(MapXY point, MapXY lp1, MapXY lp2)
    {
      return (fabs(DistanceTo(lp1, point) + DistanceTo(point, lp2)
		   - DistanceTo(lp1, lp2))
	      < Epsilon::distance);
    }

}

#endif // _EUCLIDEAN_DISTANCE_H_ //

