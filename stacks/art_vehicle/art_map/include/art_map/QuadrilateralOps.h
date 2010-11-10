#ifndef __QUADRILATERALOPS_H__
#define __QUADRILATERALOPS_H__

#include <art_msgs/ArtQuadrilateral.h>

namespace QuadOps {
// determines if point lies in interior of given polygon points on
  // edge segments are considered interior points
  bool pointInHull(float x, float y, const geometry_msgs::Point32 *p1, const geometry_msgs::Point32 *p2, const geometry_msgs::Point32 *p3, const geometry_msgs::Point32 *p4);  

  bool quickPointInPoly(float x, float y, const art_msgs::ArtQuadrilateral& p);
}
#endif
