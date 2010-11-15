#ifndef __QUADRILATERALOPS_H__
#define __QUADRILATERALOPS_H__

#include <art_msgs/ArtQuadrilateral.h>

namespace QuadOps {
  bool pointInHull(float x, float y, const geometry_msgs::Point32 *p1, const geometry_msgs::Point32 *p2, const geometry_msgs::Point32 *p3, const geometry_msgs::Point32 *p4);  

  bool quickPointInPoly(float x, float y, const art_msgs::ArtQuadrilateral& p);

  // Create a comparions operator so we can use ArtQuadrilateral's in stl::sets
  struct quad_compare {
    bool operator()(const art_msgs::ArtQuadrilateral& x, const art_msgs::ArtQuadrilateral& y) const {
      return x.poly_id < y.poly_id;
    }
  };

}
#endif
