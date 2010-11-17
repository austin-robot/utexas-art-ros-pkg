#ifndef __QUADRILATERAL_OPS_H__
#define __QUADRILATERAL_OPS_H__

#include <art_msgs/ArtQuadrilateral.h>
#include <art_msgs/ArtLanes.h>

namespace quad_ops {
  bool pointInHull(float x, float y, const geometry_msgs::Point32 *p1, const geometry_msgs::Point32 *p2, const geometry_msgs::Point32 *p3, const geometry_msgs::Point32 *p4);  

  bool quickPointInPoly(float x, float y, const art_msgs::ArtQuadrilateral& p);

  art_msgs::ArtLanes filterLanes(const art_msgs::ArtQuadrilateral& base_quad, const art_msgs::ArtLanes& quads, bool(*filter)(const art_msgs::ArtQuadrilateral&, const art_msgs::ArtQuadrilateral&) );



  // Create a comparions operator so we can use ArtQuadrilateral's in std::set or std::sort
  struct quad_less {
    bool operator()(const art_msgs::ArtQuadrilateral& x, const art_msgs::ArtQuadrilateral& y) const {
      return x.poly_id < y.poly_id;
    };
  };

  // Create some comparison operators that we can use for filtering ArtQuadirilaterals
  inline bool compare_seg_lane(const art_msgs::ArtQuadrilateral& base, const art_msgs::ArtQuadrilateral& comp) {
    return ( (base.start_way.seg == comp.start_way.seg) && 
             (base.start_way.lane == comp.start_way.lane) );
  };

  inline bool compare_forward_seg_lane(const art_msgs::ArtQuadrilateral& base, const art_msgs::ArtQuadrilateral& comp) {
    return ( (comp.poly_id > base.poly_id) &&  
             (base.start_way.seg == comp.start_way.seg) && 
             (base.start_way.lane == comp.start_way.lane) );
  };

  inline bool compare_backward_seg_lane(const art_msgs::ArtQuadrilateral& base, const art_msgs::ArtQuadrilateral& comp) {
    return ( (base.poly_id > comp.poly_id) &&  
             (base.start_way.seg == comp.start_way.seg) && 
             (base.start_way.lane == comp.start_way.lane) );
  };
}
#endif
