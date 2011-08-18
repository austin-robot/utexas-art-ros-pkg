/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART observers Quadrilateral interfaces.

     @todo Consider moving this to art_map.

     @author Michael Quinlan

 */

#ifndef __QUADRILATERAL_OPS_H__
#define __QUADRILATERAL_OPS_H__

#include <art_msgs/ArtQuadrilateral.h>
#include <art_msgs/ArtLanes.h>
#include <art_map/PolyOps.h>
#include <nav_msgs/Odometry.h>

namespace quad_ops 
{
  /// shorter type name for ART quadrilateral message
  typedef art_msgs::ArtQuadrilateral Quad;

  bool pointInHull(float x, float y,
                   const geometry_msgs::Point32 *p1,
                   const geometry_msgs::Point32 *p2,
                   const geometry_msgs::Point32 *p3,
                   const geometry_msgs::Point32 *p4);  

  bool quickPointInPoly(float x, float y, const Quad& p);
  
  bool quickPointInPolyRatio(float x, float y, const Quad& p, float ratio);

  art_msgs::ArtLanes filterLanes(const Quad& base_quad,
                                 const art_msgs::ArtLanes& quads,
                                 bool(*filter)(const Quad&, const Quad&));
  art_msgs::ArtLanes filterAdjacentLanes(MapPose &pose,
                                 const art_msgs::ArtLanes& quads,
                                 const int lane);
  // Create a comparison operator so we can use ArtQuadrilateral's in
  // std::set or std::sort
  struct quad_less
  {
    bool operator()(const Quad& x, const Quad& y) const
    {
      return x.poly_id < y.poly_id;
    };
  };

  // Create some comparison operators for filtering ArtQuadrilaterals
  inline bool compare_seg_lane(const Quad& base, const Quad& comp)
  {
    return ( (base.start_way.seg == comp.start_way.seg) && 
             (base.start_way.lane == comp.start_way.lane) &&
             (base.end_way.seg == comp.end_way.seg) && 
             (base.end_way.lane == comp.end_way.lane) &&
             (!comp.is_transition) );
  };

  inline bool compare_forward_seg_lane(const Quad& base, const Quad& comp)
  {
    return ( (comp.poly_id > base.poly_id) &&  
             (base.start_way.seg == comp.start_way.seg) && 
             (base.start_way.lane == comp.start_way.lane) &&
             (base.end_way.seg == comp.end_way.seg) && 
             (base.end_way.lane == comp.end_way.lane) &&
             (!comp.is_transition) );
  };

  inline bool compare_backward_seg_lane(const Quad& base, const Quad& comp)
  {
    return ( (base.poly_id > comp.poly_id) && 
             (base.start_way.seg == comp.start_way.seg) && 
             (base.start_way.lane == comp.start_way.lane) &&
             (base.end_way.seg == comp.end_way.seg) && 
             (base.end_way.lane == comp.end_way.lane) &&
             (!comp.is_transition) );
  };
}

#endif // __QUADRILATERAL_OPS_H__
