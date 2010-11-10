
#include <art_map/QuadrilateralOps.h>

#include <art_map/types.h>

namespace QuadOps {
// determines if point lies in interior of given polygon points on
  // edge segments are considered interior points
  bool pointInHull(float x, float y, const geometry_msgs::Point32 *p1, const geometry_msgs::Point32 *p2, const geometry_msgs::Point32 *p3, const geometry_msgs::Point32 *p4) {
    float minx=p1->x;
    float maxx=p1->x;
    float miny=p1->y;
    float maxy=p1->y;
    
    minx=fminf(fminf(fminf(minx,p2->x),p3->x),p4->x);
    miny=fminf(fminf(fminf(miny,p2->y),p3->y),p4->y);
    maxx=fmaxf(fmaxf(fmaxf(maxx,p2->x),p3->x),p4->x);
    maxy=fmaxf(fmaxf(fmaxf(maxy,p2->y),p3->y),p4->y);

    return (Epsilon::gte(x,minx) && Epsilon::lte(x,maxx) &&
	    Epsilon::gte(y,miny) && Epsilon::lte(y,maxy));
  }

  bool quickPointInPoly(float x, float y, const art_msgs::ArtQuadrilateral& p) {
    const geometry_msgs::Point32 *p1 = &p.poly.points[0];
    const geometry_msgs::Point32 *p2 = &p.poly.points[1];
    const geometry_msgs::Point32 *p3 = &p.poly.points[2];
    const geometry_msgs::Point32 *p4 = &p.poly.points[3];

    if (!pointInHull(x,y,p1,p2,p3,p4))
      return false;
    
    bool odd = false;
    
    // this is an unrolled version of the standard point-in-polygon algorithm

    if ((p1->y < y && p2->y >= y) || (p2->y < y && p1->y >= y))
      if (p1->x + (y-p1->y)/(p2->y-p1->y)*(p2->x-p1->x) < x)
        odd = !odd;
    
    if ((p2->y < y && p3->y >= y) || (p3->y < y && p2->y >= y))
      if (p2->x + (y-p2->y)/(p3->y-p2->y)*(p3->x-p2->x) < x)
        odd = !odd;
    
    if ((p3->y < y && p4->y >= y) || (p4->y < y && p3->y >= y))
      if (p3->x + (y-p3->y)/(p4->y-p3->y)*(p4->x-p3->x) < x)
        odd = !odd;
    
    if ((p4->y < y && p1->y >= y) || (p1->y < y && p4->y >= y))
      if (p4->x + (y-p4->y)/(p1->y-p4->y)*(p1->x-p4->x) < x)
        odd = !odd;
    
    if (odd)
      return true;

    return false;
    //return pointOnEdges(x, y, p);
  }
}
