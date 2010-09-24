/* -*- mode: C++ -*- */

// Filtered MapLanes
// Started - 1st August 2007
// Michael Quinlan
// $Id$

#include <art_msgs/ArtQuadrilateral.h>
#include <art_map/KF.h>
#include <art_map/Matrix.h>
#include <art_map/PolyOps.h>

#define NUM_POINTS 4

class FilteredPolygon 
{
 public:
  FilteredPolygon();
  ~FilteredPolygon() {};

  void SetPoint(int pointID, float x, float y);
  void UpdatePoint(int pointID, float visionDistance, float visionAngle,
                   float confidence,float rx, float ry, float rori);
  Matrix GetDistanceJacobian(float xb, float yb, float x, float y);
  Matrix GetAngleJacobian(float xb, float yb, float x, float y);
 
  KF point[NUM_POINTS];
  KFStruct distStruct;
  KFStruct angleStruct;

  void SetPolygon(poly p);
  poly GetPolygon();
  art_msgs::ArtQuadrilateral GetQuad();

 private:
  poly polygon_;
  PolyOps ops_;
};
