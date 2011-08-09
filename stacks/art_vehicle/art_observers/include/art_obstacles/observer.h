#ifndef _ART_OBSTACLE_OBSERVER_H_
#define _ART_OBSTACLE_OBSERVER_H_

#include <vector>

#include <art_msgs/ArtLanes.h>
#include <art_msgs/Observation.h>
#include <art_msgs/ArtQuadrilateral.h>
#include <art_msgs/ArtLanes.h>

class Observer {
public:
  Observer();
  ~Observer();

  //virtual art_msgs::Observation update(const art_msgs::ArtLanes &local_map, std::vector<art_msgs::ArtQuadrilateral> &occupied_quads, const art_msgs::ArtQuadrilateral &robot_quad) = 0;
 
protected:
  art_msgs::Observation observation_;
};

#endif
