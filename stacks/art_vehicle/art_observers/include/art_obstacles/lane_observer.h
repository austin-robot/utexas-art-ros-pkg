#ifndef _LANE_OBSERVER_H_
#define _LANE_OBSERVER_H_

#include <vector>

#include <filters/realtime_circular_buffer.h>

#include <art_msgs/ArtLanes.h>
#include <art_msgs/Observation.h>


#include <art_obstacles/observer.h>
#include <art_obstacles/filter.h>


class LaneObserver: public Observer {
public:
  LaneObserver();
  ~LaneObserver();

  art_msgs::Observation update(int root_poly_id, art_msgs::ArtLanes &lane_quads, art_msgs::ArtLanes &obstacle_quads);

private:
  std::vector<float> distance_;

  MedianFilter distance_filter_;
  MeanFilter velocity_filter_;

  ros::Time prev_update_;
  ros::Time current_update_;
};


#endif

