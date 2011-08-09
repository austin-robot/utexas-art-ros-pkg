#ifndef _LANE_FILTER_H_
#define _LANE_FILTER_H_

#include <filters/median.h>
#include <filters/mean.h>

class MedianFilter : public filters::MedianFilter<float> {
public:
  MedianFilter();
  ~MedianFilter();

  bool configure();

  bool isFull();
};

class MeanFilter : public filters::MeanFilter<float> {
public:
  MeanFilter();
  ~MeanFilter();

  bool configure();

  bool isFull();
};

#endif
