/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011 Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 * 
 *  $Id$
 */

/**  @file
 
     ART observers data filtering class interfaces.

     @author Michael Quinlan

 */

#ifndef _LANE_FILTER_H_
#define _LANE_FILTER_H_

#include <filters/mean.h>
#include <filters/median.h>
#include <filters/realtime_circular_buffer.h>

class MedianFilter : public filters::MedianFilter<float> 
{
public:
  MedianFilter();
  ~MedianFilter();
  bool configure();
  bool isFull();
};

class MeanFilter : public filters::MeanFilter<float> 
{
public:
  MeanFilter();
  ~MeanFilter();
  bool configure();
  bool isFull();
};

#endif // _LANE_FILTER_H_
