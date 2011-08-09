#include <filters/realtime_circular_buffer.h>
#include <art_obstacles/filter.h>

MedianFilter::MedianFilter() {
  number_of_observations_ = 5;
}

MedianFilter::~MedianFilter() {
}

bool MedianFilter::configure() {
  // Hack to overwrite configure for the median filter
  configured_=true; 
  data_storage_.reset( new filters::RealtimeCircularBuffer<float >(number_of_observations_, temp));
  temp_storage_.resize(number_of_observations_);
  return true;
};

bool MedianFilter::isFull() {
  if (data_storage_->size() == number_of_observations_)
    return true;
  else
    return false;
}

// -------------------------

MeanFilter::MeanFilter() {
  number_of_observations_ = 5;
}

MeanFilter::~MeanFilter() {
}

bool MeanFilter::configure() {
  // Hack to overwrite configure for the mean filter
  configured_=true; 
  data_storage_.reset(new filters::RealtimeCircularBuffer<float>(number_of_observations_, temp_));
  return true;
};

bool MeanFilter::isFull() {
  if (data_storage_->size() == number_of_observations_)
    return true;
  else
    return false;
}
