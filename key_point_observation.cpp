#include "key_point_observation.h"

#include <utility>

KeyPointObservation::KeyPointObservation(cv::KeyPoint keypoint,
                                         cv::Mat descriptor,
                                         std::weak_ptr<Frame> frame)
    : keypoint(std::move(keypoint)),
      descriptor(std::move(descriptor)),
      frame(std::move(frame)) {}
