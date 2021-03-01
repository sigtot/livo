#include "key_point_observation.h"

#include <utility>

KeyPointObservation::KeyPointObservation(cv::KeyPoint keypoint,
                                         cv::Mat descriptor)
    : keypoint(std::move(keypoint)), descriptor(std::move(descriptor)) {}
