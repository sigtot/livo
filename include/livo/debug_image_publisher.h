#ifndef ORB_TEST_INCLUDE_LIVO_DEBUG_IMAGE_PUBLISHER_H_
#define ORB_TEST_INCLUDE_LIVO_DEBUG_IMAGE_PUBLISHER_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// Singleton for publishing images
class DebugImagePublisher
{
private:
  ros::Publisher new_landmarks_publisher_;

  DebugImagePublisher() = default;
  static DebugImagePublisher& GetInstance();

public:
  DebugImagePublisher(DebugImagePublisher const&) = delete;
  void operator=(DebugImagePublisher const&) = delete;

  static void SetPublishers(const ros::Publisher& new_landmarks_publisher);

  static void PublishNewLandmarksImage(const cv::Mat& image, const std::vector<std::vector<cv::Point2f>>& tracks,
                                       double timestamp);
};

#endif  // ORB_TEST_INCLUDE_LIVO_DEBUG_IMAGE_PUBLISHER_H_
