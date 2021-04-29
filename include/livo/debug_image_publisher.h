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
  ros::Publisher reprojection_error_publisher_;
  ros::Publisher depth_image_publisher_;

  DebugImagePublisher() = default;
  static DebugImagePublisher& GetInstance();

public:
  DebugImagePublisher(DebugImagePublisher const&) = delete;
  void operator=(DebugImagePublisher const&) = delete;

  // TODO: Make this take a nh instead
  static void SetPublishers(const ros::Publisher& new_landmarks_publisher,
                            const ros::Publisher& reprojection_error_publisher,
                            const ros::Publisher& depth_image_publisher);

  static void PublishNewLandmarksImage(const cv::Mat& image, const std::vector<std::vector<cv::Point2f>>& tracks,
                                       double timestamp);
  static void PublishReprojectionErrorImage(const cv::Mat& image, const std::vector<cv::Point2f>& features,
                                            const std::vector<cv::Point2f>& reprojected_features,
                                            const std::vector<bool>& inlier_mask, double timestamp);
  static void PublishDepthImage(const cv::Mat& image, double timestamp);
};

#endif  // ORB_TEST_INCLUDE_LIVO_DEBUG_IMAGE_PUBLISHER_H_
