#include "debug_image_publisher.h"

DebugImagePublisher& DebugImagePublisher::GetInstance()
{
  static DebugImagePublisher instance;
  return instance;
}

void DebugImagePublisher::SetPublishers(const ros::Publisher& new_landmarks_publisher)
{
  GetInstance().new_landmarks_publisher_ = new_landmarks_publisher;
}

void DebugImagePublisher::PublishNewLandmarksImage(const cv::Mat& image,
                                                   const std::vector<std::vector<cv::Point2f>>& tracks,
                                                   double timestamp)
{
  cv_bridge::CvImage out_img;
  out_img.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  out_img.header.stamp = ros::Time(timestamp);
  cvtColor(image, out_img.image, CV_GRAY2RGB);

  auto color = cv::Scalar(0, 255, 0);
  for (const auto& track : tracks)
  {
    for (int i = 1; i < track.size(); ++i)
    {
      cv::line(out_img.image, track[i - 1], track[i], color, 1);
    }
    if (!track.empty())
    {
      cv::circle(out_img.image, track[0], 5, color, 1);
    }
  }

  GetInstance().new_landmarks_publisher_.publish(out_img.toImageMsg());
}
