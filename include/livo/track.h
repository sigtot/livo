#ifndef ORB_TEST_SRC_TRACK_H_
#define ORB_TEST_SRC_TRACK_H_

#include "feature.h"

#include <utility>
#include <deque>

struct Track
{
  std::deque<std::shared_ptr<Feature>> features;
  std::vector<double> parallaxes;
  boost::optional<cv::Point2f> last_parallax;
  boost::optional<cv::Point2f> last_landmark_projection;
  int id;

  int inlier_count = 0;
  int outlier_count = 0;

  double max_parallax = 0.;

  explicit Track(std::shared_ptr<Feature> feature);

  double InlierRatio() const;

  bool HasDepth() const;

  boost::optional<LidarDepthResult> LastDepth() const;

  size_t DepthFeatureCount() const;

  void AddFeature(std::shared_ptr<Feature> feature);

  /**
   * @brief Median filters and then integrates the parallaxes to produce a smoothened sum of the parallaxes
   * @return
   */
  double IntegratedMedianFilteredParallaxes();

  /**
   * @brief Finds the median parallax of the feature
   * @return
   */
  double MedianParallax() const;

  /**
   * @brief A naive, raw pixel difference parallax computation computation method
   * @return
   */
  double ParallaxNaive() const;
};

#endif  // ORB_TEST_SRC_TRACK_H_
