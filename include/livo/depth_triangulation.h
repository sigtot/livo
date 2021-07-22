#ifndef ORB_TEST_INCLUDE_LIVO_DEPTH_TRIANGULATION_H_
#define ORB_TEST_INCLUDE_LIVO_DEPTH_TRIANGULATION_H_

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <vector>

namespace gtsam
{
class Pose3;
class Cal3_S2;

struct TriangulationParameters;
class TriangulationResult;

typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;
typedef Vector2 Point2;
typedef Vector3 Point3;

template <class T>
class PinholeBaseK;

template <class T>
class PinholeCamera;
}  // namespace gtsam

class DepthTriangulation
{
private:
  static gtsam::Point3 PixelAndDepthToPoint3InCamFrame(const gtsam::Point2& pt, double depth,
                                                       const gtsam::PinholeBaseK<gtsam::Cal3_S2>& camera);

public:
  /**
   * Get a Point3 from pixel and depth
   * @param pt pixel point
   * @param depth in camera frame
   * @param camera
   * @return The Point3 in the world frame
   */
  static gtsam::Point3 PixelAndDepthToPoint3(const gtsam::Point2& pt, double depth,
                                             const gtsam::PinholeBaseK<gtsam::Cal3_S2>& camera);

  static double GetDepthInBodyFrame(const gtsam::Point2& pt, double cam_depth, const gtsam::Cal3_S2& K,
                                    const gtsam::Pose3& body_p_cam);

  static gtsam::TriangulationResult Triangulate(const std::vector<gtsam::Point2>& measurements,
                                                const std::vector<gtsam::PinholeCamera<gtsam::Cal3_S2>>& cameras,
                                                const gtsam::TriangulationParameters& params);

  static bool ReprojectionErrorIsOk(const gtsam::Point3& point, const std::vector<gtsam::Point2>& measurements,
                                    const std::vector<gtsam::PinholeCamera<gtsam::Cal3_S2>>& cameras,
                                    double mean_reproj_thresh);

  static bool LandmarkDistanceIsOk(const gtsam::Point3& point, const gtsam::PinholeCamera<gtsam::Cal3_S2>& last_camera,
                                   double max_dist);
};

#endif  // ORB_TEST_INCLUDE_LIVO_DEPTH_TRIANGULATION_H_
