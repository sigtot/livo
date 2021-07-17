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
public:
  static gtsam::Point3 PixelAndDepthToPoint3(const gtsam::Point2& pt, double depth,
                                             const gtsam::PinholeBaseK<gtsam::Cal3_S2>& camera);
  static gtsam::TriangulationResult Triangulate(const std::vector<gtsam::Point2>& measurements,
                                                const std::vector<gtsam::PinholeCamera<gtsam::Cal3_S2>>& cameras,
                                                const gtsam::TriangulationParameters& params);
  static bool TriangulationIsOk(const gtsam::Point3& point, const std::vector<gtsam::Point2>& measurements,
                                const std::vector<gtsam::PinholeCamera<gtsam::Cal3_S2>>& cameras,
                                double mean_reproj_thresh);
};

#endif  // ORB_TEST_INCLUDE_LIVO_DEPTH_TRIANGULATION_H_
