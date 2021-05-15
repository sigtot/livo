#ifndef ORB_TEST_INCLUDE_LIVO_DEPTH_TRIANGULATION_H_
#define ORB_TEST_INCLUDE_LIVO_DEPTH_TRIANGULATION_H_

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

namespace gtsam
{
class Pose3;
class Cal3_S2;

typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Vector3;
typedef Vector2 Point2;
typedef Vector3 Point3;

template <class T>
class PinholeBaseK;
}  // namespace gtsam

class DepthTriangulation
{
public:
  static gtsam::Point3 PixelAndDepthToPoint3(const gtsam::Point2& pt, double depth,
                                             const gtsam::PinholeBaseK<gtsam::Cal3_S2>& camera);
};

#endif  // ORB_TEST_INCLUDE_LIVO_DEPTH_TRIANGULATION_H_
