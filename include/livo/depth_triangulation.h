#ifndef ORB_TEST_INCLUDE_LIVO_DEPTH_TRIANGULATION_H_
#define ORB_TEST_INCLUDE_LIVO_DEPTH_TRIANGULATION_H_

#include <boost/shared_ptr.hpp>

namespace gtsam
{
class Point3;
class Pose3;
class Point2;
class Cal3_S2;

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
