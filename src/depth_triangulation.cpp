#include "depth_triangulation.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>

gtsam::Point3 DepthTriangulation::PixelAndDepthToPoint3(const gtsam::Point2& pt, double depth,
                                                        const boost::shared_ptr<gtsam::Cal3_S2>& K,
                                                        const gtsam::Pose3& body_p_sensor,
                                                        const gtsam::Pose3& world_p_body)
{
  return gtsam::Point3();
}
