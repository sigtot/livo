#include "depth_triangulation.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/CameraSet.h>

gtsam::Point3 DepthTriangulation::PixelAndDepthToPoint3(const gtsam::Point2& pt, double depth,
                                                        const gtsam::PinholeBaseK<gtsam::Cal3_S2>& camera)
{
  // We use Camera::backprojectPointAtInfinity and scale the result rather than Camera::backproject.
  // This is because backproject projects points onto a plane that is {depth} away from the camera,
  // whereas we want the hypotenuse to be {depth} long, something that only holds for the center pixel with backproject.
  auto back_projected_inf = camera.backprojectPointAtInfinity(pt);
  gtsam::Point3 d_cam_point = depth * back_projected_inf.point3();
  gtsam::Point3 estimated_point = camera.translation() + d_cam_point;
  return estimated_point;
}

gtsam::TriangulationResult DepthTriangulation::Triangulate(
    const std::vector<gtsam::Point2>& measurements, const std::vector<gtsam::PinholeCamera<gtsam::Cal3_S2>>& cameras,
    const gtsam::TriangulationParameters& params)
{
  gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>> camera_set;
  camera_set.reserve(cameras.size());
  camera_set.insert(camera_set.end(), cameras.begin(), cameras.end());

  gtsam::PinholeCamera<gtsam::Cal3_S2>::MeasurementVector measurement_vector;
  measurement_vector.reserve(measurements.size());
  measurement_vector.insert(measurement_vector.end(), measurements.begin(), measurements.end());

  return gtsam::triangulateSafe(camera_set, measurement_vector, params);
}
