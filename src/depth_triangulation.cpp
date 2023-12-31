#include "depth_triangulation.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/CameraSet.h>

gtsam::Point3 DepthTriangulation::PixelAndDepthToPoint3InCamFrame(const gtsam::Point2& pt, double depth,
                                                                  const gtsam::PinholeBaseK<gtsam::Cal3_S2>& camera)
{
  // We use Camera::backprojectPointAtInfinity and scale the result rather than Camera::backproject.
  // This is because backproject projects points onto a plane that is {depth} away from the camera,
  // whereas we want the hypotenuse to be {depth} long, something that only holds for the center pixel with backproject.
  auto back_projected_inf = camera.backprojectPointAtInfinity(pt);
  gtsam::Point3 cam_point = depth * back_projected_inf.point3();
  return cam_point;
}

gtsam::Point3 DepthTriangulation::PixelAndDepthToPoint3(const gtsam::Point2& pt, double depth,
                                                        const gtsam::PinholeBaseK<gtsam::Cal3_S2>& camera)
{
  auto cam_point = DepthTriangulation::PixelAndDepthToPoint3InCamFrame(pt, depth, camera);
  gtsam::Point3 world_point = camera.translation() + cam_point;
  return world_point;
}

double DepthTriangulation::GetDepthInBodyFrame(const gtsam::Point2& pt, double cam_depth,
                                               const gtsam::Cal3_S2& K,
                                               const gtsam::Pose3& body_p_cam)
{
  // Use a camera centered at the origin, but with the body_p_cam
  gtsam::PinholeCamera<gtsam::Cal3_S2> camera(body_p_cam, K);
  auto cam_point = DepthTriangulation::PixelAndDepthToPoint3InCamFrame(pt, cam_depth, camera);
  return body_p_cam.inverse().range(cam_point);
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

bool DepthTriangulation::ReprojectionErrorIsOk(const gtsam::Point3& point,
                                               const std::vector<gtsam::Point2>& measurements,
                                               const std::vector<gtsam::PinholeCamera<gtsam::Cal3_S2>>& cameras,
                                               double mean_reproj_thresh)
{
  double reproj_mean = 0;
  std::cout << "Reproj error: ";
  for (int i = 0; i < measurements.size(); ++i)
  {
    auto reproj_error = (cameras[i].project(point) - measurements[i]).norm();
    std::cout << reproj_error << ", " << std::endl;
    reproj_mean += reproj_error;
  }
  reproj_mean /= static_cast<double>(measurements.size());
  std::cout << std::endl;
  std::cout << "Mean: " << reproj_mean << " (vs thresh " << mean_reproj_thresh << ")" << std::endl;
  return reproj_mean < mean_reproj_thresh;
}

bool DepthTriangulation::LandmarkDistanceIsOk(const gtsam::Point3& point,
                                              const gtsam::PinholeCamera<gtsam::Cal3_S2>& last_camera, double max_dist)
{
  double dist = last_camera.range(point);
  std::cout << "Lmk distance to last camera is " << dist << " (max " << max_dist << ")" << std::endl;
  return dist < max_dist;
}
