#include "gtsam_helpers.h"
#include "gtsam_conversions.h"
#include "feature_helpers.h"

#include <gtsam/geometry/triangulation.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <cmath>

void SaveGraphToFile(const std::string& filename, const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values)
{
  std::ofstream ofs(filename, std::ofstream::out);
  graph.saveGraph(ofs, values);
  ofs.close();
}

void PrintVariableStatus(const gtsam::ISAM2Result::DetailedResults::VariableStatus& variable_status)
{
  std::cout << "isReeliminated: " << variable_status.isReeliminated << std::endl;
  std::cout << "isAboveRelinThreshold: " << variable_status.isAboveRelinThreshold << std::endl;
  std::cout << "isRelinearizeInvolved: " << variable_status.isRelinearizeInvolved << std::endl;
  std::cout << "isRelinearized: " << variable_status.isRelinearized << std::endl;
  std::cout << "isObserved: " << variable_status.isObserved << std::endl;
  std::cout << "isNew: " << variable_status.isNew << std::endl;
  std::cout << "inRootClique: " << variable_status.inRootClique << std::endl;
}

double ComputeParallaxWithOpenCV(const gtsam::Point2& point1, const gtsam::Point2& point2,
                                 const gtsam::Rot3& body1_R_body2, const boost::shared_ptr<gtsam::Cal3_S2>& K,
                                 const gtsam::Rot3& body_R_cam)
{
  auto cam1_R_cam2 = body_R_cam.inverse() * body1_R_body2 * body_R_cam;
  cv::Point2f _;
  return ComputePointParallax(ToCvPoint(point1), ToCvPoint(point2), FromMatrix3(cam1_R_cam2.matrix()),
                              FromMatrix3(K->K()), FromMatrix3(K->inverse()), _);
}

double ComputeParallaxAngle(const gtsam::Point2& point1, const gtsam::Point2& point2, const gtsam::Pose3& pose1,
                            const gtsam::Pose3& pose2, const boost::shared_ptr<gtsam::Cal3_S2>& K,
                            const gtsam::Pose3& body_p_cam)
{
  auto camera1 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose1 * body_p_cam, *K);
  auto camera2 = gtsam::PinholeCamera<gtsam::Cal3_S2>(pose2 * body_p_cam, *K);

  auto c1_unit1 = camera1.backprojectPointAtInfinity(point1);
  auto c2_unit2 = camera2.backprojectPointAtInfinity(point2);

  auto body_R_cam = body_p_cam.rotation();
  auto c1_R_c2 = body_R_cam.inverse() * pose1.between(pose2).rotation() * body_R_cam;

  auto c1_unit2 = c1_R_c2 * c2_unit2;

  auto dot = c1_unit1.dot(c2_unit2);
  return std::acos(dot) * 180 / M_PI;
}
