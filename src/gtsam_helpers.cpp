#include "gtsam_helpers.h"
#include "gtsam_conversions.h"
#include "feature_helpers.h"

#include <gtsam/geometry/triangulation.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <fstream>
#include <opencv2/core/core.hpp>

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
  return ComputePointParallax(ToCvPoint(point1), ToCvPoint(point2), FromMatrix3(cam1_R_cam2.matrix()),
                              FromMatrix3(K->K()), FromMatrix3(K->inverse()));
}
