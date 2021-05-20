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

double ComputeParallaxWithOpenCV(const cv::Point2f& point1, const cv::Point2f& point2, const gtsam::Rot3& R12,
                               const boost::shared_ptr<gtsam::Cal3_S2>& K)
{
  // TODO pass directly instead of copy
  auto R12_cv = FromMatrix3(R12.matrix());
  auto K_cv = FromMatrix3(K->K());
  auto K_cv_inv = FromMatrix3(K->inverse());

  return ComputePointParallax(point1, point2, R12_cv, K_cv, K_cv_inv);
}
