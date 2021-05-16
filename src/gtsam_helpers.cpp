#include "gtsam_helpers.h"

#include <gtsam/geometry/triangulation.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <fstream>

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
