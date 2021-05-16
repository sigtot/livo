#ifndef ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_

#include <string>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2Result.h>

void SaveGraphToFile(const std::string& filename, const gtsam::NonlinearFactorGraph& graph,
                     const gtsam::Values& values);
void PrintVariableStatus(const gtsam::ISAM2Result::DetailedResults::VariableStatus& variable_status);

#endif  // ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
