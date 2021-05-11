#ifndef ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_

#include <string>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

void SaveGraphToFile(const std::string& filename, const gtsam::NonlinearFactorGraph& graph,
                     const gtsam::Values& values);

#endif  // ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
