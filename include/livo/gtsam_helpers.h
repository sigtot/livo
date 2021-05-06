#ifndef ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
#include "feature.h"
#include "track.h"

#include <memory>
#include <string>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

void SaveGraphToFile(const std::string& filename, const gtsam::NonlinearFactorGraph& graph,
                     const gtsam::Values& values);

std::vector<std::pair<int, std::weak_ptr<Feature>>> SortFeatures(const std::map<int, std::weak_ptr<Feature>>& features);

#endif  // ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
