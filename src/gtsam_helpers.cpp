#include "gtsam_helpers.h"

#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/Pose3.h>
#include <fstream>

void SaveGraphToFile(const std::string& filename, const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values)
{
  std::ofstream ofs(filename, std::ofstream::out);
  graph.saveGraph(ofs, values);
  ofs.close();
}

std::vector<std::pair<int, std::weak_ptr<Feature>>> SortFeatures(const std::map<int, std::weak_ptr<Feature>>& features)
{
  std::vector<std::pair<int, std::weak_ptr<Feature>>> sorted_features;
  for (auto& feature_pair : features)
  {
    sorted_features.emplace_back(feature_pair);
  }

  // TODO first element has no depth. why?? something wrong with the sort function?
  std::sort(sorted_features.begin(), sorted_features.end(),
            [](const std::pair<int, std::weak_ptr<Feature>>& a, const std::pair<int, std::weak_ptr<Feature>>& b) {
              auto feat_a = a.second.lock();
              auto feat_b = b.second.lock();
              if ((feat_a && feat_a->depth) && !(feat_b && feat_b->depth))
              {
                return 1;
              }
              else if (!(feat_a && feat_a->depth) && (feat_b && feat_b->depth))
              {
                return -1;
              }
              return 0;
            });
  return sorted_features;
}
