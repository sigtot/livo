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
  std::vector<std::pair<int, std::weak_ptr<Feature>>> sorted_features(features.begin(), features.end());

  std::sort(
      sorted_features.begin(), sorted_features.end(),
      [](const std::pair<int, std::weak_ptr<Feature>>& a, const std::pair<int, std::weak_ptr<Feature>>& b) -> bool {
        auto a_ptr = a.second;
        auto b_ptr = b.second;
        auto feat_a = a_ptr.lock();
        auto feat_b = b_ptr.lock();
        if ((feat_a && feat_a->depth) && !(feat_b && feat_b->depth))
        {
          return true;
        }
        else if (!(feat_a && feat_a->depth) && (feat_b && feat_b->depth))
        {
          return false;
        }
        return false;  // Strict weak ordering :)
      });
  return sorted_features;
}
