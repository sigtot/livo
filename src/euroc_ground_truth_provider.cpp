#include "euroc_ground_truth_provider.h"
#include <utility>
#include <map>
#include <iostream>
#include <fstream>
#include <array>
#include <sstream>

void EurocGroundTruthProvider::LoadFromFile(std::multimap<double, Pose3>& gt) const
{
  std::ifstream f;
  f.open(filename_, std::ios::in);
  std::string line;
  std::getline(f, line); // First line is a header. Skip it
  while (std::getline(f, line))
  {
    std::stringstream ss(line);
    long long ts;
    char c;
    ss >> ts >> c;
    double data[19]; // 7 for pose, 6 for velocity, 6 for bias = 19 total
    int i = 0;
    while ((ss >> data[i++] >> c) && (c == ','))
      ;
    // We only use the 7 first states (i.e. the pose):
    Pose3 pose{ .point = { .x = data[0], .y = data[1], .z = data[2] },
                .rot = { .x = data[4], .y = data[5], .z = data[6], .w = data[3] } }; // quat given as [w, x, y, z]
    int secs = static_cast<int>(ts / 1000000000);
    int nsecs = static_cast<int>(ts % 1000000000);
    double ts_double = static_cast<double>(secs) + static_cast<double>(nsecs) * 1e-9;
    gt.insert({ ts_double, pose });
  }
  f.close();
}

EurocGroundTruthProvider::EurocGroundTruthProvider(std::string filename) : filename_(std::move(filename))
{
}
