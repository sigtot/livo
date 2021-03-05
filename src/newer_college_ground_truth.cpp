#include "newer_college_ground_truth.h"
#include <map>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <array>

double THRESH = 0.4;

Pose3 NewerCollegeGroundTruth::At(double timestamp)
{
  // Logarithmic in size of container. Code is a bit messy. Should clean it up.
  auto gt_it = GetInstance().gt_.upper_bound(timestamp);
  if (gt_it != GetInstance().gt_.end())
  {
    --gt_it;
    if (gt_it != GetInstance().gt_.end() && std::abs(gt_it->first - timestamp) < THRESH)
    {
      return gt_it->second;
    }
  }

  std::cout << "Did not find any ground truth pose with timestamp close to " << std::setprecision(20) << timestamp
            << std::endl;
  exit(1);
}

std::multimap<double, Pose3> NewerCollegeGroundTruth::GetAllPoses()
{
  return GetInstance().gt_;
}

NewerCollegeGroundTruth& NewerCollegeGroundTruth::GetInstance()
{
  static NewerCollegeGroundTruth instance;
  return instance;
}

void NewerCollegeGroundTruth::LoadFromFile(const std::string& filename)
{
  std::ifstream f;
  f.open(filename, std::ios::in);
  std::string line;
  bool past_first_line = false;
  while (std::getline(f, line))
  {
    if (!past_first_line)
    {
      past_first_line = true;
      continue;
    }
    std::stringstream ss(line);
    int secs, nsecs;
    double data[7];
    char c;
    ss >> secs >> c >> nsecs >> c;
    int i = 0;
    while ((ss >> data[i++] >> c) && (c == ','))
      ;
    Pose3 pose{ .point = { .x = data[0], .y = data[1], .z = data[2] },
                .rot = { .x = data[3], .y = data[4], .z = data[5], .w = data[6] } };
    double ts = double(secs) + double(nsecs) * 1e-9;
    GetInstance().gt_.insert({ ts, pose });
  }
  f.close();
}
