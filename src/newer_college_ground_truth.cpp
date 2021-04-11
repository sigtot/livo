#include "newer_college_ground_truth.h"

#include <map>
#include <iostream>
#include <fstream>
#include <array>
#include <sstream>
#include <utility>

void NewerCollegeGroundTruth::LoadFromFile(std::multimap<double, Pose3>& gt) const
{
  std::ifstream f;
  f.open(filename_, std::ios::in);
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
    gt.insert({ ts, pose });
  }
  f.close();
}

NewerCollegeGroundTruth::NewerCollegeGroundTruth(std::string filename) : filename_(std::move(filename))
{
}
