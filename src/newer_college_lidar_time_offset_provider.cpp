#include "newer_college_lidar_time_offset_provider.h"

#include <fstream>
#include <sstream>

double NewerCollegeLidarTimeOffsetProvider::GetOffset(double timestamp)
{
  auto upper_bound = time_offsets_.upper_bound(timestamp);
  if (upper_bound == time_offsets_.begin())
  {
    return 0.0;
  }
  upper_bound--;
  return upper_bound->second;
}

NewerCollegeLidarTimeOffsetProvider::NewerCollegeLidarTimeOffsetProvider(const std::string& filename)
{
  std::ifstream f;
  f.open(filename, std::ios::in);
  std::string line;
  bool past_first_line = false;  // csv file has a header, so we need to skip over it before reading data
  while (std::getline(f, line))
  {
    if (!past_first_line)
    {
      past_first_line = true;
      continue;
    }

    std::stringstream ss(line);
    int cam_secs, cam_nsecs, lidar_nsecs;
    char c;
    ss >> cam_secs >> c >> cam_nsecs >> c >> lidar_nsecs;
    double cam_ts = double(cam_secs) + double(cam_nsecs) * 1e-9;
    double lidar_offset = double(lidar_nsecs) * 1e-9;
    time_offsets_.insert({ cam_ts, lidar_offset });
  }
  f.close();
}
