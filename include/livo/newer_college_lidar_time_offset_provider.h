#ifndef ORB_TEST_SRC_NEWER_COLLEGE_LIDAR_TIME_OFFSET_PROVIDER_H_
#define ORB_TEST_SRC_NEWER_COLLEGE_LIDAR_TIME_OFFSET_PROVIDER_H_

#include "time_offset_provider.h"

#include <string>
#include <map>

class NewerCollegeLidarTimeOffsetProvider : public TimeOffsetProvider
{
private:
  std::map<double, double> time_offsets_;
public:
  NewerCollegeLidarTimeOffsetProvider(const std::string& filename);
  double GetOffset(double timestamp) override;
};

#endif  // ORB_TEST_SRC_NEWER_COLLEGE_LIDAR_TIME_OFFSET_PROVIDER_H_
