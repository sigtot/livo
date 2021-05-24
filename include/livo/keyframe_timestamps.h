#ifndef ORB_TEST_SRC_KEYFRAME_TIMESTAMPS_H_
#define ORB_TEST_SRC_KEYFRAME_TIMESTAMPS_H_

#include <set>

class KeyframeTimestamps
{
private:
  std::set<double> timestamps_;
public:
  void AddKeyframeTimestamp(double timestamp);
  double GetMostRecentKeyframeTimestamp(double timestamp);
};

#endif  // ORB_TEST_SRC_KEYFRAME_TIMESTAMPS_H_
