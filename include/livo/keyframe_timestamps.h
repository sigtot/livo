#ifndef ORB_TEST_SRC_KEYFRAME_TIMESTAMPS_H_
#define ORB_TEST_SRC_KEYFRAME_TIMESTAMPS_H_

#include <set>
#include <boost/optional.hpp>

class KeyframeTimestamps
{
private:
  std::set<double> timestamps_;
public:
  void AddKeyframeTimestamp(double timestamp);
  boost::optional<double> GetMostRecentKeyframeTimestamp(double timestamp);
};

#endif  // ORB_TEST_SRC_KEYFRAME_TIMESTAMPS_H_
