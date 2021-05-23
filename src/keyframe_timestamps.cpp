#include "keyframe_timestamps.h"
void KeyframeTimestamps::AddKeyframeTimestamp(double timestamp)
{
  timestamps_.insert(timestamp);
}
boost::optional<double> KeyframeTimestamps::GetMostRecentKeyframeTimestamp(double timestamp)
{
  auto lower_bound = timestamps_.lower_bound(timestamp);
  if (lower_bound == timestamps_.begin())
  {
    return boost::none;
  }
  lower_bound--;
  return *lower_bound;
}
