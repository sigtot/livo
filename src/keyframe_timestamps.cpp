#include "keyframe_timestamps.h"
void KeyframeTimestamps::AddKeyframeTimestamp(double timestamp)
{
  timestamps_.insert(timestamp);
}
double KeyframeTimestamps::GetMostRecentKeyframeTimestamp(double timestamp)
{
  auto upper_bound = timestamps_.upper_bound(timestamp);
  if (upper_bound == timestamps_.begin())
  {
    return 0.0;
  }
  upper_bound--;
  return *upper_bound;
}
