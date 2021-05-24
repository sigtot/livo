#include <gtest/gtest.h>

#include "keyframe_timestamps.h"

TEST(KeyframeTimestamps, GetsMostRecentKeyframeTimestamp)
{
  KeyframeTimestamps timestamps;
  timestamps.AddKeyframeTimestamp(4.0);
  timestamps.AddKeyframeTimestamp(6.0);

  EXPECT_DOUBLE_EQ(timestamps.GetMostRecentKeyframeTimestamp(5.0), 4.0);
}

TEST(KeyframeTimestamps, SameTimestampGivesItself)
{
  KeyframeTimestamps timestamps;
  timestamps.AddKeyframeTimestamp(4.0);

  EXPECT_DOUBLE_EQ(timestamps.GetMostRecentKeyframeTimestamp(4.0), 4.0);
}

TEST(KeyframeTimestamps, GetsLastElementWhenPastEnd)
{
  KeyframeTimestamps timestamps;
  timestamps.AddKeyframeTimestamp(4.0);
  timestamps.AddKeyframeTimestamp(6.0);

  EXPECT_DOUBLE_EQ(timestamps.GetMostRecentKeyframeTimestamp(7.0), 6.0);
}

TEST(KeyframeTimestamps, GetsNoneWhenBeforeStart)
{
  KeyframeTimestamps timestamps;
  timestamps.AddKeyframeTimestamp(4.0);
  timestamps.AddKeyframeTimestamp(6.0);

  EXPECT_DOUBLE_EQ(timestamps.GetMostRecentKeyframeTimestamp(3.0), 0.0);
}

TEST(KeyframeTimestamps, GetsOnlyElementIfAfter)
{
  KeyframeTimestamps timestamps;
  timestamps.AddKeyframeTimestamp(4.0);

  EXPECT_DOUBLE_EQ(timestamps.GetMostRecentKeyframeTimestamp(5.0), 4.0);
}

TEST(KeyframeTimestamps, DoesNotGetOnlyElementIfBefore)
{
  KeyframeTimestamps timestamps;
  timestamps.AddKeyframeTimestamp(4.0);

  EXPECT_DOUBLE_EQ(timestamps.GetMostRecentKeyframeTimestamp(3.0), 0.0);
}

TEST(KeyframeTimestamps, WhenEmptyReturnsNone)
{
  KeyframeTimestamps timestamps;

  EXPECT_DOUBLE_EQ(timestamps.GetMostRecentKeyframeTimestamp(3.0), 0.0);
}
