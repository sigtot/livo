#ifndef ORB_TEST_SRC_ZERO_TIME_OFFSET_PROVIDER_H_
#define ORB_TEST_SRC_ZERO_TIME_OFFSET_PROVIDER_H_

#include "time_offset_provider.h"

/**
 * TimeOffsetProvider that always returns zero. Use when there is no time offset.
 */
class ZeroTimeOffsetProvider : public TimeOffsetProvider
{
  double GetOffset(double timestamp) override;
};

#endif  // ORB_TEST_SRC_ZERO_TIME_OFFSET_PROVIDER_H_
