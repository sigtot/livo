#ifndef ORB_TEST_INCLUDE_LIVO_TIME_OFFSET_PROVIDER_H_
#define ORB_TEST_INCLUDE_LIVO_TIME_OFFSET_PROVIDER_H_

class TimeOffsetProvider
{
public:
  virtual double GetOffset(double timestamp) = 0;
};

#endif  // ORB_TEST_INCLUDE_LIVO_TIME_OFFSET_PROVIDER_H_
