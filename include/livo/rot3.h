#ifndef ORB_TEST_INCLUDE_LIVO_ROT3_H_
#define ORB_TEST_INCLUDE_LIVO_ROT3_H_

struct Rot3
{
  double x;
  double y;
  double z;
  double w;

  static Rot3 Eye()
  {
    return Rot3{ .x = 0, .y = 0, .z = 0, .w = 1 };
  }
};

#endif  // ORB_TEST_INCLUDE_LIVO_ROT3_H_
