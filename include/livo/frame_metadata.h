#ifndef ORB_TEST_INCLUDE_LIVO_FRAME_METADATA_H_
#define ORB_TEST_INCLUDE_LIVO_FRAME_METADATA_H_

struct FrameMetadata
{
  int frame_id;
  int n_landmarks;
  bool loam_degenerate;
  int n_ransac_outliers;
};

#endif  // ORB_TEST_INCLUDE_LIVO_FRAME_METADATA_H_
