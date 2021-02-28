#ifndef ORB_TEST__GLOBAL_PARAMS_H_
#define ORB_TEST__GLOBAL_PARAMS_H_

#include <ros/ros.h>

class GlobalParams {
 private:
  GlobalParams() = default;
  static GlobalParams& getInstance() {
    static GlobalParams instance;
    return instance;
  }

  // Add parameters here
  int max_features_ = 200;

 public:
  static void loadParams(const ros::NodeHandle& nh);
  GlobalParams(GlobalParams const&) = delete;
  void operator=(GlobalParams const&) = delete;

  // Add parameter accessors here
  static int MaxFeatures();
};

#endif
