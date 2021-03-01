#ifndef ORB_TEST__GLOBAL_PARAMS_H_
#define ORB_TEST__GLOBAL_PARAMS_H_

#include <ros/ros.h>

class GlobalParams {
 private:
  GlobalParams() = default;
  static GlobalParams& GetInstance();
  template <class T>
  static void ReadVariable(const ros::NodeHandle& nh,
                           const std::string& variable_name, T& variable);

  // Add parameters here
  int max_features_ = 10;

 public:
  static void LoadParams(const ros::NodeHandle& nh);
  GlobalParams(GlobalParams const&) = delete;
  void operator=(GlobalParams const&) = delete;

  // Add parameter accessors here
  static int MaxFeatures();
};

#endif
