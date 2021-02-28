#include "GlobalParams.h"

GlobalParams& GlobalParams::getInstance() {
  static GlobalParams instance;
  return instance;
}

template <class T>
void GlobalParams::ReadVariable(const ros::NodeHandle& nh,
                                const std::string& variable_name, T& variable) {
  if (!nh.getParam(variable_name, variable)) {
    std::cout
        << "Could not read param max_features from parameter server, so using "
           "default value "
        << variable << std::endl;
  }
}

void GlobalParams::LoadParams(const ros::NodeHandle& nh) {
  ReadVariable(nh, "max_features", getInstance().max_features_);
}

int GlobalParams::MaxFeatures() { return getInstance().max_features_; }
