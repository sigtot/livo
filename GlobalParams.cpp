#include "GlobalParams.h"

GlobalParams& GlobalParams::GetInstance() {
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
  // Add ReadVariable calls here
  ReadVariable(nh, "max_features_per_cell",
               GetInstance().max_features_per_cell_);
}

// Implement parameter accessors here
int GlobalParams::MaxFeaturesPerCell() {
  return GetInstance().max_features_per_cell_;
}