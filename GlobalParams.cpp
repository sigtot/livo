#include "GlobalParams.h"

using namespace std;

void GlobalParams::loadParams(const ros::NodeHandle& nh) {
  if (!nh.getParam("max_features", getInstance().max_features_)) {
    cout << "Could not read param max_features from parameter server, so using "
            "default value "
         << getInstance().max_features_ << endl;
  }
}

int GlobalParams::MaxFeatures() { return getInstance().max_features_; }
