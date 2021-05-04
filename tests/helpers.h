#ifndef ORB_TEST_TESTS_HELPERS_H_
#define ORB_TEST_TESTS_HELPERS_H_

#include <boost/shared_ptr.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> PimParams();

#endif  // ORB_TEST_TESTS_HELPERS_H_
