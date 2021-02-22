#include "Smoother.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

typedef BatchFixedLagSmoother::KeyTimestampMap TimestampMap;

Smoother::Smoother() {
    // TODO impl
}

void Smoother::update() {
    NonlinearFactorGraph newFactors;
    Values newValues;
    TimestampMap newTimestamps;
}
