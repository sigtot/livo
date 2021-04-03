#ifndef ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
#include "track.h"

#include <memory>
#include <string>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/CameraSet.h>

typedef gtsam::PinholePose<gtsam::Cal3_S2> Camera;

bool CanTriangulate(const Camera::MeasurementVector& measurements, const gtsam::CameraSet<Camera>& cameras,
                    const gtsam::Cal3_S2::shared_ptr& K);

void SaveGraphToFile(const std::string& filename, const gtsam::NonlinearFactorGraph& graph,
                      const gtsam::Values& values);

#endif  // ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
