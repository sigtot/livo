#include "gtsam_helpers.h"

#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/Pose3.h>

bool CanTriangulate(const Camera::MeasurementVector& measurements, const gtsam::CameraSet<Camera>& cameras,
                    const gtsam::Cal3_S2::shared_ptr& K)
{
  gtsam::TriangulationParameters params(1.0, false, 15.0, 10);
  auto triangulationResult = gtsam::triangulateSafe(cameras, measurements, params);

  return triangulationResult != boost::none;
}
