#ifndef ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
#define ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_

#include <string>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Rot3.h>
#include <opencv2/core/core.hpp>

void SaveGraphToFile(const std::string& filename, const gtsam::NonlinearFactorGraph& graph,
                     const gtsam::Values& values);
void PrintVariableStatus(const gtsam::ISAM2Result::DetailedResults::VariableStatus& variable_status);

double ComputeParallaxWithOpenCV(const gtsam::Point2& point1, const gtsam::Point2& point2,
                                 const gtsam::Rot3& body1_R_body2, const boost::shared_ptr<gtsam::Cal3_S2>& K,
                                 const gtsam::Rot3& body_R_cam);

#endif  // ORB_TEST_INCLUDE_LIVO_GTSAM_HELPERS_H_
