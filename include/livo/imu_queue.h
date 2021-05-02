#ifndef ORB_TEST_SRC_IMU_QUEUE_H_
#define ORB_TEST_SRC_IMU_QUEUE_H_

#include <map>
#include <mutex>
#include <sensor_msgs/Imu.h>
#include <memory>

#include "rot3.h"

// Forward declarations for gtsam
namespace gtsam
{
class TangentPreintegration;
typedef TangentPreintegration PreintegrationType;
}  // namespace gtsam

class IMUQueue
{
private:
  std::map<double, sensor_msgs::Imu> imuMap;
  std::mutex mu;

public:
  void addMeasurement(const sensor_msgs::Imu& measurement);

  Rot3 RefineInitialAttitude(ros::Time start, ros::Time end, const Rot3& init_rot);
  Rot3 RefineInitialAttitude(double start, double end, const Rot3& init_rot);

  bool hasMeasurementsInRange(ros::Time start, ros::Time end);
  bool hasMeasurementsInRange(double start, double end);

  int integrateIMUMeasurements(std::shared_ptr<gtsam::PreintegrationType>& imuMeasurements, ros::Time start,
                               ros::Time end);
  int integrateIMUMeasurements(std::shared_ptr<gtsam::PreintegrationType>& imuMeasurements, double start, double end);
};

#endif  // ORB_TEST_SRC_IMU_QUEUE_H_
