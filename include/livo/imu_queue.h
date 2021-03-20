#ifndef ORB_TEST_SRC_IMU_QUEUE_H_
#define ORB_TEST_SRC_IMU_QUEUE_H_

#include <map>
#include <mutex>
#include <sensor_msgs/Imu.h>
#include <memory>

using namespace std;

// Forward declarations for gtsam
namespace gtsam {
  class TangentPreintegration;
  typedef TangentPreintegration PreintegrationType;
}


class IMUQueue {
private:
  map<double, sensor_msgs::Imu> imuMap;
  mutex mu;

public:
  void addMeasurement(const sensor_msgs::Imu &measurement);

  bool hasMeasurementsInRange(ros::Time start, ros::Time end);
  bool hasMeasurementsInRange(double start, double end);

  int integrateIMUMeasurements(std::shared_ptr<gtsam::PreintegrationType> &imuMeasurements, ros::Time start, ros::Time end);
  int integrateIMUMeasurements(std::shared_ptr<gtsam::PreintegrationType> &imuMeasurements, double start, double end);

};


#endif  // ORB_TEST_SRC_IMU_QUEUE_H_
