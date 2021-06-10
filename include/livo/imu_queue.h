#ifndef ORB_TEST_SRC_IMU_QUEUE_H_
#define ORB_TEST_SRC_IMU_QUEUE_H_

#include <map>
#include <mutex>
#include <sensor_msgs/Imu.h>
#include <memory>
#include <gtsam/geometry/Rot3.h>

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

  double timeshift_cam_imu_;

public:
  IMUQueue(double timeshift_cam_imu, int max_messages_retained);

private:
  int max_messages_retained_;

  /**
   * Return an iterator to the first measurement within the range [start, end]. If there are none within the range,
   * a past-the-end iterator is returned. It basically amounts to a call to std::map::lower_bound with the added
   * requirement that the iterator must also be before end.
   *
   * @warning Not threadsafe. Lock the mutex before using.
   */
  std::map<double, sensor_msgs::Imu>::iterator GetFirstMeasurementInRange(double start, double end);

public:
  void addMeasurement(const sensor_msgs::Imu& measurement);

  gtsam::Rot3 RefineInitialAttitude(ros::Time start, ros::Time end, const gtsam::Rot3& init_rot,
                                    const gtsam::Rot3& body_R_imu);
  gtsam::Rot3 RefineInitialAttitude(double start, double end, const gtsam::Rot3& init_rot,
                                    const gtsam::Rot3& body_R_imu);

  bool hasMeasurementsInRange(ros::Time start, ros::Time end);
  bool hasMeasurementsInRange(double start, double end);

  int integrateIMUMeasurements(std::shared_ptr<gtsam::PreintegrationType>& imuMeasurements, ros::Time start,
                               ros::Time end);

  /**
   * Integrate IMU measurements between timestamps start and end. The integration includes the measurement immediately
   * following the end, but not the measurement immediately preceding start. The first and last integrated measurements
   * are integrated over the time delta between the measurement and start or end respectively.
   *
   * @example Given measurements at times 2.0, 4.0, 6.0 and 8.0: When we integrate from start=3.0 to end=7.0,
   * the measurements at 4.0, 6.0, and 8.0 are included in the integration.
   *
   * @param imuMeasurements
   * @param start
   * @param end
   * @return number of measurements integrated
   */
  int integrateIMUMeasurements(std::shared_ptr<gtsam::PreintegrationType>& imuMeasurements, double start, double end);
};

#endif  // ORB_TEST_SRC_IMU_QUEUE_H_
