#include "imu_queue.h"
#include "gtsam_conversions.h"

#include <iostream>
#include <gtsam/navigation/CombinedImuFactor.h>

IMUQueue::IMUQueue(double timeshift_cam_imu, int max_messages_retained)
  : timeshift_cam_imu_(timeshift_cam_imu), max_messages_retained_(max_messages_retained)
{
}

void IMUQueue::addMeasurement(const sensor_msgs::Imu& measurement)
{
  std::lock_guard<std::mutex> lock(mu);
  auto stampCorrectedMeasurement = measurement;
  stampCorrectedMeasurement.header.stamp = measurement.header.stamp - ros::Duration(timeshift_cam_imu_);
  imuMap[stampCorrectedMeasurement.header.stamp.toSec()] = stampCorrectedMeasurement;

  if (imuMap.size() > max_messages_retained_)
  {
    imuMap.erase(imuMap.begin());
  }

  /* Debug IMU orientation make sure to include "global_params.h"
  gtsam::Rot3 body_R_imu = gtsam::Rot3::Quaternion(GlobalParams::BodyPImuQuat()[3], GlobalParams::BodyPImuQuat()[0],
                                                   GlobalParams::BodyPImuQuat()[1], GlobalParams::BodyPImuQuat()[2]);

  auto acc =
      gtsam::Vector3(stampCorrectedMeasurement.linear_acceleration.x, stampCorrectedMeasurement.linear_acceleration.y,
                     stampCorrectedMeasurement.linear_acceleration.z);
  std::cout << "acc: " << acc.transpose() << std::endl;
  std::cout << "acc corrected: " << (body_R_imu * acc).transpose() << std::endl;
   */
}

gtsam::Rot3 IMUQueue::RefineInitialAttitude(ros::Time start, ros::Time end, const gtsam::Rot3& init_rot,
                                            const gtsam::Rot3& body_R_imu)
{
  if (!hasMeasurementsInRange(start, end))
  {
    return init_rot;
  }

  int num_summed = 0;
  gtsam::Vector3 acc_sum(0, 0, 0);

  std::lock_guard<std::mutex> lock(mu);
  for (auto& it : imuMap)
  {
    auto imuMsg = it.second;
    if (imuMsg.header.stamp > end || num_summed > 200)
    {
      break;
    }
    if (imuMsg.header.stamp > start)
    {
      auto linearMsg = imuMsg.linear_acceleration;
      gtsam::Vector3 acc(linearMsg.x, linearMsg.y, linearMsg.z);
      acc_sum += acc;
      num_summed++;
    }
  }
  acc_sum /= static_cast<double>(num_summed);
  auto acc_sum_corrected = init_rot * body_R_imu * acc_sum;
  acc_sum_corrected.normalize();           // Normalize so both vectors have length 1
  gtsam::Vector3 acc_at_rest(0., 0., 1.);  // An accelerometer at rest will measure 9.81 m/s^2 straight upwards
  auto rotation_diff = gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(acc_sum_corrected, acc_at_rest));
  auto aligned = rotation_diff * init_rot;

  std::cout << "Aligned initial attitude along gravity using stationary IMU measurements." << std::endl;
  std::cout << "Initial: " << init_rot.toQuaternion().coeffs().transpose() << std::endl;
  std::cout << "Aligned: " << aligned.toQuaternion().coeffs().transpose() << std::endl;

  auto aligned_acc = aligned * body_R_imu * acc_sum;
  std::cout << "Measured acceleration stationary accel unaligned: " << (body_R_imu * acc_sum).transpose() << std::endl;
  std::cout << "Measured acceleration stationary accel aligned: " << aligned_acc.transpose() << std::endl;

  return aligned;
}

gtsam::Rot3 IMUQueue::RefineInitialAttitude(double start, double end, const gtsam::Rot3& init_rot,
                                            const gtsam::Rot3& body_R_imu)
{
  return RefineInitialAttitude(ros::Time(start), ros::Time(end), init_rot, body_R_imu);
}

bool IMUQueue::hasMeasurementsInRange(ros::Time start, ros::Time end)
{
  std::lock_guard<std::mutex> lock(mu);
  auto first_in_range = GetFirstMeasurementInRange(start.toSec(), end.toSec());
  return first_in_range != imuMap.end();
}

bool IMUQueue::hasMeasurementsInRange(double start, double end)
{
  return hasMeasurementsInRange(ros::Time(start), ros::Time(end));
}

int IMUQueue::integrateIMUMeasurements(std::shared_ptr<gtsam::PreintegrationType>& imuMeasurements, ros::Time start,
                                       ros::Time end)
{
  std::lock_guard<std::mutex> lock(mu);
  int numIntg = 0;
  auto lastTime = start;
  bool ranToEnd = true;
  bool isPastEnd = false;
  for (auto& it : imuMap)
  {
    auto imuMsg = it.second;
    if (imuMsg.header.stamp > end)
    {
      isPastEnd = true;
    }
    if (imuMsg.header.stamp > start)
  {
    auto dt = isPastEnd ? end - lastTime : imuMsg.header.stamp - lastTime;
      auto linearMsg = imuMsg.linear_acceleration;
      auto acc = gtsam::Vector3(linearMsg.x, linearMsg.y, linearMsg.z);
      auto angularMsg = imuMsg.angular_velocity;
      auto omega = gtsam::Vector3(angularMsg.x, angularMsg.y, angularMsg.z);
      imuMeasurements->integrateMeasurement(acc, omega, dt.toSec());
      lastTime = imuMsg.header.stamp;
      numIntg++;
    }
    if (isPastEnd)
    {
      ranToEnd = false;
      break;
    }
  }
  if (ranToEnd)
  {
    std::cout << "WARN: imu integration ran to the end of the queue. This could imply that some msgs were lost"
              << std::endl;
  }
  return numIntg;
}

int IMUQueue::integrateIMUMeasurements(std::shared_ptr<gtsam::PreintegrationType>& imuMeasurements, double start,
                                       double end)
{
  return integrateIMUMeasurements(imuMeasurements, ros::Time(start), ros::Time(end));
}

std::map<double, sensor_msgs::Imu>::iterator IMUQueue::GetFirstMeasurementInRange(double start, double end)
{
  auto it = imuMap.lower_bound(start);

  // If we have a non-end iterator here, we know from the lower_bound that it will be after start, but we still need to
  // check that it is before end.
  if (it != imuMap.end() && it->first > end)
  {
    return imuMap.end();
  }

  // The final return can be either imuMap.end() or the first measurement in range
  return it;
}
