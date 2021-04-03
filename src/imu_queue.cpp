#include "imu_queue.h"
#include "global_params.h"
#include "gtsam_conversions.h"

#include <iostream>
#include <gtsam/navigation/CombinedImuFactor.h>

void IMUQueue::addMeasurement(const sensor_msgs::Imu& measurement)
{
  lock_guard<mutex> lock(mu);
  auto stampCorrectedMeasurement = measurement;
  stampCorrectedMeasurement.header.stamp = measurement.header.stamp - ros::Duration(GlobalParams::TimeshiftCamImu());
  imuMap[stampCorrectedMeasurement.header.stamp.toSec()] = stampCorrectedMeasurement;

  /* Debug IMU orientation
  gtsam::Rot3 body_R_imu = gtsam::Rot3::Quaternion(GlobalParams::BodyPImuQuat()[3], GlobalParams::BodyPImuQuat()[0],
                                                   GlobalParams::BodyPImuQuat()[1], GlobalParams::BodyPImuQuat()[2]);

  auto acc =
      gtsam::Vector3(stampCorrectedMeasurement.linear_acceleration.x, stampCorrectedMeasurement.linear_acceleration.y,
                     stampCorrectedMeasurement.linear_acceleration.z);
  std::cout << "acc: " << acc.transpose() << std::endl;
  std::cout << "acc corrected: " << (body_R_imu * acc).transpose() << std::endl;
   */
}

Rot3 IMUQueue::RefineInitialAttitude(ros::Time start, ros::Time end, const Rot3& init_rot)
{
  if (!hasMeasurementsInRange(start, end))
  {
    return Rot3::Eye();
  }

  gtsam::Rot3 body_R_imu = gtsam::Rot3::Quaternion(GlobalParams::BodyPImuQuat()[3], GlobalParams::BodyPImuQuat()[0],
                                                   GlobalParams::BodyPImuQuat()[1], GlobalParams::BodyPImuQuat()[2]);

  int num_summed = 0;
  gtsam::Vector3 acc_sum(0, 0, 0);

  lock_guard<mutex> lock(mu);
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
  auto acc_sum_corrected = ToGtsamRot(init_rot) * body_R_imu * acc_sum;
  acc_sum_corrected.normalize();           // Normalize so both vectors have length 1
  gtsam::Vector3 acc_at_rest(0., 0., 1.);  // An accelerometer at rest will measure 9.81 m/s^2 straight upwards
  auto rotation_diff = gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(acc_sum_corrected, acc_at_rest));
  auto aligned_init_rot = rotation_diff * ToGtsamRot(init_rot);
  Rot3 aligned = ToRot(aligned_init_rot);

  std::cout << "Aligned initial attitude along gravity using stationary IMU measurements: " << aligned.x << ", "
            << aligned.y << ", " << aligned.z << ", " << aligned.w << std::endl;

  auto aligned_acc = aligned_init_rot * body_R_imu * acc_sum;
  std::cout << "Measured acceleration stationary acceleration: " << aligned_acc.transpose() << std::endl;

  return aligned;
}

Rot3 IMUQueue::RefineInitialAttitude(double start, double end, const Rot3& init_rot)
{
  return RefineInitialAttitude(ros::Time(start), ros::Time(end), init_rot);
}

bool IMUQueue::hasMeasurementsInRange(ros::Time start, ros::Time end)
{
  lock_guard<mutex> lock(mu);
  int betweenCount = 0;
  for (auto& it : imuMap)
  {
    auto imuStamp = it.second.header.stamp;
    if (imuStamp > end)
    {
      break;
    }
    if (imuStamp > start)
    {
      ++betweenCount;
    }
  }
  return betweenCount > 1;  // TODO make it > 0?
}

bool IMUQueue::hasMeasurementsInRange(double start, double end)
{
  return hasMeasurementsInRange(ros::Time(start), ros::Time(end));
}

int IMUQueue::integrateIMUMeasurements(std::shared_ptr<gtsam::PreintegrationType>& imuMeasurements, ros::Time start,
                                       ros::Time end)
{
  lock_guard<mutex> lock(mu);
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
    cout << "WARN: imu integration ran to the end of the queue. This could imply that some msgs were lost" << endl;
  }
  return numIntg;
}

int IMUQueue::integrateIMUMeasurements(shared_ptr<gtsam::PreintegrationType>& imuMeasurements, double start, double end)
{
  return integrateIMUMeasurements(imuMeasurements, ros::Time(start), ros::Time(end));
}
