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

gtsam::Rot3 IMUQueue::RefineInitialAttitude(ros::Time start, ros::Time end, const gtsam::Rot3& w_R_body_init,
                                            const gtsam::Rot3& body_R_imu)
{
  if (!hasMeasurementsInRange(start, end))
  {
    return w_R_body_init;
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
  acc_sum.normalize();  // Normalize so both vectors have length 1
  auto b_g = body_R_imu * acc_sum;  // Gravity in body frame
  gtsam::Vector3 w_g(0., 0., 1.);  // An accelerometer at rest will measure 9.81 m/s^2 straight upwards
  auto w_R_b_pitch_roll = gtsam::Rot3(Eigen::Quaterniond::FromTwoVectors(b_g, w_g));

  // Aligning gravity with [0, 0, 1] only makes roll and pitch observable. For yaw we use the value initially given.
  auto w_R_b = gtsam::Rot3::Ypr(w_R_body_init.yaw(), w_R_b_pitch_roll.pitch(), w_R_b_pitch_roll.roll());

  std::cout << "Aligned attitude along gravity using stationary IMU measurements." << std::endl;
  std::cout << "Aligned: " << w_R_b.toQuaternion().coeffs().transpose() << std::endl;

  auto w_g_estimated = w_R_b * b_g;
  std::cout << "Measured acceleration stationary accel unaligned: " << (b_g).transpose() << std::endl;
  std::cout << "Measured acceleration stationary accel aligned: " << w_g_estimated.transpose() << std::endl;

  return w_R_b;
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
  bool ran_to_end_of_queue = true;
  bool isPastEnd = false;
  auto it = GetFirstMeasurementInRange(start.toSec(), end.toSec());
  for (; it != imuMap.end(); ++it)
  {
    auto imuMsg = it->second;
    if (imuMsg.header.stamp > end)
    {
      isPastEnd = true;
    }

    auto dt = isPastEnd ? end - lastTime : imuMsg.header.stamp - lastTime;
    auto linearMsg = imuMsg.linear_acceleration;
    auto acc = gtsam::Vector3(linearMsg.x, linearMsg.y, linearMsg.z);
    auto angularMsg = imuMsg.angular_velocity;
    auto omega = gtsam::Vector3(angularMsg.x, angularMsg.y, angularMsg.z);
    imuMeasurements->integrateMeasurement(acc, omega, dt.toSec());
    lastTime = imuMsg.header.stamp;
    numIntg++;

    if (isPastEnd)
    {
      ran_to_end_of_queue = false;
      break;
    }
  }
  if (ran_to_end_of_queue)
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
