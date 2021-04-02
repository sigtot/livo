#include "imu_queue.h"
#include "global_params.h"

#include <iostream>
#include <gtsam/navigation/CombinedImuFactor.h>

void IMUQueue::addMeasurement(const sensor_msgs::Imu& measurement)
{
  lock_guard<mutex> lock(mu);
  auto stampCorrectedMeasurement = measurement;
  stampCorrectedMeasurement.header.stamp = measurement.header.stamp - ros::Duration(GlobalParams::TimeshiftCamImu());
  imuMap[stampCorrectedMeasurement.header.stamp.toSec()] = stampCorrectedMeasurement;
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
