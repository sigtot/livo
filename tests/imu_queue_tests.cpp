#include <gtest/gtest.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

#include "imu_queue.h"

#include "helpers.h"

sensor_msgs::Imu make_imu_measurement(double ts, double accel = 0.1, double gyro = 0.1)
{
  sensor_msgs::Imu msg;
  msg.header.frame_id = "world";
  msg.header.stamp = ros::Time(ts);

  msg.linear_acceleration.x = accel;
  msg.linear_acceleration.y = accel;
  msg.linear_acceleration.z = accel;

  msg.angular_velocity.x = gyro;
  msg.angular_velocity.y = gyro;
  msg.angular_velocity.z = gyro;

  return msg;
}

TEST(IMUQueue, IntegratesCorrectNumberOfMessages)
{
  IMUQueue queue(0.0, 10000);
  std::vector<sensor_msgs::Imu> msgs{
    make_imu_measurement(0.0), make_imu_measurement(2.0), make_imu_measurement(4.0),
    make_imu_measurement(6.0), make_imu_measurement(8.0), make_imu_measurement(10.0),
  };

  for (const auto& msg : msgs)
  {
    queue.addMeasurement(msg);
  }

  auto pim = std::make_shared<gtsam::PreintegrationType>(PimParams());

  int n_integrated = queue.integrateIMUMeasurements(pim, 3.0, 7.0);

  EXPECT_EQ(n_integrated, 3);
}

TEST(IMUQueue, HasMeasurementsInRange)
{
  IMUQueue queue(0.0, 10000);
  std::vector<sensor_msgs::Imu> msgs{
    make_imu_measurement(1.0),
    make_imu_measurement(2.0),
  };

  for (const auto& msg : msgs)
  {
    queue.addMeasurement(msg);
  }

  EXPECT_TRUE(queue.hasMeasurementsInRange(0.0, 1.5));
  EXPECT_TRUE(queue.hasMeasurementsInRange(1.5, 2.5));
  EXPECT_FALSE(queue.hasMeasurementsInRange(2.5, 3.5));
  EXPECT_FALSE(queue.hasMeasurementsInRange(1.5, 1.6));
}