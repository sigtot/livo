#ifndef ORB_TEST_SRC_QUEUED_MEASUREMENT_PROCESSOR_H_
#define ORB_TEST_SRC_QUEUED_MEASUREMENT_PROCESSOR_H_

#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <map>

#include <ros/init.h>

template <class T>
class QueuedMeasurementProcessor
{
public:
  /**
   * @param process_fn The function called to process the oldest measurement in the queue
   * @param min_process_count The minimum number of items needed in the the queue for oldest item to be processed
   */
  QueuedMeasurementProcessor(std::function<void(const T&)> const& process_fn, int min_process_count)
    : process_fn_(process_fn), min_process_count_(min_process_count)
  {
    process_thread_ = std::thread(&QueuedMeasurementProcessor::waitAndProcessMessages, this);
  }

  size_t size() const
  {
    return measurements_.size();
  }

  void addMeasurement(const T& measurement)
  {
    {
      std::lock_guard<std::mutex> lock(measurement_mutex_);
      if (measurement->header.stamp.toSec() > last_processed_timestamp_)
      {
        measurements_[measurement->header.stamp.toSec()] = measurement;
      }
      else
      {
        std::cout << std::fixed << "Rejected measurement because its timestamp is before the one last processed ("
                  << measurement->header.stamp.toSec() << " < " << last_processed_timestamp_ << ")" << std::endl;
      }
    }
    cv_.notify_one();
  }

  virtual ~QueuedMeasurementProcessor()
  {
    cv_.notify_all();
    process_thread_.join();
  }

private:
  void waitAndProcessMessages()
  {
    std::unique_lock<std::mutex> newMeasurementNotifier(new_measurement_notifier_mutex_);
    while (!ros::isShuttingDown())
    {
      T measurement;
      {
        std::lock_guard<std::mutex> lock(measurement_mutex_);
        if (measurements_.size() >= min_process_count_)
        {
          measurement = measurements_.begin()->second;
          measurements_.erase(measurements_.begin());
          process_fn_(measurement);
          last_processed_timestamp_ = measurement->header.stamp.toSec();
        }
      }
      if (measurements_.size() < min_process_count_)
      {
        cv_.wait(newMeasurementNotifier);
      }
    }
  }

  const int min_process_count_;

  double last_processed_timestamp_ = 0.0;

  std::mutex new_measurement_notifier_mutex_;
  std::condition_variable cv_;

  std::mutex measurement_mutex_;
  std::map<double, T> measurements_;

  std::function<void(T measurement)> process_fn_;
  std::thread process_thread_;
};

#endif  // ORB_TEST_SRC_QUEUED_MEASUREMENT_PROCESSOR_H_
