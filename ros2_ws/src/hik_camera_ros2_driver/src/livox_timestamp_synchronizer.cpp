#include "hik_camera_ros2_driver/livox_timestamp_synchronizer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>

namespace hik_camera_ros2_driver
{

LivoxTimestampSynchronizer::LivoxTimestampSynchronizer(
  const LivoxTimestampSynchronizerConfig & config)
: config_(config)
{
}

bool LivoxTimestampSynchronizer::addSample(
  int64_t stamp_ns,
  int64_t header_stamp_ns,
  int64_t host_receive_steady_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (stamp_ns <= 0) {
    ++stats_.invalid_livox_stamps;
    return false;
  }
  if (header_stamp_ns != stamp_ns) {
    ++stats_.header_mismatch_stamps;
    return false;
  }
  if (last_livox_stamp_ns_ > 0 && stamp_ns == last_livox_stamp_ns_) {
    ++stats_.duplicate_livox_stamps;
    return false;
  }
  if (last_livox_stamp_ns_ > 0 && stamp_ns < last_livox_stamp_ns_) {
    ++stats_.rollback_livox_stamps;
    return false;
  }
  if (last_livox_stamp_ns_ > 0 && isAbnormalInterval(stamp_ns - last_livox_stamp_ns_)) {
    ++stats_.abnormal_interval_stamps;
    return false;
  }

  queue_.push_back({stamp_ns, host_receive_steady_ns});
  last_livox_stamp_ns_ = stamp_ns;
  ++stats_.accepted_livox_stamps;

  while (queue_.size() > config_.queue_size) {
    queue_.pop_front();
    ++stats_.queue_overflow_drops;
  }

  cv_.notify_all();
  return true;
}

LivoxPairingResult LivoxTimestampSynchronizer::matchImage(int64_t image_host_steady_ns)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const auto timeout = std::chrono::nanoseconds(config_.wait_timeout_ns);
  cv_.wait_for(lock, timeout, [this]() {return shutdown_ || !queue_.empty();});

  LivoxPairingResult result;
  result.queue_size = queue_.size();
  if (shutdown_ || queue_.empty()) {
    ++stats_.dropped_unsynced_frames;
    return result;
  }

  std::size_t best_index = 0;
  int64_t best_abs_delta = std::llabs(queue_[0].host_receive_steady_ns - image_host_steady_ns);
  for (std::size_t i = 1; i < queue_.size(); ++i) {
    const int64_t delta = queue_[i].host_receive_steady_ns - image_host_steady_ns;
    const int64_t abs_delta = std::llabs(delta);
    if (abs_delta < best_abs_delta) {
      best_abs_delta = abs_delta;
      best_index = i;
    }
  }

  const LivoxStampSample sample = queue_[best_index];
  if (best_abs_delta > config_.max_pairing_host_delta_ns) {
    ++stats_.dropped_unsynced_frames;
    return result;
  }
  if (last_published_image_stamp_ns_ > 0 && sample.stamp_ns <= last_published_image_stamp_ns_) {
    ++stats_.image_rollback_drops;
    return result;
  }

  queue_.erase(queue_.begin(), queue_.begin() + static_cast<std::ptrdiff_t>(best_index + 1));
  last_published_image_stamp_ns_ = sample.stamp_ns;
  ++stats_.matched_frames;

  result.matched = true;
  result.sample = sample;
  result.host_delta_ns = sample.host_receive_steady_ns - image_host_steady_ns;
  result.queue_size = queue_.size();
  return result;
}

void LivoxTimestampSynchronizer::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  shutdown_ = true;
  cv_.notify_all();
}

LivoxTimestampStats LivoxTimestampSynchronizer::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

std::size_t LivoxTimestampSynchronizer::queueSize() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

bool LivoxTimestampSynchronizer::isAbnormalInterval(int64_t interval_ns) const
{
  return interval_ns <= 0 || interval_ns > 1000000000LL;
}

}  // namespace hik_camera_ros2_driver
