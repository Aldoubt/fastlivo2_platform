#ifndef HIK_CAMERA_ROS2_DRIVER__LIVOX_TIMESTAMP_SYNCHRONIZER_HPP_
#define HIK_CAMERA_ROS2_DRIVER__LIVOX_TIMESTAMP_SYNCHRONIZER_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

namespace hik_camera_ros2_driver
{

struct LivoxTimestampSynchronizerConfig
{
  std::size_t queue_size = 30;
  int64_t wait_timeout_ns = 80000000;
  int64_t max_pairing_host_delta_ns = 50000000;
};

struct LivoxStampSample
{
  int64_t stamp_ns = 0;
  int64_t host_receive_steady_ns = 0;
};

struct LivoxTimestampStats
{
  uint64_t accepted_livox_stamps = 0;
  uint64_t invalid_livox_stamps = 0;
  uint64_t duplicate_livox_stamps = 0;
  uint64_t rollback_livox_stamps = 0;
  uint64_t abnormal_interval_stamps = 0;
  uint64_t header_mismatch_stamps = 0;
  uint64_t queue_overflow_drops = 0;
  uint64_t matched_frames = 0;
  uint64_t dropped_unsynced_frames = 0;
  uint64_t image_rollback_drops = 0;
};

struct LivoxPairingResult
{
  bool matched = false;
  LivoxStampSample sample;
  int64_t host_delta_ns = 0;
  std::size_t queue_size = 0;
};

class LivoxTimestampSynchronizer
{
public:
  explicit LivoxTimestampSynchronizer(const LivoxTimestampSynchronizerConfig & config);

  bool addSample(
    int64_t stamp_ns,
    int64_t header_stamp_ns,
    int64_t host_receive_steady_ns);

  LivoxPairingResult matchImage(int64_t image_host_steady_ns);

  void shutdown();

  LivoxTimestampStats stats() const;
  std::size_t queueSize() const;

private:
  bool isAbnormalInterval(int64_t interval_ns) const;

  LivoxTimestampSynchronizerConfig config_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<LivoxStampSample> queue_;
  LivoxTimestampStats stats_;
  int64_t last_livox_stamp_ns_ = 0;
  int64_t last_published_image_stamp_ns_ = 0;
  bool shutdown_ = false;
};

}  // namespace hik_camera_ros2_driver

#endif  // HIK_CAMERA_ROS2_DRIVER__LIVOX_TIMESTAMP_SYNCHRONIZER_HPP_
