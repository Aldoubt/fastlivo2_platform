#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "MvCameraControl.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "hik_camera_ros2_driver/livox_timestamp_synchronizer.hpp"
#include "image_transport/image_transport.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"

namespace hik_camera_ros2_driver
{
namespace
{
std::string sdkStatusToHex(int status)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase << static_cast<uint32_t>(status);
  return stream.str();
}

int64_t steadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

int64_t stampToNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}
}  // namespace

class HikCameraRos2DriverNode : public rclcpp::Node
{
public:
  explicit HikCameraRos2DriverNode(const rclcpp::NodeOptions & options)
  : Node("hik_camera_ros2_driver", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraRos2DriverNode!");

    if (!initializeCamera() || !declareAndApplyParameters() || !startCamera()) {
      RCLCPP_FATAL(this->get_logger(), "Camera initialization failed; grabbing thread will not start.");
      rclcpp::shutdown();
      return;
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraRos2DriverNode::dynamicParametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread(&HikCameraRos2DriverNode::captureLoop, this);
  }

  ~HikCameraRos2DriverNode() override
  {
    running_ = false;
    if (livox_synchronizer_) {
      livox_synchronizer_->shutdown();
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      int status = MV_CC_StopGrabbing(camera_handle_);
      if (status != MV_OK) {
        RCLCPP_DEBUG(this->get_logger(), "Stop grabbing during shutdown returned %s",
          sdkStatusToHex(status).c_str());
      }
      status = MV_CC_CloseDevice(camera_handle_);
      if (status != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to close camera device, status = %s",
          sdkStatusToHex(status).c_str());
      }
      status = MV_CC_DestroyHandle(&camera_handle_);
      if (status != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to destroy camera handle, status = %s",
          sdkStatusToHex(status).c_str());
      }
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraRos2DriverNode destroyed!");
  }

private:
  bool initializeCamera()
  {
    MV_CC_DEVICE_INFO_LIST device_list;
    std::memset(&device_list, 0, sizeof(device_list));

    while (rclcpp::ok()) {
      n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
      if (n_ret_ != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enumerate devices, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else if (device_list.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "No camera found, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else {
        RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);
        break;
      }
    }
    if (!rclcpp::ok()) {
      return false;
    }

    n_ret_ = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create camera handle, status = %s",
        sdkStatusToHex(n_ret_).c_str());
      return false;
    }

    n_ret_ = MV_CC_OpenDevice(camera_handle_);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera device, status = %s",
        sdkStatusToHex(n_ret_).c_str());
      return false;
    }

    n_ret_ = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get camera image info, status = %s",
        sdkStatusToHex(n_ret_).c_str());
      return false;
    }

    image_msg_.data.resize(img_info_.nHeightMax * img_info_.nWidthMax * 3);
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    return true;
  }

  bool declareAndApplyParameters()
  {
    camera_info_url_ = this->declare_parameter(
      "camera_info_url", "package://hik_camera_ros2_driver/config/camera_info.yaml");
    pixel_format_ = this->declare_parameter("pixel_format", "RGB8Packed");
    adc_bit_depth_ = this->declare_parameter("adc_bit_depth", "Bits_8");
    use_sensor_data_qos_ = this->declare_parameter("use_sensor_data_qos", true);
    image_scale_ = this->declare_parameter("image_scale", 1.0);
    camera_name_ = this->declare_parameter("camera_name", "camera");
    frame_id_ = this->declare_parameter("frame_id", camera_name_ + "_optical_frame");
    camera_topic_ = this->declare_parameter("camera_topic", camera_name_ + "/image");

    trigger_mode_ = this->declare_parameter("trigger_mode", false);
    trigger_source_ = this->declare_parameter("trigger_source", "Line0");
    trigger_activation_ = this->declare_parameter("trigger_activation", "RisingEdge");
    trigger_delay_us_ = this->declare_parameter("trigger_delay_us", 0.0);
    timestamp_source_ = this->declare_parameter("timestamp_source", "ros_now");
    livox_topic_ = this->declare_parameter("livox_topic", "/livox/lidar");
    sync_queue_size_ = static_cast<std::size_t>(
      this->declare_parameter("sync_queue_size", 30));
    sync_wait_timeout_ms_ = this->declare_parameter("sync_wait_timeout_ms", 80);
    max_pairing_host_delta_ms_ = this->declare_parameter("max_pairing_host_delta_ms", 50.0);
    timestamp_diagnostics_ = this->declare_parameter("timestamp_diagnostics", true);

    acquisition_frame_rate_enable_ =
      this->declare_parameter("acquisition_frame_rate_enable", true);
    acquisition_frame_rate_ = this->declare_parameter("acquisition_frame_rate", 165.0);

    exposure_auto_ = this->declare_parameter("exposure_auto", "Off");
    exposure_time_ = this->declare_parameter("exposure_time", 5000.0);
    gain_auto_ = this->declare_parameter("gain_auto", "Off");
    gain_ = this->declare_parameter("gain", 15.0);

    if (!validateEnum("exposure_auto", exposure_auto_, auto_modes_) ||
      !validateEnum("gain_auto", gain_auto_, auto_modes_) ||
      !validateEnum("trigger_source", trigger_source_, trigger_sources_) ||
      !validateEnum("trigger_activation", trigger_activation_, trigger_activations_))
    {
      return false;
    }
    if (image_scale_ <= 0.0 || image_scale_ > 1.0) {
      RCLCPP_ERROR(this->get_logger(), "image_scale %.3f is outside range (0.0, 1.0]",
        image_scale_);
      return false;
    }
    if (!validateTimestampParameters()) {
      return false;
    }

    return applyCameraConfiguration();
  }

  bool validateTimestampParameters()
  {
    if (timestamp_source_ == "ros_now") {
      timestamp_source_mode_ = TimestampSourceMode::RosNow;
    } else if (timestamp_source_ == "livox_timebase") {
      timestamp_source_mode_ = TimestampSourceMode::LivoxTimebase;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid timestamp_source value '%s'",
        timestamp_source_.c_str());
      return false;
    }

    if (timestamp_source_mode_ == TimestampSourceMode::LivoxTimebase && !trigger_mode_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "timestamp_source=livox_timebase requires trigger_mode=true");
      return false;
    }
    if (sync_queue_size_ == 0) {
      RCLCPP_ERROR(this->get_logger(), "sync_queue_size must be positive");
      return false;
    }
    if (sync_wait_timeout_ms_ <= 0) {
      RCLCPP_ERROR(this->get_logger(), "sync_wait_timeout_ms must be positive");
      return false;
    }
    if (max_pairing_host_delta_ms_ <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "max_pairing_host_delta_ms must be positive");
      return false;
    }

    return true;
  }

  bool applyCameraConfiguration()
  {
    bool ok = true;

    setEnumStringFeature("ADCBitDepth", adc_bit_depth_, false);
    ok = setEnumStringFeature("PixelFormat", pixel_format_, true) && ok;

    ok = setEnumStringFeature("ExposureAuto", exposure_auto_, true) && ok;
    if (exposure_auto_ == "Off") {
      ok = setFloatFeatureChecked("ExposureTime", exposure_time_, true) && ok;
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Exposure auto is %s; fixed ExposureTime request %.3f us is kept as standby only.",
        exposure_auto_.c_str(), exposure_time_);
    }

    ok = setEnumStringFeature("GainAuto", gain_auto_, true) && ok;
    if (gain_auto_ == "Off") {
      ok = setFloatFeatureChecked("Gain", gain_, true) && ok;
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Gain auto is %s; fixed Gain request %.3f is kept as standby only.",
        gain_auto_.c_str(), gain_);
    }

    if (trigger_mode_) {
      ok = setBoolFeature("AcquisitionFrameRateEnable", false, true) && ok;
      ok = setEnumStringFeature("TriggerMode", "Off", true) && ok;
      ok = setEnumStringFeature("TriggerSource", trigger_source_, true) && ok;
      ok = setEnumStringFeature("TriggerActivation", trigger_activation_, true) && ok;
      ok = setFloatFeatureChecked("TriggerDelay", trigger_delay_us_, false) && ok;
      ok = setEnumStringFeature("TriggerMode", "On", true) && ok;
    } else {
      ok = setEnumStringFeature("TriggerMode", "Off", true) && ok;
      ok = setBoolFeature("AcquisitionFrameRateEnable", acquisition_frame_rate_enable_, true) && ok;
      if (acquisition_frame_rate_enable_) {
        ok = setFloatFeatureChecked("AcquisitionFrameRate", acquisition_frame_rate_, true) && ok;
      }
    }

    logConfigurationSummary();
    return ok;
  }

  bool startCamera()
  {
    auto qos = use_sensor_data_qos_ ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, camera_topic_, qos);
    if (timestamp_source_mode_ == TimestampSourceMode::LivoxTimebase) {
      LivoxTimestampSynchronizerConfig config;
      config.queue_size = sync_queue_size_;
      config.wait_timeout_ns = sync_wait_timeout_ms_ * 1000000LL;
      config.max_pairing_host_delta_ns =
        static_cast<int64_t>(max_pairing_host_delta_ms_ * 1000000.0);
      livox_synchronizer_.reset(new LivoxTimestampSynchronizer(config));

      livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&HikCameraRos2DriverNode::livoxCallback, this, std::placeholders::_1));
    }

    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    if (camera_info_manager_->validateURL(camera_info_url_)) {
      camera_info_manager_->loadCameraInfo(camera_info_url_);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
      applyCameraInfoScale();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url_.c_str());
    }

    n_ret_ = MV_CC_StartGrabbing(camera_handle_);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start grabbing, status = %s",
        sdkStatusToHex(n_ret_).c_str());
      return false;
    }

    return true;
  }

  void captureLoop()
  {
    MV_FRAME_OUT out_frame;
    RCLCPP_INFO(this->get_logger(), "Publishing image!");

    image_msg_.header.frame_id = frame_id_;
    image_msg_.encoding = "rgb8";

    while (rclcpp::ok() && running_) {
      {
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        n_ret_ = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
      }
      const int64_t image_host_steady_ns = steadyNowNs();
      if (MV_OK == n_ret_) {
        rclcpp::Time image_stamp;
        LivoxPairingResult pairing_result;
        if (!resolveImageTimestamp(out_frame, image_host_steady_ns, image_stamp, pairing_result)) {
          freeImageBuffer(out_frame);
          continue;
        }

        const uint32_t src_width = out_frame.stFrameInfo.nWidth;
        const uint32_t src_height = out_frame.stFrameInfo.nHeight;
        const uint32_t dst_width = scaledDimension(src_width);
        const uint32_t dst_height = scaledDimension(src_height);
        const bool do_resize = dst_width != src_width || dst_height != src_height;
        std::vector<uint8_t> & convert_buffer = do_resize ? convert_buffer_ : image_msg_.data;

        convert_buffer.resize(src_width * src_height * 3);
        image_msg_.height = dst_height;
        image_msg_.width = dst_width;
        image_msg_.step = dst_width * 3;
        image_msg_.data.resize(image_msg_.step * image_msg_.height);

        convert_param_.nWidth = out_frame.stFrameInfo.nWidth;
        convert_param_.nHeight = out_frame.stFrameInfo.nHeight;
        convert_param_.pDstBuffer = convert_buffer.data();
        convert_param_.nDstBufferSize = convert_buffer.size();
        convert_param_.pSrcData = out_frame.pBufAddr;
        convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
        convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

        int convert_status = MV_OK;
        {
          std::lock_guard<std::mutex> lock(sdk_mutex_);
          convert_status = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
        }
        if (convert_status != MV_OK) {
          RCLCPP_WARN(this->get_logger(), "MV_CC_ConvertPixelType failed, status = %s",
            sdkStatusToHex(convert_status).c_str());
          freeImageBuffer(out_frame);
          continue;
        }

        if (do_resize) {
          const cv::Mat src_image(src_height, src_width, CV_8UC3, convert_buffer.data());
          cv::Mat dst_image(dst_height, dst_width, CV_8UC3, image_msg_.data.data());
          cv::resize(src_image, dst_image, dst_image.size(), 0.0, 0.0, cv::INTER_LINEAR);
        }

        image_msg_.header.stamp = image_stamp;
        camera_info_msg_.header = image_msg_.header;
        camera_pub_.publish(image_msg_, camera_info_msg_);
        fail_count_ = 0;
        logTimestampDiagnostics(out_frame, pairing_result, false);

        freeImageBuffer(out_frame);

        static auto last_log_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 3) {
          MVCC_FLOATVALUE f_value;
          int status = MV_OK;
          {
            std::lock_guard<std::mutex> lock(sdk_mutex_);
            status = MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &f_value);
          }
          if (status == MV_OK) {
            RCLCPP_DEBUG(this->get_logger(), "ResultingFrameRate: %f Hz", f_value.fCurValue);
          }
          last_log_time = now;
        }

      } else {
        const auto status = static_cast<unsigned int>(n_ret_);
        if (status == MV_E_NODATA || status == MV_E_GC_TIMEOUT) {
          RCLCPP_DEBUG(this->get_logger(),
            "No image received before timeout, status = %s", sdkStatusToHex(n_ret_).c_str());
          continue;
        }

        RCLCPP_WARN(this->get_logger(), "Get buffer failed, status = %s",
          sdkStatusToHex(n_ret_).c_str());
        if (restartGrabbing()) {
          fail_count_ = 0;
        } else {
          fail_count_++;
        }
      }

      if (fail_count_ > 5) {
        RCLCPP_FATAL(this->get_logger(), "Camera failed!");
        rclcpp::shutdown();
      }
    }
  }

  bool resolveImageTimestamp(
    const MV_FRAME_OUT & out_frame,
    int64_t image_host_steady_ns,
    rclcpp::Time & image_stamp,
    LivoxPairingResult & pairing_result)
  {
    if (timestamp_source_mode_ == TimestampSourceMode::RosNow) {
      image_stamp = this->now();
      return true;
    }

    pairing_result = livox_synchronizer_->matchImage(image_host_steady_ns);
    if (!pairing_result.matched) {
      logTimestampDiagnostics(out_frame, pairing_result, true);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Dropping camera frame %u because no compatible Livox timebase was available",
        out_frame.stFrameInfo.nFrameNum);
      return false;
    }

    image_stamp = rclcpp::Time(pairing_result.sample.stamp_ns);
    return true;
  }

  void livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    if (msg->timebase == 0 ||
      msg->timebase > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
    {
      if (livox_synchronizer_) {
        livox_synchronizer_->addSample(0, 0, steadyNowNs());
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Rejected invalid Livox timebase: %llu",
        static_cast<unsigned long long>(msg->timebase));
      return;
    }

    const int64_t stamp_ns = static_cast<int64_t>(msg->timebase);
    const int64_t header_stamp_ns = stampToNs(msg->header.stamp);
    const bool accepted = livox_synchronizer_->addSample(
      stamp_ns, header_stamp_ns, steadyNowNs());
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Rejected Livox timestamp sample: timebase=%ld header=%ld",
        stamp_ns, header_stamp_ns);
    }
  }

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      const auto & name = param.get_name();

      if (name == "exposure_time") {
        if (exposure_auto_ != "Off") {
          result.successful = false;
          result.reason = "exposure_time can only be changed when exposure_auto is Off";
          continue;
        }
        double value = parameterAsDouble(param, result);
        if (!result.successful) {
          continue;
        }
        if (!setFloatFeatureChecked("ExposureTime", value, true)) {
          result.successful = false;
          result.reason = "Failed to set exposure_time";
          continue;
        }
        exposure_time_ = value;
      } else if (name == "gain") {
        if (gain_auto_ != "Off") {
          result.successful = false;
          result.reason = "gain can only be changed when gain_auto is Off";
          continue;
        }
        double value = parameterAsDouble(param, result);
        if (!result.successful) {
          continue;
        }
        if (!setFloatFeatureChecked("Gain", value, true)) {
          result.successful = false;
          result.reason = "Failed to set gain";
          continue;
        }
        gain_ = value;
      } else {
        result.successful = false;
        result.reason = name + " is startup-only in this driver";
      }
    }

    return result;
  }

  double parameterAsDouble(
    const rclcpp::Parameter & param, rcl_interfaces::msg::SetParametersResult & result)
  {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return param.as_double();
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(param.as_int());
    }
    result.successful = false;
    result.reason = param.get_name() + " must be a number";
    return 0.0;
  }

  bool validateEnum(
    const std::string & parameter_name, const std::string & value,
    const std::map<std::string, std::string> & allowed_values)
  {
    if (allowed_values.find(value) != allowed_values.end()) {
      return true;
    }

    RCLCPP_ERROR(this->get_logger(), "Invalid %s value '%s'", parameter_name.c_str(), value.c_str());
    return false;
  }

  bool setBoolFeature(const std::string & feature, bool value, bool required)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int status = MV_CC_SetBoolValue(camera_handle_, feature.c_str(), value);
    if (status == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "%s set to %s", feature.c_str(), value ? "true" : "false");
      return true;
    }
    logSetFailure(feature, value ? "true" : "false", status, required);
    return !required;
  }

  bool setEnumStringFeature(const std::string & feature, const std::string & value, bool required)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int status = MV_CC_SetEnumValueByString(camera_handle_, feature.c_str(), value.c_str());
    if (status == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "%s set to %s", feature.c_str(), value.c_str());
      return true;
    }
    logSetFailure(feature, value, status, required);
    return !required;
  }

  bool setFloatFeatureChecked(const std::string & feature, double value, bool required)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    MVCC_FLOATVALUE float_value;
    int status = MV_CC_GetFloatValue(camera_handle_, feature.c_str(), &float_value);
    if (status != MV_OK) {
      logSetFailure(feature, std::to_string(value), status, required);
      return !required;
    }

    if (value < float_value.fMin || value > float_value.fMax) {
      RCLCPP_ERROR(this->get_logger(),
        "%s value %.6f is outside camera range [%.6f, %.6f]",
        feature.c_str(), value, float_value.fMin, float_value.fMax);
      return !required;
    }

    status = MV_CC_SetFloatValue(camera_handle_, feature.c_str(), static_cast<float>(value));
    if (status == MV_OK) {
      MVCC_FLOATVALUE actual_value;
      if (MV_CC_GetFloatValue(camera_handle_, feature.c_str(), &actual_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "%s requested %.6f, actual %.6f",
          feature.c_str(), value, actual_value.fCurValue);
      } else {
        RCLCPP_INFO(this->get_logger(), "%s set to %.6f", feature.c_str(), value);
      }
      return true;
    }

    logSetFailure(feature, std::to_string(value), status, required);
    return !required;
  }

  void logSetFailure(
    const std::string & feature, const std::string & value, int status, bool required)
  {
    if (required) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set %s to %s, SDK status = %s",
        feature.c_str(), value.c_str(), sdkStatusToHex(status).c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Optional feature %s could not be set to %s, SDK status = %s",
        feature.c_str(), value.c_str(), sdkStatusToHex(status).c_str());
    }
  }

  void logConfigurationSummary()
  {
    RCLCPP_INFO(this->get_logger(), "Camera acquisition mode: %s",
      trigger_mode_ ? "ExternalTrigger" : "FreeRun");
    RCLCPP_INFO(this->get_logger(), "Timestamp source: %s", timestamp_source_.c_str());
    RCLCPP_INFO(this->get_logger(), "Livox topic: %s", livox_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Sync queue size: %zu", sync_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Sync wait timeout: %ld ms", sync_wait_timeout_ms_);
    RCLCPP_INFO(this->get_logger(), "Max pairing host delta: %.3f ms",
      max_pairing_host_delta_ms_);
    RCLCPP_INFO(this->get_logger(), "Trigger source: %s", trigger_source_.c_str());
    RCLCPP_INFO(this->get_logger(), "Trigger activation: %s", trigger_activation_.c_str());
    RCLCPP_INFO(this->get_logger(), "Trigger delay: %.3f us", trigger_delay_us_);
    RCLCPP_INFO(this->get_logger(), "Acquisition frame rate enabled: %s",
      trigger_mode_ ? "false" : (acquisition_frame_rate_enable_ ? "true" : "false"));
    RCLCPP_INFO(this->get_logger(), "Acquisition frame rate: %.3f Hz", acquisition_frame_rate_);
    RCLCPP_INFO(this->get_logger(), "Exposure auto: %s", exposure_auto_.c_str());
    RCLCPP_INFO(this->get_logger(), "Exposure time: %.3f us", exposure_time_);
    RCLCPP_INFO(this->get_logger(), "Gain auto: %s", gain_auto_.c_str());
    RCLCPP_INFO(this->get_logger(), "Gain: %.3f", gain_);
    RCLCPP_INFO(this->get_logger(), "Pixel format: %s", pixel_format_.c_str());
    RCLCPP_INFO(this->get_logger(), "ADC bit depth: %s", adc_bit_depth_.c_str());
    RCLCPP_INFO(this->get_logger(), "Image scale: %.3f", image_scale_);
  }

  void logTimestampDiagnostics(
    const MV_FRAME_OUT & out_frame,
    const LivoxPairingResult & pairing_result,
    bool dropped)
  {
    if (!timestamp_diagnostics_ ||
      timestamp_source_mode_ != TimestampSourceMode::LivoxTimebase ||
      !livox_synchronizer_)
    {
      return;
    }

    const int64_t now_ns = steadyNowNs();
    if (!dropped && now_ns - last_timestamp_diag_ns_ < 3000000000LL) {
      return;
    }
    if (dropped && now_ns - last_timestamp_warn_ns_ < 3000000000LL) {
      return;
    }
    if (dropped) {
      last_timestamp_warn_ns_ = now_ns;
    } else {
      last_timestamp_diag_ns_ = now_ns;
    }

    const auto stats = livox_synchronizer_->stats();
    const double host_delta_ms = pairing_result.host_delta_ns / 1000000.0;
    RCLCPP_INFO(
      this->get_logger(),
      "timestamp diagnostics: frame=%u trigger=%u matched=%s livox=%ld "
      "host_delta=%.3fms queue=%zu matched_count=%llu dropped=%llu invalid=%llu "
      "duplicate=%llu rollback=%llu header_mismatch=%llu abnormal_interval=%llu",
      out_frame.stFrameInfo.nFrameNum,
      out_frame.stFrameInfo.nTriggerIndex,
      pairing_result.matched ? "true" : "false",
      pairing_result.sample.stamp_ns,
      host_delta_ms,
      pairing_result.queue_size,
      static_cast<unsigned long long>(stats.matched_frames),
      static_cast<unsigned long long>(stats.dropped_unsynced_frames),
      static_cast<unsigned long long>(stats.invalid_livox_stamps),
      static_cast<unsigned long long>(stats.duplicate_livox_stamps),
      static_cast<unsigned long long>(stats.rollback_livox_stamps),
      static_cast<unsigned long long>(stats.header_mismatch_stamps),
      static_cast<unsigned long long>(stats.abnormal_interval_stamps));
  }

  uint32_t scaledDimension(uint32_t value) const
  {
    return static_cast<uint32_t>(std::max(1.0, std::round(value * image_scale_)));
  }

  void applyCameraInfoScale()
  {
    if (image_scale_ == 1.0) {
      return;
    }

    camera_info_msg_.width = scaledDimension(camera_info_msg_.width);
    camera_info_msg_.height = scaledDimension(camera_info_msg_.height);
    camera_info_msg_.k[0] *= image_scale_;
    camera_info_msg_.k[2] *= image_scale_;
    camera_info_msg_.k[4] *= image_scale_;
    camera_info_msg_.k[5] *= image_scale_;
    camera_info_msg_.p[0] *= image_scale_;
    camera_info_msg_.p[2] *= image_scale_;
    camera_info_msg_.p[5] *= image_scale_;
    camera_info_msg_.p[6] *= image_scale_;
  }

  void freeImageBuffer(MV_FRAME_OUT & out_frame)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int status = MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
    if (status != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to free image buffer, status = %s",
        sdkStatusToHex(status).c_str());
    }
  }

  bool restartGrabbing()
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    int status = MV_CC_StopGrabbing(camera_handle_);
    if (status != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to stop grabbing for recovery, status = %s",
        sdkStatusToHex(status).c_str());
      return false;
    }
    status = MV_CC_StartGrabbing(camera_handle_);
    if (status != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to restart grabbing for recovery, status = %s",
        sdkStatusToHex(status).c_str());
      return false;
    }
    return true;
  }

  const std::map<std::string, std::string> auto_modes_ = {
    {"Off", "Off"},
    {"Once", "Once"},
    {"Continuous", "Continuous"},
  };
  const std::map<std::string, std::string> trigger_sources_ = {
    {"Line0", "Line0"},
    {"Line1", "Line1"},
    {"Software", "Software"},
  };
  const std::map<std::string, std::string> trigger_activations_ = {
    {"RisingEdge", "RisingEdge"},
    {"FallingEdge", "FallingEdge"},
  };
  enum class TimestampSourceMode
  {
    RosNow,
    LivoxTimebase,
  };

  void * camera_handle_ = nullptr;
  int n_ret_ = MV_OK;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::string camera_info_url_;
  std::string pixel_format_;
  std::string adc_bit_depth_;
  bool use_sensor_data_qos_ = true;
  double image_scale_ = 1.0;
  std::string camera_name_;
  std::string frame_id_;
  std::string camera_topic_;

  bool trigger_mode_ = false;
  std::string trigger_source_;
  std::string trigger_activation_;
  double trigger_delay_us_ = 0.0;
  std::string timestamp_source_ = "ros_now";
  TimestampSourceMode timestamp_source_mode_ = TimestampSourceMode::RosNow;
  std::string livox_topic_ = "/livox/lidar";
  std::size_t sync_queue_size_ = 30;
  int64_t sync_wait_timeout_ms_ = 80;
  double max_pairing_host_delta_ms_ = 50.0;
  bool timestamp_diagnostics_ = true;
  bool acquisition_frame_rate_enable_ = true;
  double acquisition_frame_rate_ = 165.0;
  std::string exposure_auto_;
  double exposure_time_ = 5000.0;
  std::string gain_auto_;
  double gain_ = 15.0;

  std::thread capture_thread_;
  std::atomic_bool running_{true};
  std::mutex sdk_mutex_;
  std::vector<uint8_t> convert_buffer_;
  std::unique_ptr<LivoxTimestampSynchronizer> livox_synchronizer_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  int64_t last_timestamp_diag_ns_ = 0;
  int64_t last_timestamp_warn_ns_ = 0;
  int fail_count_ = 0;
};
}  // namespace hik_camera_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera_ros2_driver::HikCameraRos2DriverNode)
