[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/SMBU-PolarBear-Robotics-Team/hik_camera_ros2_driver/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/hik_camera_ros2_driver/actions/workflows/ci.yml)

# hik_camera_ros2_driver

## Overview

The `hik_camera_ros2_driver` package provides a ROS 2 driver for controlling and interfacing with Hikvision cameras. It supports functionalities such as camera initialization, parameter configuration, and image publishing. This package is intended for applications requiring reliable and configurable image data acquisition in a ROS 2 environment.

### Executables

The package includes the `hik_camera_node`, which manages the camera and publishes image data along with camera information to ROS 2 topics.

### Subscribed Topics

None.

### Published Topics

- `<camera_topic>` (sensor_msgs/msg/Image)
  - The image data captured by the Hikvision camera.

- `<camera_topic>/camera_info` (sensor_msgs/msg/CameraInfo)
  - Camera calibration information.

### Parameters

- `trigger_mode` (bool, default: `false`)
  - `false` for normal free-run acquisition; `true` for external hardware trigger mode.

- `trigger_source` (string, default: `Line0`)
  - Trigger input source in trigger mode. Supported values in this driver are `Line0`, `Line1`, and `Software`.

- `trigger_activation` (string, default: `RisingEdge`)
  - Trigger edge. Supported values are `RisingEdge` and `FallingEdge`.

- `trigger_delay_us` (double, default: `0.0`)
  - Trigger delay in microseconds. If the camera does not support `TriggerDelay`, startup prints a warning and continues.

- `acquisition_frame_rate_enable` (bool, default: `true`)
  - Enables the camera internal frame-rate limiter in free-run mode. In trigger mode this is forced off.

- `acquisition_frame_rate` (double, default: `165`)
  - The acquisition frame rate in Hz for free-run mode. It does not control the actual frame rate in external trigger mode.

- `exposure_auto` (string, default: `Off`)
  - Exposure mode: `Off`, `Once`, or `Continuous`. `exposure_time` is written only when this is `Off`.

- `exposure_time` (double, default: `5000`)
  - The camera exposure time in microseconds.

- `gain_auto` (string, default: `Off`)
  - Gain mode: `Off`, `Once`, or `Continuous`. `gain` is written only when this is `Off`.

- `gain` (double, default: `15.0`)
  - The gain setting for the camera.

- `pixel_format` (string, default: `RGB8Packed`)
  - The pixel format for the image data. Supported values: `Mono8`, `Mono10`, `Mono12`, `RGB8Packed`, `BGR8Packed`, `YUV422_YUYV_Packed`, `YUV422Packed`, `BayerRG8`, `BayerRG10`, `BayerRG10Packed`, `BayerRG12`, `BayerRG12Packed`.

- `adc_bit_depth` (string, default: `Bits_8`)
  - The ADC bit depth for the camera. Supported values: `Bits_8`, `Bits_12`.

- `use_sensor_data_qos` (bool, default: true)
  - Whether to use the `sensor_data` QoS profile for image topic publication.

- `camera_name` (string, default: `camera`)
  - The name of the camera for identification purposes.

- `frame_id` (string, default: `<camera_name>_optical_frame`)
  - The frame_id assigned to the published image data.

- `camera_topic` (string, default: `<camera_name>/image`)
  - The topic name for publishing image and info data.

- `camera_info_url` (string, default: `package://hik_camera_ros2_driver/config/camera_info.yaml`)
  - The URL for the camera calibration information file.

### Usage

#### Installation

To use this package, build it from source or include it in your ROS 2 workspace. Ensure that all dependencies are installed. You **don't** need to install the Hikvision camera SDK and include its libraries in your environment.

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/hik_camera_ros2_driver.git
```

```bash
cd ~/ros_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Run

You can use the provided launch file for starting the camera node with default or custom parameters:

```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py
```

#### Free-Run Mode

The default `config/camera_params.yaml` keeps the camera in normal free-run mode:

```yaml
trigger_mode: false
acquisition_frame_rate_enable: true
acquisition_frame_rate: 200.0
exposure_auto: "Off"
exposure_time: 5000.0
gain_auto: "Off"
gain: 12.0
```

In this mode the camera publishes continuously. If `acquisition_frame_rate_enable` is true, `acquisition_frame_rate` is applied to the camera.

#### FAST-LIVO2 External Trigger Mode

FAST-LIVO2 hardware-synchronized setups should use an external trigger input, typically wired to the camera Line0 according to the electrical specification of the exact Hikvision camera model.

```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py \
  params_file:=/home/jetson/fastlivo2_platform/ros2_ws/src/hik_camera_ros2_driver/config/fast_livo2_trigger_params.yaml
```

The example config uses:

```yaml
trigger_mode: true
trigger_source: "Line0"
trigger_activation: "RisingEdge"
trigger_delay_us: 0.0
acquisition_frame_rate_enable: false
acquisition_frame_rate: 10.0
exposure_auto: "Off"
exposure_time: 5000.0
gain_auto: "Continuous"
gain: 15.0
use_sensor_data_qos: false
```

In trigger mode, the camera internal frame-rate limiter is forced off. `acquisition_frame_rate` remains a parameter for free-run compatibility, but it does not decide the actual image rate. The ROS image publication rate should follow the external trigger pulse rate. If there is no external trigger pulse, receiving no images is normal.

FAST-LIVO2 subscribes to the image topic with default ROS 2 QoS in this workspace, so the trigger example keeps `use_sensor_data_qos: false`. Setting it to `true` may make the image publisher use sensor-data best-effort QoS, which can be incompatible with FAST-LIVO2's default reliable subscription and result in no images being received.

FAST-LIVO2 is generally better served by fixed exposure, because direct image alignment depends on stable photometric behavior. Auto gain may be used; when `gain_auto` is `Once` or `Continuous`, the fixed `gain` value is kept only as a standby value for when auto gain is disabled.

#### Runtime Parameter Updates

Only fixed exposure and fixed gain are safely updated while grabbing:

```bash
ros2 param set /hik_camera_ros2_driver exposure_time 5000.0
ros2 param set /hik_camera_ros2_driver gain 15.0
```

These updates are accepted only when `exposure_auto: "Off"` and `gain_auto: "Off"` respectively. Acquisition-mode parameters such as `trigger_mode`, `trigger_source`, `trigger_activation`, `pixel_format`, `adc_bit_depth`, and `acquisition_frame_rate_enable` are startup-only in this driver.

Check parameters and frame rate with:

```bash
ros2 param list /hik_camera_ros2_driver
ros2 param get /hik_camera_ros2_driver trigger_mode
ros2 topic hz /left_camera/image
```

#### Timestamp Note

`Image.header.stamp` and the paired `CameraInfo.header.stamp` are assigned from the ROS host clock near image retrieval/publication. This preserves the original driver behavior, but it is not a true external-trigger timestamp and is not a complete hardware time-synchronization solution. The driver does not convert camera hardware ticks into ROS time.
