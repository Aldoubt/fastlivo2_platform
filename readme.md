# FAST-LIVO2 Platform 启动说明

本文档记录 MID360、海康相机和 FAST-LIVO2 算法的常用启动方式。以下命令默认在本仓库根目录执行：

```bash
cd /home/jetson/fastlivo2_platform
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

如果当前终端已经 source 过 `ros2_ws/install/setup.bash`，可以不用重复执行。

## 1. MID360 使用 RViz 启动

单独查看 MID360 点云时，使用 Livox 驱动自带的 RViz 启动文件：

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

该方式会启动：

- `livox_ros_driver2_node`
- `rviz2`

主要话题：

- `/livox/lidar`
- `/livox/imu`

说明：该 RViz 启动方式默认使用 `PointCloud2` 输出，适合查看点云。如果后续要给 FAST-LIVO2 算法使用，需要使用 `CustomMsg` 输出，见第 4 节。

## 2. 海康相机启动

普通连续采集模式：

```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py
```

FAST-LIVO2 硬件触发模式：

```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py \
  params_file:=/home/jetson/fastlivo2_platform/ros2_ws/src/hik_camera_ros2_driver/config/fast_livo2_trigger_params.yaml
```

主要话题：

- `/left_camera/image`
- `/left_camera/image/camera_info`

触发模式说明：

- 相机触发输入默认使用 `Line0`
- 触发沿默认使用 `RisingEdge`
- 没有外部触发脉冲时，相机不发布图像属于正常现象
- FAST-LIVO2 默认订阅 `/left_camera/image`

检查相机图像频率：

```bash
ros2 topic hz /left_camera/image
```

## 3. 传感器和 RViz 一键启动

如果只想同时启动 MID360、海康相机和 RViz，可以使用本仓库的 bringup：

```bash
ros2 launch fastlivo2_bringup sensors_rviz.launch.py
```

该启动文件会同时启动：

- MID360 Livox 驱动
- 海康相机驱动
- RViz

默认雷达输出格式为 `PointCloud2`，适合传感器联调和 RViz 观察。

如果需要关闭 RViz：

```bash
ros2 launch fastlivo2_bringup sensors_rviz.launch.py use_rviz:=false
```

## 4. FAST-LIVO2 算法总体启动

建议使用 3 个终端分别启动传感器和算法。

### 终端 1：启动 MID360，给算法输出 CustomMsg

```bash
cd /home/jetson/fastlivo2_platform
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

```

### 终端 2：启动海康相机

普通连续采集：

```bash
cd /home/jetson/fastlivo2_platform
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py
```

硬件触发采集：

```bash
cd /home/jetson/fastlivo2_platform
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py \
  params_file:=/home/jetson/fastlivo2_platform/ros2_ws/src/hik_camera_ros2_driver/config/fast_livo2_trigger_params.yaml
```

这个配置会启用 `timestamp_source: livox_timebase`。相机每收到一帧硬触发图像后，会等待最近一次 `/livox/lidar` 的 `livox_ros_driver2/msg/CustomMsg.timebase`，并把 `/left_camera/image.header.stamp` 改成同一个 Livox 时间基；匹配失败时直接丢帧，不回退到主机时间。普通 `camera_params.yaml` 仍使用 `timestamp_source: ros_now`，适合自由运行调试。

### 终端 3：启动 FAST-LIVO2 算法

启动算法和 RViz：

```bash
cd /home/jetson/fastlivo2_platform
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True
```

只启动算法，不启动 RViz：

```bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=False
```

FAST-LIVO2 默认订阅：

- `/livox/lidar`
- `/livox/imu`
- `/left_camera/image`

也可以用本仓库的一键启动文件同时启动 MID360、海康相机和 FAST-LIVO2：

```bash
ros2 launch fastlivo2_bringup mid360_hik_fastlivo2.launch.py use_rviz:=False
```

该启动文件会强制 Livox 使用 `xfer_format=1` 输出 `CustomMsg`，相机使用 `fast_livo2_trigger_params.yaml`，FAST-LIVO2 使用 `mid360_hik.yaml` 和 `camera_mid360_hik.yaml`。

## 5. 常用检查命令

查看节点：

```bash
ros2 node list
```

查看话题：

```bash
ros2 topic list -t
```

检查雷达频率：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

检查相机频率：

```bash
ros2 topic hz /left_camera/image
```

查看 FAST-LIVO2 是否启动：

```bash
ros2 node list | grep laserMapping
```

检查在线时间戳是否在同一时间基：

```bash
python3 tools/check_livo_time.py
```

重点看输出中的 `header-timebase`、`img-lidar_timebase` 和 `imu-lidar_timebase`。正常情况下，Livox 的 `header-timebase` 应接近 0；硬触发 FAST-LIVO2 模式下，相机 `img-lidar_timebase` 应在毫秒级触发/接收抖动范围内，而不应该再出现相机是当前系统时间、IMU 或雷达落到 2020 年这种跨时间基现象。
