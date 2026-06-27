# FAST-LIVO2 边缘部署项目说明

## 1. 项目定位

本项目面向 Jetson Orin 平台，目标是构建一套可快速部署、可迁移、可现场验证的三维扫描与导航原型系统。

核心用途：

- 三维扫描仪边缘部署验证
- LiDAR + IMU + 相机融合建图验证
- rosbag2 数据采集、回放与算法调试
- 后续接入 Nav2，验证定位、建图与速度指令输出链路

## 2. 当前平台与软件栈

### 2.1 硬件平台

- Jetson Orin
- Livox MID-360 / Livox 系列雷达
- 海康相机
- 可选小显示器，用于现场直观看图
- 可选远程电脑，用于 NoMachine 连接、监控与调试

### 2.2 系统与 ROS 环境

- Ubuntu 22.04
- ROS 2 Humble
- NoMachine 远程连接与自启动

### 2.3 已接入或已放入仓库的核心组件

| 模块 | 路径 | 作用 | 当前状态 |
| --- | --- | --- | --- |
| `livox_ros_driver2` | `ros2_ws/src/livox_ros_driver2` | Livox 雷达与 IMU 数据发布 | 已接入，需现场持续验证 |
| `hik_camera_ros2_driver` | `ros2_ws/src/hik_camera_ros2_driver` | 海康相机图像发布 | 已接入，需现场持续验证 |
| `FAST-LIVO2` / `fast_livo` | `ros2_ws/src/FAST-LIVO2` | LiDAR-Inertial-Visual 建图算法 | 已接入 |
| `rpg_vikit_ros2_fisheye` | `ros2_ws/src/rpg_vikit_ros2_fisheye` | FAST-LIVO2 依赖的视觉/相机模型库 | 已接入 |
| `Sophus` | `ros2_ws/src/Sophus` | 李群数学库依赖 | 已接入 |
| 时间戳检查工具 | `tools/check_livo_time.py` | 检查雷达、IMU、相机话题时间差 | 已编写，待实测记录 |
| Livox SDK2 | `third_party/Livox-SDK2` | Livox 底层 SDK 与样例 | 已放入仓库 |

## 3. 目标工作模式

### 模式 A：Jetson 采集与存储，电脑远程查看

Jetson 主要负责：

- 启动雷达、IMU、相机驱动
- 录制 rosbag2 数据
- 运行基础健康检查脚本
- 通过 NoMachine / SSH 供电脑端查看状态

电脑主要负责：

- 远程监控话题与节点状态
- 查看 RViz / 建图效果
- 分析 rosbag2 数据
- 调试参数、外参与时间同步问题

### 模式 B：Jetson 外接小屏，现场直观看图

Jetson 主要负责：

- 本机启动完整传感器链路
- 本机启动 FAST-LIVO2
- 本机启动 RViz 或轻量化可视化界面
- 现场观察建图质量、漂移情况与运行负载

适用场景：

- 现场快速演示
- 无电脑或弱网络环境
- 快速判断传感器、外参、时间同步是否可用

## 4. 后续导航接口预留

未来接入 Nav2 或其他导航模块时，需要预留以下能力：

- 快速建图与地图保存
- 定位结果输出到统一坐标系
- 路径规划接口
- 速度指令输出节点
- 与上位机、底盘或其他设备的通信节点
- `tf` 树、外参和时间同步的一致性检查

## 5. 当前完成情况核查

### 5.1 已完成

- Jetson Orin 已完成 NoMachine 接入，并设置自启动。
- 仓库中已包含 `livox_ros_driver2`。
- 仓库中已包含海康相机 ROS 2 驱动。
- 仓库中已包含 FAST-LIVO2 ROS 2 版本代码。
- 仓库中已包含 FAST-LIVO2 所需的 `vikit`、`Sophus` 等依赖。
- 初步验证过传感器话题和数据发布正常。
- 已编写 `tools/check_livo_time.py`，可用于查看 `/livox/lidar`、`/livox/imu`、`/camera/image` 三类话题的时间差。

### 5.2 需要实测确认

- 时间同步精度是否满足 FAST-LIVO2 稳定运行要求。
- 相机图像话题名是否与 FAST-LIVO2 配置完全一致。
- 雷达、IMU、相机三类话题在长时间运行下是否稳定。
- FAST-LIVO2 在 Jetson Orin 上的实时性能、CPU/GPU/内存占用。
- 外参、内参是否已经达到可稳定建图的水平。
- rosbag2 录制与回放是否覆盖完整调试链路。
- 外接小显示器场景下 RViz / 可视化是否流畅。
- 当前 `install/` 目录可能不是最新构建结果，需重新 `colcon build` 后确认 launch/config 是否完整安装。

### 5.3 尚未完成或未在仓库中发现

- FAST-Calib ROS 2 外参标定流程尚未接入当前仓库。
- 面向当前设备的统一启动脚本尚未整理。
- rosbag2 采集脚本、数据目录规范、回放命令尚未整理。
- 面向迁移部署的安装文档、依赖安装步骤、环境变量说明尚未整理。
- Nav2 定位、建图、速度输出链路尚未接入。
- 通信节点仍处于预留阶段。

## 6. 近期计划

### P0：时间同步与话题一致性

目标：确认雷达、IMU、相机数据流能被 FAST-LIVO2 稳定消费。

待完成：

- 运行 `tools/check_livo_time.py`，记录 `img-lidar`、`imu-lidar` 的时间差。
- 检查 `/livox/lidar`、`/livox/imu`、`/camera/image` 是否与实际驱动发布话题一致。
- 对比 FAST-LIVO2 配置文件中的订阅话题、frame_id、时间戳设置。

验收标准：

- 三类话题连续发布，无明显丢帧或断流。
- 时间差稳定，波动范围可记录、可复现。
- FAST-LIVO2 能正常订阅并进入建图流程。

### P1：rosbag2 采集与回放链路

目标：形成可复现实验数据，方便离线调试时间同步、外参与建图效果。

待完成：

- 编写常用 rosbag2 录制命令。
- 数据统一保存到仓库根目录下的 `data/bags/`。
- 记录最小必需话题列表。
- 验证 rosbag2 回放时 FAST-LIVO2 能正常运行。

建议最小话题：

```bash
/livox/lidar
/livox/imu
/camera/image
/camera/camera_info
/tf
/tf_static
```

### P2：外参与内参标定

目标：建立当前雷达与相机组合的可复用标定流程。

计划接入：

- FAST-Calib ROS 2 外参标定算法
- 当前项目计划参考仓库：https://github.com/ichangjian/FAST-Calib-ROS2.git

待完成：

- 接入或记录 FAST-Calib ROS 2 的安装方式。
- 明确标定数据采集方法。
- 输出相机内参、雷达到相机外参。
- 将标定结果写入 FAST-LIVO2 配置文件。
- 用现场数据验证建图质量。

### P3：仓库初始化与迁移部署

目标：让未来迁移到其他 Jetson 时可按文档快速复现。

待完成：

- 增加根目录 `README.md`。
- 增加 `.gitignore`，排除 `build/`、`install/`、`log/`、大体积 rosbag 数据。
- 记录依赖安装步骤。
- 记录构建命令与启动命令。
- 整理设备 IP、话题名、frame_id、配置文件位置。

### P4：联调与验收

目标：从传感器启动到建图显示形成一条闭环流程。

待完成：

- 一键或半自动启动雷达、相机、FAST-LIVO2、RViz。
- 记录 NoMachine 远程模式验证结果。
- 记录小显示器本机显示模式验证结果。
- 形成一次完整 rosbag2 采集、回放、建图测试记录。

## 7. 常用命令草案

以下命令默认从仓库根目录执行。当前设备上的仓库根目录是 `/home/jetson/fastlivo2_platform`，迁移到其他 Jetson 时只需要进入新的仓库根目录即可。

### 7.1 查看工作空间包

```bash
cd ros2_ws
colcon list
```

### 7.2 构建工作空间

```bash
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd ..
```

### 7.3 启动 Livox MID-360 驱动

```bash
source ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 7.4 启动海康相机

```bash
source ros2_ws/install/setup.bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py
```

### 7.5 检查时间戳差异

```bash
source ros2_ws/install/setup.bash
python3 tools/check_livo_time.py
```

### 7.6 启动 FAST-LIVO2

```bash
source ros2_ws/install/setup.bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True
```

### 7.7 录制最小 rosbag2

`ros2 bag record` 默认会把数据写入当前终端所在目录。为了避免 rosbag 散落在工作空间或源码目录中，本项目统一使用 `-o data/bags/<bag_name>` 指定输出目录。

```bash
mkdir -p data/bags
ros2 bag record \
  -o data/bags/livo_$(date +%Y%m%d_%H%M%S) \
  /livox/lidar \
  /livox/imu \
  /camera/image \
  /camera/camera_info \
  /tf \
  /tf_static
```

录制结果会保存在仓库根目录的 `data/bags/` 下；在当前设备上对应 `/home/jetson/fastlivo2_platform/data/bags/`。

说明：

- 如果 `ros2 launch livox_ros_driver2 msg_MID360_launch.py` 找不到文件，优先检查 `ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/`。
- 如果 `fast_livo` 或 `hik_camera_ros2_driver` 的 launch 文件未出现在 `install/` 目录中，先重新构建工作空间并重新 `source install/setup.bash`。

## 8. 后续研究方向

- 多相机 / 鱼眼相机与 MID-360 的外参标定。
- 不同相机模型对 FAST-LIVO2 建图质量的影响。
- 人机交互界面优化，减少现场调试命令量。
- 数据采集、参数对比、建图效果评估流程标准化。
- 边缘设备运行负载优化。

## 9. 下一步建议

优先顺序建议：

1. 先验证时间同步和话题一致性。
2. 再录制一组最小 rosbag2 数据，确保调试可复现。
3. 接入 FAST-Calib 或整理现有标定结果。
4. 调通 FAST-LIVO2 在线建图。
5. 最后整理一键启动、迁移部署和 Nav2 接口预留。
