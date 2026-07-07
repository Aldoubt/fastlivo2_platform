#!/usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


def load_pointcloud2_from_bag(
    bag_path: str,
    topic_name: str,
    storage_id: str,
    frame_stride: int,
) -> np.ndarray:
    """从 ROS2 bag 中读取并合并 PointCloud2 的 XYZ 点。"""

    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id=storage_id,
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="",
        output_serialization_format="",
    )

    reader.open(storage_options, converter_options)

    topic_type_map = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    if topic_name not in topic_type_map:
        available_topics = "\n".join(
            f"  {name}: {msg_type}"
            for name, msg_type in topic_type_map.items()
        )
        raise RuntimeError(
            f"未找到话题 {topic_name}\n"
            f"当前 bag 中的话题：\n{available_topics}"
        )

    message_type_name = topic_type_map[topic_name]

    if message_type_name != "sensor_msgs/msg/PointCloud2":
        raise TypeError(
            f"{topic_name} 的消息类型为 {message_type_name}，"
            "当前脚本仅支持 sensor_msgs/msg/PointCloud2"
        )

    reader.set_filter(
        rosbag2_py.StorageFilter(topics=[topic_name])
    )

    message_type = get_message(message_type_name)

    xyz_blocks = []
    frame_index = 0

    while reader.has_next():
        current_topic, serialized_data, _ = reader.read_next()

        if current_topic != topic_name:
            continue

        if frame_index % frame_stride != 0:
            frame_index += 1
            continue

        msg = deserialize_message(serialized_data, message_type)

        points = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )

        frame_xyz = np.asarray(
            [[float(p[0]), float(p[1]), float(p[2])] for p in points],
            dtype=np.float64,
        )

        if frame_xyz.size > 0:
            xyz_blocks.append(frame_xyz)

        frame_index += 1

    if not xyz_blocks:
        raise RuntimeError("bag 中没有读取到有效 XYZ 点云")

    return np.concatenate(xyz_blocks, axis=0)


def select_roi_points(
    xyz: np.ndarray,
    voxel_size: float,
) -> np.ndarray:
    """使用 Open3D 交互选取标定板周围的点。"""

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)

    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size)

    print("操作方法：")
    print("1. 按住 Shift 并单击，选择标定板边缘或角点")
    print("2. 建议选择标定板四周至少 4 个点")
    print("3. 按 Q 关闭窗口并计算范围")

    visualizer = o3d.visualization.VisualizerWithEditing()
    visualizer.create_window(
        window_name="FAST-Calib ROS2 Distance Filter Tool"
    )
    visualizer.add_geometry(cloud)
    visualizer.run()
    visualizer.destroy_window()

    picked_indices = visualizer.get_picked_points()

    if len(picked_indices) < 4:
        raise RuntimeError(
            f"至少需要选择 4 个点，当前只选择了 {len(picked_indices)} 个"
        )

    cloud_xyz = np.asarray(cloud.points)

    # 使用全部选点，而不是原版工具中的前四个点
    return cloud_xyz[picked_indices]


def calculate_bounds(
    selected_xyz: np.ndarray,
    margin: float,
) -> tuple[np.ndarray, np.ndarray]:
    """根据选点计算 XYZ 最小值和最大值。"""

    lower = np.min(selected_xyz, axis=0) - margin
    upper = np.max(selected_xyz, axis=0) + margin

    return lower, upper


def print_ros2_yaml(lower: np.ndarray, upper: np.ndarray) -> None:
    """输出可以直接写入 qr_params.yaml 的 ROS2 参数。"""

    print("\n建议的 ROS2 参数：\n")
    print("fast_calib:")
    print("  ros__parameters:")
    print(f"    x_min: {lower[0]:.3f}")
    print(f"    x_max: {upper[0]:.3f}")
    print(f"    y_min: {lower[1]:.3f}")
    print(f"    y_max: {upper[1]:.3f}")
    print(f"    z_min: {lower[2]:.3f}")
    print(f"    z_max: {upper[2]:.3f}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="FAST-Calib ROS2 XYZ 点云过滤范围选取工具"
    )

    parser.add_argument(
        "bag_path",
        type=str,
        help="ROS2 bag 数据目录",
    )

    parser.add_argument(
        "--topic",
        type=str,
        default="/livox/lidar",
        help="PointCloud2 话题名称",
    )

    parser.add_argument(
        "--storage-id",
        type=str,
        default="sqlite3",
        choices=("sqlite3", "mcap"),
        help="rosbag2 存储类型",
    )

    parser.add_argument(
        "--margin",
        type=float,
        default=0.2,
        help="XYZ 每个方向增加的裕量，单位 m",
    )

    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.02,
        help="显示前的体素降采样尺寸，单位 m",
    )

    parser.add_argument(
        "--frame-stride",
        type=int,
        default=5,
        help="每隔多少帧读取一帧，避免内存占用过高",
    )

    args = parser.parse_args()

    if not Path(args.bag_path).exists():
        raise FileNotFoundError(f"bag 路径不存在：{args.bag_path}")

    if args.frame_stride < 1:
        raise ValueError("frame_stride 必须大于等于 1")

    xyz = load_pointcloud2_from_bag(
        bag_path=args.bag_path,
        topic_name=args.topic,
        storage_id=args.storage_id,
        frame_stride=args.frame_stride,
    )

    print(f"读取点数：{xyz.shape[0]}")

    selected_xyz = select_roi_points(
        xyz=xyz,
        voxel_size=args.voxel_size,
    )

    lower, upper = calculate_bounds(
        selected_xyz=selected_xyz,
        margin=args.margin,
    )

    print_ros2_yaml(lower, upper)


if __name__ == "__main__":
    main()
    