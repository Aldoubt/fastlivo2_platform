from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    camera_params_file = LaunchConfiguration("camera_params_file")
    lidar_config_file = LaunchConfiguration("lidar_config_file")

    lidar_xfer_format = LaunchConfiguration("lidar_xfer_format")
    lidar_publish_freq = LaunchConfiguration("lidar_publish_freq")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")

    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("fastlivo2_bringup"),
        "rviz",
        "sensors.rviz",
    ])
    default_camera_params = PathJoinSubstitution([
        FindPackageShare("hik_camera_ros2_driver"),
        "config",
        "camera_params.yaml",
    ])
    default_lidar_config = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"),
        "config",
        "MID360_config.json",
    ])

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "xfer_format": ParameterValue(lidar_xfer_format, value_type=int),
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": ParameterValue(lidar_publish_freq, value_type=float),
            "output_data_type": 0,
            "frame_id": lidar_frame_id,
            "lvx_file_path": "/tmp/livox_test.lvx",
            "user_config_path": lidar_config_file,
            "cmdline_input_bd_code": "livox0000000001",
        }],
    )

    hik_camera = Node(
        package="hik_camera_ros2_driver",
        executable="hik_camera_ros2_driver_node",
        name="hik_camera_ros2_driver",
        output="screen",
        parameters=[camera_params_file],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("camera_log_level")],
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz for live sensor visualization.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config file.",
        ),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=default_camera_params,
            description="Hikvision camera parameter YAML.",
        ),
        DeclareLaunchArgument(
            "camera_log_level",
            default_value="info",
            description="Log level for the Hikvision camera node.",
        ),
        DeclareLaunchArgument(
            "lidar_config_file",
            default_value=default_lidar_config,
            description="Livox MID-360 JSON config.",
        ),
        DeclareLaunchArgument(
            "lidar_xfer_format",
            default_value="0",
            description="Livox output format: 0=PointCloud2 for RViz, 1=CustomMsg for FAST-LIVO2.",
        ),
        DeclareLaunchArgument(
            "lidar_publish_freq",
            default_value="10.0",
            description="Livox publish frequency.",
        ),
        DeclareLaunchArgument(
            "lidar_frame_id",
            default_value="livox_frame",
            description="Frame id used by Livox point cloud messages.",
        ),
        livox_driver,
        hik_camera,
        rviz,
    ])
