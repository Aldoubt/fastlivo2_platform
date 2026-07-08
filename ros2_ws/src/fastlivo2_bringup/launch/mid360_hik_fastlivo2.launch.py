from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    camera_params_file = LaunchConfiguration("camera_params_file")
    lidar_config_file = LaunchConfiguration("lidar_config_file")
    fast_livo_params_file = LaunchConfiguration("fast_livo_params_file")
    fast_livo_camera_params_file = LaunchConfiguration("fast_livo_camera_params_file")
    lidar_publish_freq = LaunchConfiguration("lidar_publish_freq")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")

    default_camera_params = PathJoinSubstitution([
        FindPackageShare("hik_camera_ros2_driver"),
        "config",
        "fast_livo2_trigger_params.yaml",
    ])
    default_lidar_config = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"),
        "config",
        "MID360_config.json",
    ])
    default_fast_livo_params = PathJoinSubstitution([
        FindPackageShare("fast_livo"),
        "config",
        "mid360_hik.yaml",
    ])
    default_fast_livo_camera_params = PathJoinSubstitution([
        FindPackageShare("fast_livo"),
        "config",
        "camera_mid360_hik.yaml",
    ])
    mapping_launch = PathJoinSubstitution([
        FindPackageShare("fast_livo"),
        "launch",
        "mapping_aviz.launch.py",
    ])

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "xfer_format": 1,
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

    fast_livo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        launch_arguments={
            "use_rviz": use_rviz,
            "avia_params_file": fast_livo_params_file,
            "camera_params_file": fast_livo_camera_params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="False",
            description="Start FAST-LIVO2 RViz.",
        ),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=default_camera_params,
            description="Hikvision trigger-mode parameter YAML.",
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
            "lidar_publish_freq",
            default_value="10.0",
            description="Livox CustomMsg publish frequency for FAST-LIVO2.",
        ),
        DeclareLaunchArgument(
            "lidar_frame_id",
            default_value="livox_frame",
            description="Frame id used by Livox messages.",
        ),
        DeclareLaunchArgument(
            "fast_livo_params_file",
            default_value=default_fast_livo_params,
            description="FAST-LIVO2 MID360/Hikvision parameter YAML.",
        ),
        DeclareLaunchArgument(
            "fast_livo_camera_params_file",
            default_value=default_fast_livo_camera_params,
            description="FAST-LIVO2 camera intrinsic parameter YAML.",
        ),
        livox_driver,
        hik_camera,
        fast_livo,
    ])
