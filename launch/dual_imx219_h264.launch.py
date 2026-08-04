from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _streamer(index: int) -> Node:
    prefix = f"camera{index}"
    return Node(
        package="auv_vision_control",
        executable="ros_image_webrtc_streamer",
        name=f"imx219_{prefix}_webrtc",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration(f"{prefix}_topic"),
                "bind_address": LaunchConfiguration("webrtc_bind_address"),
                "port": ParameterValue(
                    LaunchConfiguration(f"{prefix}_webrtc_port"), value_type=int
                ),
                "bitrate": ParameterValue(
                    LaunchConfiguration("webrtc_bitrate"), value_type=int
                ),
                "iframe_interval": ParameterValue(
                    LaunchConfiguration("webrtc_iframe_interval"), value_type=int
                ),
                "max_fps": ParameterValue(
                    LaunchConfiguration("webrtc_max_fps"), value_type=float
                ),
            }
        ],
    )


def generate_launch_description() -> LaunchDescription:
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("auv_imx219_camera"),
                    "launch",
                    "dual_imx219.launch.py",
                ]
            )
        ),
        launch_arguments={
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "framerate": LaunchConfiguration("framerate"),
            "camera0_sensor_id": LaunchConfiguration("camera0_sensor_id"),
            "camera0_flip_method": LaunchConfiguration("camera0_flip_method"),
            "camera1_sensor_id": LaunchConfiguration("camera1_sensor_id"),
            "camera1_flip_method": LaunchConfiguration("camera1_flip_method"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="720"),
            DeclareLaunchArgument("framerate", default_value="30"),
            DeclareLaunchArgument("camera0_sensor_id", default_value="0"),
            DeclareLaunchArgument("camera0_flip_method", default_value="0"),
            DeclareLaunchArgument("camera1_sensor_id", default_value="1"),
            DeclareLaunchArgument("camera1_flip_method", default_value="0"),
            DeclareLaunchArgument(
                "camera0_topic",
                default_value="/imx219/camera0/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "camera1_topic",
                default_value="/imx219/camera1/image_raw/compressed",
            ),
            DeclareLaunchArgument("webrtc_bind_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("camera0_webrtc_port", default_value="8091"),
            DeclareLaunchArgument("camera1_webrtc_port", default_value="8092"),
            DeclareLaunchArgument("webrtc_bitrate", default_value="2000000"),
            DeclareLaunchArgument("webrtc_iframe_interval", default_value="10"),
            DeclareLaunchArgument("webrtc_max_fps", default_value="30.0"),
            camera_launch,
            _streamer(0),
            _streamer(1),
        ]
    )
