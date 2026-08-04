from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")
    start_imx219 = LaunchConfiguration("start_imx219")
    camera0_topic = LaunchConfiguration("camera0_topic")

    camera_and_raw_streams = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("auv_vision_control"),
                    "launch",
                    "dual_imx219_h264.launch.py",
                ]
            )
        ),
        launch_arguments={
            "width": LaunchConfiguration("camera_width"),
            "height": LaunchConfiguration("camera_height"),
            "framerate": LaunchConfiguration("camera_framerate"),
            "camera0_sensor_id": LaunchConfiguration("camera0_sensor_id"),
            "camera0_flip_method": LaunchConfiguration("camera0_flip_method"),
            "camera1_sensor_id": LaunchConfiguration("camera1_sensor_id"),
            "camera1_flip_method": LaunchConfiguration("camera1_flip_method"),
            "camera0_topic": camera0_topic,
            "camera1_topic": LaunchConfiguration("camera1_topic"),
            "camera0_webrtc_port": LaunchConfiguration("camera0_webrtc_port"),
            "camera1_webrtc_port": LaunchConfiguration("camera1_webrtc_port"),
        }.items(),
        condition=IfCondition(start_imx219),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("auv_vision_control"), "config", "vision_control.yaml"]
                ),
                description="Shared YOLO, WebRTC, and lane-controller configuration.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("auv_vision_control"), "models", "best.pt"]
                ),
                description="Buoy-only Ultralytics model path.",
            ),
            DeclareLaunchArgument(
                "start_imx219",
                default_value="true",
                description="Start both IMX219 cameras and their raw H.264 WebRTC streams.",
            ),
            DeclareLaunchArgument("camera_width", default_value="1280"),
            DeclareLaunchArgument("camera_height", default_value="720"),
            DeclareLaunchArgument("camera_framerate", default_value="30"),
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
            DeclareLaunchArgument("camera0_webrtc_port", default_value="8091"),
            DeclareLaunchArgument("camera1_webrtc_port", default_value="8092"),
            camera_and_raw_streams,
            Node(
                package="auv_vision_control",
                executable="yolo_buoy_detector",
                name="yolo_buoy_detector",
                output="screen",
                parameters=[
                    config_file,
                    {"model_path": model_path, "image_topic": camera0_topic},
                ],
            ),
            Node(
                package="auv_vision_control",
                executable="lane_vision_controller_node",
                name="lane_vision_controller_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
