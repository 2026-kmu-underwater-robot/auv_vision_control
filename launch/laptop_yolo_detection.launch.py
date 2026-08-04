from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/imx219/camera0/image_raw/compressed",
                description="Compressed IMX219 camera topic used for YOLO inference.",
            ),
            DeclareLaunchArgument(
                "bbox_topic",
                default_value="/vision/buoy_bbox",
                description="BBox topic published back to the AUV NUC.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("auv_vision_control"), "models", "best.pt"]
                ),
                description="Required .pt model path. Example: /home/user/models/yolo26m_underwater_batch4_last.pt",
            ),
            DeclareLaunchArgument(
                "target_class_name",
                default_value="",
                description="Target class name. Leave empty to accept every detected class.",
            ),
            DeclareLaunchArgument(
                "target_class_id",
                default_value="0",
                description="Target class id. Overrides target_class_name when >= 0.",
            ),
            DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument(
                "device",
                default_value="auto",
                description="Inference device: auto, cpu, cuda:0, etc.",
            ),
            DeclareLaunchArgument("imgsz", default_value="640"),
            DeclareLaunchArgument(
                "show_preview",
                default_value="true",
                description="Show OpenCV preview window with live detections.",
            ),
            DeclareLaunchArgument(
                "preview_window_name",
                default_value="YOLO Buoy Detection",
                description="OpenCV window title for the preview UI.",
            ),
            DeclareLaunchArgument(
                "publish_per_class",
                default_value="false",
                description="Publish one best buoy detection per frame.",
            ),
            DeclareLaunchArgument(
                "webrtc_enabled",
                default_value="true",
                description="Enable the default NVENC H.264 WebRTC browser transport.",
            ),
            DeclareLaunchArgument("webrtc_bind_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("webrtc_port", default_value="8090"),
            DeclareLaunchArgument("webrtc_bitrate", default_value="2000000"),
            DeclareLaunchArgument("webrtc_iframe_interval", default_value="10"),
            DeclareLaunchArgument("webrtc_max_fps", default_value="30.0"),
            Node(
                package="auv_vision_control",
                executable="yolo_buoy_detector",
                name="yolo_buoy_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "bbox_topic": LaunchConfiguration("bbox_topic"),
                        "model_path": LaunchConfiguration("model_path"),
                        "target_class_name": LaunchConfiguration("target_class_name"),
                        "target_class_id": ParameterValue(LaunchConfiguration("target_class_id"), value_type=int),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "device": LaunchConfiguration("device"),
                        "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
                        "show_preview": ParameterValue(LaunchConfiguration("show_preview"), value_type=bool),
                        "preview_window_name": LaunchConfiguration("preview_window_name"),
                        "publish_per_class": ParameterValue(
                            LaunchConfiguration("publish_per_class"), value_type=bool
                        ),
                        "webrtc_enabled": ParameterValue(
                            LaunchConfiguration("webrtc_enabled"), value_type=bool
                        ),
                        "webrtc_bind_address": LaunchConfiguration(
                            "webrtc_bind_address"
                        ),
                        "webrtc_port": ParameterValue(
                            LaunchConfiguration("webrtc_port"), value_type=int
                        ),
                        "webrtc_bitrate": ParameterValue(
                            LaunchConfiguration("webrtc_bitrate"), value_type=int
                        ),
                        "webrtc_iframe_interval": ParameterValue(
                            LaunchConfiguration("webrtc_iframe_interval"),
                            value_type=int,
                        ),
                        "webrtc_max_fps": ParameterValue(
                            LaunchConfiguration("webrtc_max_fps"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
