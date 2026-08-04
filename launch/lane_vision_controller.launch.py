"""Launch the lane vision controller with every ROS parameter exposed."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _argument(name, default, description):
    if isinstance(default, bool):
        default_text = "true" if default else "false"
    else:
        default_text = str(default)
    return DeclareLaunchArgument(
        name,
        default_value=default_text,
        description=description,
    )


def generate_launch_description():
    # (parameter name, default value, ROS value type, launch argument description)
    parameter_specs = [
        # Topics
        ("bbox_topic", "/vision/buoy_bbox", str, "Buoy bounding-box input topic."),
        ("depth_topic", "/auv/depth", str, "Fallback scalar depth input topic."),
        ("depth_pose_topic", "/depth/pose", str, "Primary depth pose input topic; empty disables it."),
        ("odometry_topic", "/odometry/filtered", str, "Filtered odometry input topic."),
        ("start_frame_topic", "/start_frame", str, "Arena start-frame input topic."),
        ("vision_search_request_topic", "/homing/vision_search_active", str, "Hydrophone-to-vision search request topic."),
        ("target_confirmed_topic", "/vision/target_confirmed", str, "Vision target confirmation output topic."),
        ("vision_control_granted_topic", "/homing/vision_control_granted", str, "Hydrophone control-grant input topic."),
        ("emergency_stop_topic", "/mission/emergency_stop", str, "Emergency-stop input topic."),
        ("state_topic", "/mission/state", str, "Mission-state output topic."),
        ("rc_override_topic", "/mavros/rc/override", str, "MAVROS RC override output topic."),
        ("rc_monitor_topic", "/mission/rc_command", str, "RC command monitoring topic."),

        # Arena and lane generation
        ("arena_length_m", 15.0, float, "Arena length in metres."),
        ("arena_width_m", 16.0, float, "Arena width in metres."),
        ("arena_offset_x_m", -0.3, float, "Arena X offset from the start frame."),
        ("arena_offset_y_m", 0.3, float, "Arena Y offset from the start frame."),
        ("arena_safety_margin_m", 0.5, float, "Safety margin inside the arena boundary."),
        ("arena_start_corner", "bottom_left", str, "Arena orientation: bottom_left or bottom_right."),
        ("lane_search_offset_m", 2.0, float, "Lane coverage half-width and target cross-track limit."),
        ("incapable_skip_distance_m", 2.0, float, "Distance to ignore a failed target after returning to a lane."),
        ("waypoint_reach_tolerance_m", 0.15, float, "Waypoint arrival tolerance in metres."),

        # Input timing, depth conversion, and target selection
        ("control_rate_hz", 20.0, float, "Controller update rate."),
        ("odometry_timeout_sec", 0.5, float, "Odometry timeout before FAILSAFE."),
        ("detection_timeout_sec", 1.0, float, "Bounding-box freshness timeout."),
        ("depth_timeout_sec", 1.0, float, "Depth input timeout before FAILSAFE."),
        ("depth_pose_scale", -1.0, float, "Scale applied to depth pose Z."),
        ("depth_pose_offset_m", 0.0, float, "Offset added after scaling depth pose Z."),
        ("max_depth_m", 10.5, float, "Maximum allowed depth before FAILSAFE."),
        ("buoy_class_id", 0, int, "Detector class ID used for buoys."),
        ("target_confirm_hits", 3, int, "Consecutive detections required for confirmation."),
        ("target_confirm_sec", 0.2, float, "Additional stable-detection time."),
        ("buoy_confidence_similar_delta", 0.05, float, "Confidence difference considered meaningful when areas match."),
        ("buoy_same_target_center_ratio", 0.12, float, "Per-axis normalized centre tolerance for the same target."),
        ("lpf_tau_sec", 0.3, float, "Bounding-box low-pass-filter time constant; zero disables it."),

        # Initial scan, reacquisition, and approach
        ("initial_search_radius_m", 2.0, float, "Maximum initial-target excursion from handoff."),
        ("initial_scan_yaw_pwm", 1560, int, "Yaw PWM used during the initial 360-degree scan."),
        ("initial_scan_completion_tolerance_rad", 0.15, float, "Angular tolerance for completing the initial scan."),
        ("initial_target_reacquire_timeout_sec", 5.0, float, "Timeout while turning back to the scan candidate."),
        ("reacquire_yaw_pwm", 1470, int, "Yaw PWM used for short target reacquisition."),
        ("reacquire_yaw_duration_sec", 0.5, float, "Duration of active reacquisition yaw."),
        ("reacquire_timeout_sec", 1.0, float, "Total target reacquisition timeout."),
        ("approach_target_x", 0.50, float, "Normalized target centre X during approach."),
        ("approach_target_y", 0.30, float, "Normalized target centre Y during approach."),
        ("approach_area_ratio", 0.03, float, "Bounding-box area ratio that stops normal approach."),
        ("approach_close_hold_sec", 2.0, float, "Required continuous close-area hold time."),
        ("strong_forward_pwm", 1700, int, "Forward PWM during STRONG_FORWARD."),
        ("strong_forward_duration_sec", 5.0, float, "STRONG_FORWARD duration."),
        ("strong_backoff_pwm", 1300, int, "Forward-channel PWM during STRONG_BACKOFF."),
        ("strong_backoff_duration_sec", 3.0, float, "STRONG_BACKOFF duration."),

        # RC channels and horizontal motion
        ("throttle_channel", 3, int, "One-based RC throttle/depth channel."),
        ("yaw_channel", 4, int, "One-based RC yaw channel."),
        ("forward_channel", 5, int, "One-based RC forward channel."),
        ("neutral_pwm", 1500, int, "Neutral PWM."),
        ("min_pwm", 1300, int, "Minimum controller PWM."),
        ("max_pwm", 1700, int, "Maximum controller PWM."),
        ("rc_pwm_span", 400.0, float, "PWM scaling span for waypoint yaw control."),
        ("lane_forward_pwm", 1700, int, "Forward PWM during waypoint and lane motion."),
        ("approach_forward_pwm", 1560, int, "Minimum non-neutral approach PWM near the area threshold."),
        ("approach_forward_max_pwm", 1700, int, "Maximum approach PWM for a distant target."),
        ("waypoint_heading_tolerance_rad", 0.1745, float, "Heading tolerance required before waypoint forward motion."),
        ("waypoint_yaw_kp", 1.15, float, "Waypoint yaw PID proportional gain."),
        ("waypoint_yaw_ki", 0.15, float, "Waypoint yaw PID integral gain."),
        ("waypoint_yaw_kd", 0.08, float, "Waypoint yaw PID derivative gain."),
        ("waypoint_yaw_integral_limit", 2.0, float, "Absolute waypoint yaw integral limit."),
        ("max_waypoint_yaw_delta_pwm", 180, int, "Maximum waypoint yaw deviation from neutral."),
        ("waypoint_yaw_invert", True, bool, "Invert waypoint yaw output direction."),

        # Vision alignment and depth PID
        ("vision_yaw_kp_pwm", 100.0, float, "Vision yaw PWM gain per normalized horizontal error."),
        ("max_vision_yaw_delta_pwm", 100, int, "Maximum vision yaw deviation from neutral."),
        ("max_vision_throttle_delta_pwm", 100, int, "Maximum vision vertical deviation from neutral."),
        ("vision_yaw_invert", False, bool, "Invert vision yaw output direction."),
        ("vertical_positive_is_up", True, bool, "Treat increasing vertical command as upward motion."),
        ("approach_vision_throttle_weight", 0.4, float, "Vision weight in approach vertical blending."),
        ("depth_kp_pwm_per_m", 120.0, float, "Depth PID proportional gain."),
        ("depth_ki_pwm_per_m_sec", 20.0, float, "Depth PID integral gain."),
        ("depth_kd_pwm_sec_per_m", 16.0, float, "Depth PID derivative gain."),
        ("depth_integral_limit_m_sec", 2.0, float, "Absolute depth integral limit."),
        ("buoyancy_hold_delta_pwm", 40, int, "Constant buoyancy compensation PWM delta."),
        ("max_depth_delta_pwm", 160, int, "Maximum depth-control deviation from neutral."),
    ]

    declared_arguments = [
        _argument(name, default, description)
        for name, default, _, description in parameter_specs
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type, _ in parameter_specs
    }

    controller = Node(
        package="auv_vision_control",
        executable="lane_vision_controller_node",
        name="lane_vision_controller_node",
        output="screen",
        emulate_tty=True,
        parameters=[parameters],
    )

    return LaunchDescription(declared_arguments + [controller])
