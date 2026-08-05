#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "auv_lane_vision_control/arena_frame_transform.hpp"
#include "auv_lane_vision_control/lane_planner.hpp"

namespace auv_lane_vision_control
{
class LaneVisionControllerNode : public rclcpp::Node
{
public:
  // 파라미터와 통신 인터페이스를 준비하고 제어 노드를 초기화한다.
  LaneVisionControllerNode()
  : Node("lane_vision_controller_node")
  {
    declare_topics();
    declare_arena_parameters();
    declare_mission_parameters();
    declare_control_parameters();
    validate_parameters();

    lane_planner_ = std::make_unique<LanePlanner>(
      ArenaConfig{
        arena_length_m_,
        arena_width_m_,
        arena_offset_x_m_,
        arena_offset_y_m_,
        arena_safety_margin_m_,
        lane_search_offset_m_,
        arena_start_corner_});
    lane_completed_.assign(lane_planner_->lanes().size(), false);

    create_interfaces();

    state_entered_at_ = now();
    publish_state();
    publish_target_confirmed(false);

    const auto & bounds = lane_planner_->safe_bounds();
    RCLCPP_INFO(
      get_logger(),
      "Lane vision controller ready: lanes=%zu spacing=%.2f m "
      "safe_bounds=[%.2f, %.2f]x[%.2f, %.2f]",
      lane_planner_->lanes().size(), lane_planner_->actual_lane_spacing_m(),
      bounds.x_min, bounds.x_max, bounds.y_min, bounds.y_max);
    RCLCPP_INFO(
      get_logger(),
      "RC remains silent until %s grants control",
      vision_control_granted_topic_.c_str());
  }

  // 노드 종료 전에 사용한 제어 채널을 한 번 해제한다.
  void publish_release_once()
  {
    if (!vision_has_control_) {
      return;
    }
    auto channels = nochange_channels();
    release_controlled_channels(channels);
    publish_channels(channels);
  }

private:
  static constexpr double YAW_DERIVATIVE_ALPHA = 0.2;

  enum class State
  {
    IDLE,
    TARGET_CONFIRM,
    WAIT_CONTROL_GRANT,
    INITIAL_SCAN_360,
    TURN_TO_INITIAL_TARGET,
    TARGET_HOLD,
    REACQUIRE_BUOY,
    APPROACH_BUOY,
    STRONG_FORWARD,
    STRONG_BACKOFF,
    MOVE_TO_LANE_START,
    LANE_FOLLOWING_WITH_SEARCH,
    RETURN_TO_ACTIVE_LANE,
    COMPLETE,
    FAILSAFE
  };

  enum class TargetContext
  {
    INITIAL,
    LANE
  };

  enum class LaneTransferPhase
  {
    ALIGN_TRANSFER,
    TRACK_TRANSFER,
    ALIGN_NEW_LANE
  };

  struct Detection
  {
    float confidence{0.0F};
    float center_x{0.0F};
    float center_y{0.0F};
    float width{0.0F};
    float height{0.0F};
    float image_width{0.0F};
    float image_height{0.0F};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    int consecutive_hits{1};
  };

  struct DepthSample
  {
    double depth_m{0.0};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
  };

  struct ScanCandidate
  {
    Detection detection{};
    double arena_yaw_rad{0.0};
    double area_ratio{0.0};
  };

  // 입력과 출력에 사용할 토픽 이름 파라미터를 선언한다.
  void declare_topics()
  {
    bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/vision/buoy_bbox");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/auv/depth");
    depth_pose_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    odometry_topic_ =
      declare_parameter<std::string>("odometry_topic", "/odometry/filtered");
    start_frame_topic_ =
      declare_parameter<std::string>("start_frame_topic", "/start_frame");
    vision_search_request_topic_ = declare_parameter<std::string>(
      "vision_search_request_topic", "/homing/vision_search_active");
    target_confirmed_topic_ =
      declare_parameter<std::string>("target_confirmed_topic", "/vision/target_confirmed");
    vision_control_granted_topic_ = declare_parameter<std::string>(
      "vision_control_granted_topic", "/homing/vision_control_granted");
    emergency_stop_topic_ =
      declare_parameter<std::string>("emergency_stop_topic", "/mission/emergency_stop");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mission/state");
    rc_override_topic_ =
      declare_parameter<std::string>("rc_override_topic", "/mavros/rc/override");
    rc_monitor_topic_ =
      declare_parameter<std::string>("rc_monitor_topic", "/mission/rc_command");
  }

  // 수조 경계와 레인 생성에 필요한 파라미터를 선언한다.
  void declare_arena_parameters()
  {
    arena_length_m_ = declare_parameter<double>("arena_length_m", 15.0);
    arena_width_m_ = declare_parameter<double>("arena_width_m", 16.0);
    arena_offset_x_m_ = declare_parameter<double>("arena_offset_x_m", -0.3);
    arena_offset_y_m_ = declare_parameter<double>("arena_offset_y_m", 0.3);
    arena_safety_margin_m_ = declare_parameter<double>("arena_safety_margin_m", 0.5);
    arena_start_corner_ =
      declare_parameter<std::string>("arena_start_corner", "bottom_left");
    lane_search_offset_m_ = declare_parameter<double>("lane_search_offset_m", 2.0);
    incapable_skip_distance_m_ =
      declare_parameter<double>("incapable_skip_distance_m", 2.0);
    waypoint_reach_tolerance_m_ =
      declare_parameter<double>("waypoint_reach_tolerance_m", 0.15);
  }

  // 탐색과 부표 작업에 필요한 파라미터를 선언한다.
  void declare_mission_parameters()
  {
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    odometry_timeout_sec_ = declare_parameter<double>("odometry_timeout_sec", 0.5);
    detection_timeout_sec_ = declare_parameter<double>("detection_timeout_sec", 1.0);
    depth_timeout_sec_ = declare_parameter<double>("depth_timeout_sec", 1.0);
    depth_pose_scale_ = declare_parameter<double>("depth_pose_scale", -1.0);
    depth_pose_offset_m_ = declare_parameter<double>("depth_pose_offset_m", 0.0);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 10.5);

    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);
    target_confirm_hits_ = declare_parameter<int>("target_confirm_hits", 3);
    target_confirm_sec_ = declare_parameter<double>("target_confirm_sec", 0.2);
    buoy_confidence_similar_delta_ =
      declare_parameter<double>("buoy_confidence_similar_delta", 0.05);
    buoy_same_target_center_ratio_ =
      declare_parameter<double>("buoy_same_target_center_ratio", 0.12);
    lpf_tau_sec_ = declare_parameter<double>("lpf_tau_sec", 0.3);

    initial_search_radius_m_ =
      declare_parameter<double>("initial_search_radius_m", 2.0);
    initial_scan_yaw_pwm_ = declare_parameter<int>("initial_scan_yaw_pwm", 1560);
    initial_scan_completion_tolerance_rad_ =
      declare_parameter<double>("initial_scan_completion_tolerance_rad", 0.15);
    initial_target_reacquire_timeout_sec_ =
      declare_parameter<double>("initial_target_reacquire_timeout_sec", 5.0);

    reacquire_yaw_pwm_ = declare_parameter<int>("reacquire_yaw_pwm", 1470);
    reacquire_yaw_duration_sec_ =
      declare_parameter<double>("reacquire_yaw_duration_sec", 0.5);
    reacquire_timeout_sec_ = declare_parameter<double>("reacquire_timeout_sec", 1.0);

    // 화면 가로 중앙, 세로 30% 지점을 추종하며 접근한다.
    approach_target_x_ = declare_parameter<double>("approach_target_x", 0.50);
    approach_target_y_ = declare_parameter<double>("approach_target_y", 0.30);
    approach_area_ratio_ = declare_parameter<double>("approach_area_ratio", 0.03);
    approach_close_hold_sec_ = declare_parameter<double>("approach_close_hold_sec", 2.0);

    strong_forward_pwm_ = declare_parameter<int>("strong_forward_pwm", 1700);
    strong_forward_duration_sec_ =
      declare_parameter<double>("strong_forward_duration_sec", 5.0);
    strong_backoff_pwm_ = declare_parameter<int>("strong_backoff_pwm", 1300);
    strong_backoff_duration_sec_ =
      declare_parameter<double>("strong_backoff_duration_sec", 3.0);
  }

  // 이동과 수심 유지에 필요한 제어 파라미터를 선언한다.
  void declare_control_parameters()
  {
    throttle_channel_ = declare_parameter<int>("throttle_channel", 3);
    yaw_channel_ = declare_parameter<int>("yaw_channel", 4);
    forward_channel_ = declare_parameter<int>("forward_channel", 5);
    neutral_pwm_ = declare_parameter<int>("neutral_pwm", 1500);
    min_pwm_ = declare_parameter<int>("min_pwm", 1300);
    max_pwm_ = declare_parameter<int>("max_pwm", 1700);
    rc_pwm_span_ = declare_parameter<double>("rc_pwm_span", 400.0);

    lane_forward_pwm_ = declare_parameter<int>("lane_forward_pwm", 1680);
    lane_forward_slow_pwm_ = declare_parameter<int>("lane_forward_slow_pwm", 1560);
    lane_lookahead_distance_m_ =
      declare_parameter<double>("lane_lookahead_distance_m", 1.0);
    lane_rejoin_cross_track_tolerance_m_ =
      declare_parameter<double>("lane_rejoin_cross_track_tolerance_m", 0.25);
    lane_rejoin_heading_tolerance_rad_ =
      declare_parameter<double>("lane_rejoin_heading_tolerance_rad", 0.2618);
    lane_forward_full_heading_rad_ =
      declare_parameter<double>("lane_forward_full_heading_rad", 0.1745);
    lane_forward_stop_heading_rad_ =
      declare_parameter<double>("lane_forward_stop_heading_rad", 0.5236);
    lane_transfer_forward_pwm_ =
      declare_parameter<int>("lane_transfer_forward_pwm", 1680);
    lane_transfer_dynamic_rejoin_ =
      declare_parameter<bool>("lane_transfer_dynamic_rejoin", true);
    lane_end_transition_distance_m_ =
      declare_parameter<double>("lane_end_transition_distance_m", 3.0);
    lane_transfer_slowdown_distance_m_ =
      declare_parameter<double>("lane_transfer_slowdown_distance_m", 0.75);
    lane_transfer_heading_tolerance_rad_ =
      declare_parameter<double>("lane_transfer_heading_tolerance_rad", 0.0873);
    lane_transfer_heading_hold_sec_ =
      declare_parameter<double>("lane_transfer_heading_hold_sec", 0.3);
    waypoint_slowdown_distance_m_ =
      declare_parameter<double>("waypoint_slowdown_distance_m", 0.25);
    waypoint_settle_speed_mps_ =
      declare_parameter<double>("waypoint_settle_speed_mps", 0.15);
    waypoint_settle_hold_sec_ =
      declare_parameter<double>("waypoint_settle_hold_sec", 0.3);
    approach_forward_pwm_ = declare_parameter<int>("approach_forward_pwm", 1560);
    approach_forward_max_pwm_ =
      declare_parameter<int>("approach_forward_max_pwm", 1700);
    waypoint_heading_tolerance_rad_ =
      declare_parameter<double>("waypoint_heading_tolerance_rad", 0.1745);
    waypoint_heading_release_rad_ =
      declare_parameter<double>("waypoint_heading_release_rad", 0.3491);
    waypoint_yaw_kp_ = declare_parameter<double>("waypoint_yaw_kp", 0.8);
    waypoint_yaw_ki_ = declare_parameter<double>("waypoint_yaw_ki", 0.1);
    waypoint_yaw_kd_ = declare_parameter<double>("waypoint_yaw_kd", 0.02);
    waypoint_yaw_integral_limit_ =
      declare_parameter<double>("waypoint_yaw_integral_limit", 0.7);
    max_waypoint_yaw_delta_pwm_ =
      declare_parameter<int>("max_waypoint_yaw_delta_pwm", 160);
    waypoint_yaw_invert_ = declare_parameter<bool>("waypoint_yaw_invert", true);

    vision_yaw_kp_pwm_ = declare_parameter<double>("vision_yaw_kp_pwm", 100.0);
    max_vision_yaw_delta_pwm_ =
      declare_parameter<int>("max_vision_yaw_delta_pwm", 100);
    max_vision_throttle_delta_pwm_ =
      declare_parameter<int>("max_vision_throttle_delta_pwm", 100);
    vision_yaw_invert_ = declare_parameter<bool>("vision_yaw_invert", false);
    vertical_positive_is_up_ =
      declare_parameter<bool>("vertical_positive_is_up", true);
    approach_vision_throttle_weight_ =
      declare_parameter<double>("approach_vision_throttle_weight", 0.4);

    depth_kp_pwm_per_m_ = declare_parameter<double>("depth_kp_pwm_per_m", 120.0);
    depth_ki_pwm_per_m_sec_ =
      declare_parameter<double>("depth_ki_pwm_per_m_sec", 20.0);
    depth_kd_pwm_sec_per_m_ =
      declare_parameter<double>("depth_kd_pwm_sec_per_m", 16.0);
    depth_integral_limit_m_sec_ =
      declare_parameter<double>("depth_integral_limit_m_sec", 2.0);
    buoyancy_hold_delta_pwm_ =
      declare_parameter<int>("buoyancy_hold_delta_pwm", 40);
    max_depth_delta_pwm_ = declare_parameter<int>("max_depth_delta_pwm", 160);
  }

  // 선언한 파라미터가 안전한 범위와 관계를 만족하는지 확인한다.
  void validate_parameters() const
  {
    if (
      min_pwm_ < 1300 || max_pwm_ > 1700 || min_pwm_ >= max_pwm_ ||
      neutral_pwm_ < min_pwm_ || neutral_pwm_ > max_pwm_)
    {
      throw std::invalid_argument(
              "PWM range must stay within 1300..1700 and include neutral_pwm");
    }
    for (const int channel : {throttle_channel_, yaw_channel_, forward_channel_}) {
      if (channel < 1 || channel > 18) {
        throw std::invalid_argument("controlled RC channels must be in [1, 18]");
      }
    }
    if (
      throttle_channel_ == yaw_channel_ || throttle_channel_ == forward_channel_ ||
      yaw_channel_ == forward_channel_)
    {
      throw std::invalid_argument("controlled RC channels must be unique");
    }
    if (
      control_rate_hz_ <= 0.0 || odometry_timeout_sec_ <= 0.0 ||
      detection_timeout_sec_ <= 0.0 || depth_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("control rate and input timeouts must be positive");
    }
    if (
      target_confirm_hits_ < 1 || target_confirm_sec_ < 0.0 ||
      lpf_tau_sec_ < 0.0 || initial_search_radius_m_ <= 0.0 ||
      !std::isfinite(buoy_confidence_similar_delta_) ||
      buoy_confidence_similar_delta_ < 0.0 || buoy_confidence_similar_delta_ > 1.0 ||
      !std::isfinite(buoy_same_target_center_ratio_) ||
      buoy_same_target_center_ratio_ < 0.0 || buoy_same_target_center_ratio_ > 1.0)
    {
      throw std::invalid_argument("detection and initial search parameters are invalid");
    }
    if (
      !std::isfinite(approach_target_x_) || !std::isfinite(approach_target_y_) ||
      approach_target_x_ < 0.0 || approach_target_x_ > 1.0 ||
      approach_target_y_ < 0.0 || approach_target_y_ > 1.0 ||
      !std::isfinite(approach_close_hold_sec_) || approach_close_hold_sec_ < 0.0)
    {
      throw std::invalid_argument("invalid approach target or close hold time");
    }
    if (
      approach_vision_throttle_weight_ < 0.0 ||
      approach_vision_throttle_weight_ > 1.0)
    {
      throw std::invalid_argument("approach_vision_throttle_weight must be in [0, 1]");
    }
    if (
      rc_pwm_span_ <= 0.0 || lane_forward_pwm_ < neutral_pwm_ ||
      lane_forward_pwm_ > max_pwm_ ||
      lane_forward_slow_pwm_ < neutral_pwm_ ||
      lane_forward_slow_pwm_ > lane_forward_pwm_ ||
      lane_lookahead_distance_m_ <= 0.0 ||
      lane_rejoin_cross_track_tolerance_m_ <= 0.0 ||
      lane_rejoin_cross_track_tolerance_m_ > lane_search_offset_m_ ||
      lane_rejoin_heading_tolerance_rad_ <= 0.0 ||
      lane_rejoin_heading_tolerance_rad_ > M_PI ||
      lane_forward_full_heading_rad_ < 0.0 ||
      lane_forward_stop_heading_rad_ <= lane_forward_full_heading_rad_ ||
      lane_forward_stop_heading_rad_ > M_PI ||
      lane_transfer_forward_pwm_ < lane_forward_slow_pwm_ ||
      lane_transfer_forward_pwm_ > lane_forward_pwm_ ||
      !std::isfinite(lane_end_transition_distance_m_) ||
      lane_end_transition_distance_m_ < 0.0 ||
      lane_end_transition_distance_m_ >=
      arena_length_m_ - 2.0 * arena_safety_margin_m_ ||
      lane_transfer_slowdown_distance_m_ < waypoint_reach_tolerance_m_ ||
      lane_transfer_heading_tolerance_rad_ <= 0.0 ||
      lane_transfer_heading_tolerance_rad_ > M_PI ||
      lane_transfer_heading_hold_sec_ < 0.0 ||
      waypoint_slowdown_distance_m_ < waypoint_reach_tolerance_m_ ||
      waypoint_settle_speed_mps_ < 0.0 || waypoint_settle_hold_sec_ < 0.0 ||
      waypoint_heading_tolerance_rad_ <= 0.0 ||
      waypoint_heading_release_rad_ < waypoint_heading_tolerance_rad_ ||
      waypoint_heading_release_rad_ > M_PI || waypoint_yaw_kp_ < 0.0 ||
      waypoint_yaw_ki_ < 0.0 || waypoint_yaw_kd_ < 0.0 ||
      waypoint_yaw_integral_limit_ < 0.0 || max_waypoint_yaw_delta_pwm_ < 0)
    {
      throw std::invalid_argument("waypoint motion parameters are invalid");
    }
    if (
      reacquire_yaw_duration_sec_ < 0.0 ||
      reacquire_timeout_sec_ < reacquire_yaw_duration_sec_ ||
      approach_area_ratio_ <= 0.0 ||
      approach_area_ratio_ > 1.0 || approach_forward_max_pwm_ < approach_forward_pwm_ ||
      !std::isfinite(strong_forward_duration_sec_) || strong_forward_duration_sec_ < 0.0 ||
      !std::isfinite(strong_backoff_duration_sec_) || strong_backoff_duration_sec_ < 0.0 ||
      strong_forward_pwm_ < min_pwm_ || strong_forward_pwm_ > max_pwm_ ||
      strong_backoff_pwm_ < min_pwm_ || strong_backoff_pwm_ > max_pwm_ ||
      !std::isfinite(vision_yaw_kp_pwm_) || vision_yaw_kp_pwm_ < 0.0 ||
      max_vision_yaw_delta_pwm_ < 0)
    {
      throw std::invalid_argument("approach and reacquire parameters are invalid");
    }
  }

  // 구독자와 발행자 및 주기 실행기를 생성한다.
  void create_interfaces()
  {
    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, 10,
      std::bind(&LaneVisionControllerNode::on_bbox, this, std::placeholders::_1));
    depth_sub_ = create_subscription<std_msgs::msg::Float64>(
      depth_topic_, 10,
      std::bind(&LaneVisionControllerNode::on_depth, this, std::placeholders::_1));
    if (!depth_pose_topic_.empty()) {
      depth_pose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        depth_pose_topic_, 10,
        std::bind(&LaneVisionControllerNode::on_depth_pose, this, std::placeholders::_1));
    }
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 30,
      std::bind(&LaneVisionControllerNode::on_odometry, this, std::placeholders::_1));
    start_frame_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      start_frame_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LaneVisionControllerNode::on_start_frame, this, std::placeholders::_1));
    vision_search_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      vision_search_request_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &LaneVisionControllerNode::on_vision_search_request, this,
        std::placeholders::_1));
    vision_control_granted_sub_ = create_subscription<std_msgs::msg::Bool>(
      vision_control_granted_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &LaneVisionControllerNode::on_vision_control_granted, this,
        std::placeholders::_1));
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LaneVisionControllerNode::on_emergency_stop, this, std::placeholders::_1));

    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);
    rc_monitor_pub_ =
      create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_monitor_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local());
    target_confirmed_pub_ = create_publisher<std_msgs::msg::Bool>(
      target_confirmed_topic_, rclcpp::QoS(1).reliable().transient_local());

    const double period_sec = 1.0 / control_rate_hz_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&LaneVisionControllerNode::on_timer, this));
  }

  // 시작 위치와 방향을 받아 수조 좌표 변환 기준을 설정한다.
  void on_start_frame(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (arena_transform_.initialized()) {
      return;
    }
    const Vec2 origin{msg->pose.position.x, msg->pose.position.y};
    const double yaw = ArenaFrameTransform::yaw_from_quaternion(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);
    if (!finite(origin) || !std::isfinite(yaw)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid start frame");
      return;
    }
    arena_transform_.initialize(origin, yaw);
    RCLCPP_INFO(
      get_logger(), "Arena start frame accepted: origin=(%.2f, %.2f), yaw=%.3f",
      origin.x, origin.y, yaw);
  }

  // 위치와 방향을 수조 좌표로 변환해 현재 상태를 갱신한다.
  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Vec2 odom_position{msg->pose.pose.position.x, msg->pose.pose.position.y};
    const double odom_yaw = ArenaFrameTransform::yaw_from_quaternion(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    if (!finite(odom_position) || !std::isfinite(odom_yaw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring non-finite odometry");
      return;
    }
    if (!arena_transform_.initialized()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for /start_frame");
      return;
    }

    current_position_ = arena_transform_.position_from_odom(odom_position);
    current_yaw_rad_ = arena_transform_.yaw_from_odom(odom_yaw);
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    if (std::isfinite(vx) && std::isfinite(vy)) {
      current_horizontal_speed_mps_ = std::hypot(vx, vy);
    }
    odometry_received_at_ = now();
    have_odometry_ = true;
  }

  // 단일 수심 값을 보조 수심 입력으로 저장한다.
  void on_depth(const std_msgs::msg::Float64::SharedPtr msg)
  {
    accept_depth(scalar_depth_, msg->data);
  }

  // 위치 메시지의 높이값을 주 수심 입력으로 변환해 저장한다.
  void on_depth_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    accept_depth(
      pose_depth_,
      depth_pose_scale_ * msg->pose.pose.position.z + depth_pose_offset_m_);
  }

  // 수심 값의 유효성을 검사한 뒤 수신 시각과 함께 저장한다.
  void accept_depth(std::optional<DepthSample> & slot, const double depth_m)
  {
    if (!std::isfinite(depth_m) || depth_m < 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring invalid depth sample");
      return;
    }
    slot = DepthSample{depth_m, now()};
  }

  // 검출 메시지에서 가장 적합한 부표 상자를 선택해 갱신한다.
  void on_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 10) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring bbox with fewer than 10 values");
      return;
    }

    std::optional<Detection> best_buoy;
    const size_t block_count = msg->data.size() / 10;
    for (size_t block = 0; block < block_count; ++block) {
      const size_t base = block * 10;
      bool values_are_finite = true;
      for (size_t index = 0; index < 10; ++index) {
        if (!std::isfinite(msg->data[base + index])) {
          values_are_finite = false;
          break;
        }
      }
      if (
        !values_are_finite || msg->data[base + 1] < 0.5F ||
        msg->data[base + 8] <= 0.0F || msg->data[base + 9] <= 0.0F)
      {
        continue;
      }

      const int class_id = static_cast<int>(std::lround(msg->data[base + 2]));
      if (class_id != buoy_class_id_) {
        continue;
      }
      Detection candidate{
        msg->data[base + 3],
        msg->data[base + 4],
        msg->data[base + 5],
        msg->data[base + 6],
        msg->data[base + 7],
        msg->data[base + 8],
        msg->data[base + 9],
        now(),
        1};
      if (!best_buoy || is_better_buoy(candidate, *best_buoy)) {
        best_buoy = candidate;
      }
    }

    if (!best_buoy) {
      return;
    }
    const Detection raw_detection = *best_buoy;
    accept_buoy_detection(raw_detection);
    if (state_ == State::INITIAL_SCAN_360 && have_odometry_) {
      consider_scan_candidate(raw_detection);
    }
  }

  // bbox 면적을 최우선으로 하고, 면적이 같을 때 confidence와 오른쪽 순으로 판단한다.
  bool is_better_buoy(const Detection & candidate, const Detection & current) const
  {
    const double candidate_area = std::max(
      0.0, static_cast<double>(candidate.width) * static_cast<double>(candidate.height));
    const double current_area = std::max(
      0.0, static_cast<double>(current.width) * static_cast<double>(current.height));
    if (candidate_area != current_area) {
      return candidate_area > current_area;
    }
    if (
      std::abs(candidate.confidence - current.confidence) >
      buoy_confidence_similar_delta_)
    {
      return candidate.confidence > current.confidence;
    }
    return candidate.center_x > current.center_x;
  }

  // 탐색 중에는 더 좋은 후보로 교체하고, 접근 이후에는 선택한 타깃만 유지한다.
  void accept_buoy_detection(Detection incoming)
  {
    const bool selecting =
      state_ == State::IDLE || state_ == State::TARGET_CONFIRM ||
      state_ == State::INITIAL_SCAN_360 ||
      state_ == State::LANE_FOLLOWING_WITH_SEARCH ||
      state_ == State::TARGET_HOLD || state_ == State::REACQUIRE_BUOY;

    if (!recent(buoy_)) {
      buoy_ = incoming;
      target_confirm_started_at_.reset();
      target_hold_confirm_started_at_.reset();
      detection_confirm_started_at_.reset();
      return;
    }
    if (same_buoy_target(incoming, *buoy_)) {
      update_detection_slot(incoming);
      return;
    }
    if (selecting && is_better_buoy(incoming, *buoy_)) {
      buoy_ = incoming;
      target_confirm_started_at_.reset();
      target_hold_confirm_started_at_.reset();
      detection_confirm_started_at_.reset();
    }
  }

  // 두 검출 중심의 차이로 같은 부표인지 판단한다.
  bool same_buoy_target(const Detection & a, const Detection & b) const
  {
    const double width = std::max(a.image_width, b.image_width);
    const double height = std::max(a.image_height, b.image_height);
    if (width <= 0.0 || height <= 0.0) {
      return false;
    }
    return
      std::abs(a.center_x - b.center_x) / width <= buoy_same_target_center_ratio_ &&
      std::abs(a.center_y - b.center_y) / height <= buoy_same_target_center_ratio_;
  }

  // 같은 타깃의 연속 검출 횟수와 저역통과필터 값을 갱신한다.
  void update_detection_slot(Detection incoming)
  {
    const auto received_at = now();
    const double dt = (received_at - buoy_->received_at).seconds();
    incoming.center_x =
      static_cast<float>(low_pass(buoy_->center_x, incoming.center_x, dt));
    incoming.center_y =
      static_cast<float>(low_pass(buoy_->center_y, incoming.center_y, dt));
    incoming.width = static_cast<float>(low_pass(buoy_->width, incoming.width, dt));
    incoming.height =
      static_cast<float>(low_pass(buoy_->height, incoming.height, dt));
    incoming.received_at = received_at;
    incoming.consecutive_hits = buoy_->consecutive_hits + 1;
    buoy_ = incoming;
  }

  // 이전 값과 새 값을 시간 간격에 따라 부드럽게 결합한다.
  double low_pass(const double previous, const double sample, const double dt) const
  {
    if (lpf_tau_sec_ <= 1.0e-9 || dt <= 0.0) {
      return sample;
    }
    const double alpha = dt / (lpf_tau_sec_ + dt);
    return alpha * sample + (1.0 - alpha) * previous;
  }

  // 회전 탐색 중 공통 선택 우선순위에 가장 잘 맞는 후보와 관측 방향을 저장한다.
  void consider_scan_candidate(const Detection & detection)
  {
    const double area = detection_area_ratio(detection);
    if (!scan_candidate_ || is_better_buoy(detection, scan_candidate_->detection)) {
      scan_candidate_ = ScanCandidate{detection, current_yaw_rad_, area};
    }
  }

  // 탐색 요청을 받으면 이전 기록을 초기화하고 부표 확인을 시작한다.
  void on_vision_search_request(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || state_ != State::IDLE) {
      return;
    }
    reset_mission();
    publish_target_confirmed(false);
    transition_to(State::TARGET_CONFIRM, "hydrophone requested visual confirmation");
  }

  // 제어권 승인 시 현재 수심을 고정하고 초기 부표 처리를 시작한다.
  void on_vision_control_granted(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    if (
      state_ != State::IDLE && state_ != State::TARGET_CONFIRM &&
      state_ != State::WAIT_CONTROL_GRANT)
    {
      return;
    }

    vision_has_control_ = true;
    if (!have_recent_odometry()) {
      transition_to(State::FAILSAFE, "control granted without valid arena odometry");
      return;
    }
    const auto depth = current_depth();
    if (!depth) {
      transition_to(State::FAILSAFE, "control granted without valid depth");
      return;
    }
    mission_hold_depth_m_ = *depth;
    reset_depth_pid();
    handoff_position_ = current_position_;

    if (confirmed_buoy()) {
      begin_target(TargetContext::INITIAL, "confirmed hydrophone handoff target");
    } else {
      begin_initial_scan();
    }
  }

  // 비상정지 요청을 받으면 안전 정지 상태로 전환한다.
  void on_emergency_stop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    transition_to(State::FAILSAFE, "emergency stop");
  }

  // 새 임무를 시작할 수 있도록 진행 상태와 제어 누적값을 초기화한다.
  void reset_mission()
  {
    buoy_.reset();
    scan_candidate_.reset();
    active_lane_index_.reset();
    lane_completed_.assign(lane_planner_->lanes().size(), false);
    target_confirm_started_at_.reset();
    target_hold_confirm_started_at_.reset();
    approach_close_started_at_.reset();
    detection_confirm_started_at_.reset();
    ignore_detections_until_progress_m_.reset();
    lane_transfer_enabled_ = false;
    lane_transfer_heading_stable_started_at_.reset();
    mission_hold_depth_m_.reset();
    vision_has_control_ = false;
    reset_waypoint_pid();
    reset_depth_pid();
  }

  // 입력 안전성을 검사하고 현재 상태에 맞는 제어 동작을 주기적으로 수행한다.
  void on_timer()
  {
    if (state_ == State::IDLE) {
      return;
    }
    if (state_ == State::TARGET_CONFIRM) {
      run_target_confirm();
      return;
    }
    if (state_ == State::WAIT_CONTROL_GRANT || !vision_has_control_) {
      return;
    }

    auto channels = nochange_channels();
    if (state_ == State::COMPLETE || state_ == State::FAILSAFE) {
      release_controlled_channels(channels);
      publish_channels(channels);
      return;
    }

    if (!have_recent_odometry()) {
      transition_to(State::FAILSAFE, "odometry stale");
    } else if (!current_depth() || !mission_hold_depth_m_) {
      transition_to(State::FAILSAFE, "depth stale");
    } else if (*current_depth() > max_depth_m_) {
      transition_to(State::FAILSAFE, "maximum depth exceeded");
    }
    if (state_ == State::FAILSAFE) {
      release_controlled_channels(channels);
      publish_channels(channels);
      return;
    }

    switch (state_) {
      case State::IDLE:
      case State::TARGET_CONFIRM:
      case State::WAIT_CONTROL_GRANT:
      case State::COMPLETE:
      case State::FAILSAFE:
        break;
      case State::INITIAL_SCAN_360:
        run_initial_scan(channels);
        break;
      case State::TURN_TO_INITIAL_TARGET:
        run_turn_to_initial_target(channels);
        break;
      case State::TARGET_HOLD:
        run_target_hold(channels);
        break;
      case State::REACQUIRE_BUOY:
        run_reacquire_buoy(channels);
        break;
      case State::APPROACH_BUOY:
        run_approach_buoy(channels);
        break;
      case State::STRONG_FORWARD:
        run_strong_forward(channels);
        break;
      case State::STRONG_BACKOFF:
        run_strong_backoff(channels);
        break;
      case State::MOVE_TO_LANE_START:
        run_move_to_lane_start(channels);
        break;
      case State::LANE_FOLLOWING_WITH_SEARCH:
        run_lane_following(channels);
        break;
      case State::RETURN_TO_ACTIVE_LANE:
        run_return_to_active_lane(channels);
        break;
    }

    if (state_ == State::COMPLETE || state_ == State::FAILSAFE) {
      channels = nochange_channels();
      release_controlled_channels(channels);
    }
    publish_channels(channels);
  }

  // 연속 검출 조건을 만족한 부표를 확정해 제어권 승인을 요청한다.
  void run_target_confirm()
  {
    if (!confirmed_buoy()) {
      target_confirm_started_at_.reset();
      return;
    }
    if (!target_confirm_started_at_) {
      target_confirm_started_at_ = now();
      return;
    }
    if ((now() - *target_confirm_started_at_).seconds() < target_confirm_sec_) {
      return;
    }
    publish_target_confirmed(true);
    transition_to(State::WAIT_CONTROL_GRANT, "stable buoy confirmed");
  }

  // 초기 부표가 없을 때 한 바퀴 회전을 위한 누적값을 초기화한다.
  void begin_initial_scan()
  {
    scan_candidate_.reset();
    scan_accumulated_yaw_rad_ = 0.0;
    scan_previous_yaw_rad_ = current_yaw_rad_;
    transition_to(State::INITIAL_SCAN_360, "no confirmed target; starting 360-degree scan");
  }

  // 제자리에서 한 바퀴 회전하며 가장 가까운 부표 후보를 찾는다.
  void run_initial_scan(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    set_channel(channels, yaw_channel_, initial_scan_yaw_pwm_);

    const double delta = std::abs(wrap_pi(current_yaw_rad_ - scan_previous_yaw_rad_));
    if (delta < 0.5) {
      scan_accumulated_yaw_rad_ += delta;
    }
    scan_previous_yaw_rad_ = current_yaw_rad_;
    if (
      scan_accumulated_yaw_rad_ <
      2.0 * M_PI - initial_scan_completion_tolerance_rad_)
    {
      return;
    }

    if (!scan_candidate_) {
      select_next_lane("360-degree scan completed without a buoy");
      return;
    }
    initial_target_yaw_rad_ = scan_candidate_->arena_yaw_rad;
    buoy_.reset();
    reset_waypoint_pid();
    transition_to(State::TURN_TO_INITIAL_TARGET, "nearest scan candidate selected");
  }

  // 회전 탐색에서 선택한 방향으로 돌며 초기 부표를 다시 찾는다.
  void run_turn_to_initial_target(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    set_heading_control(channels, initial_target_yaw_rad_);
    if (recent(buoy_)) {
      begin_target(TargetContext::INITIAL, "initial scan target reacquired");
      return;
    }
    if (state_age_sec() >= initial_target_reacquire_timeout_sec_) {
      select_next_lane("initial scan target could not be reacquired");
    }
  }

  // 부표의 발견 위치와 처리 구간을 저장하고, 정지한 상태에서 재확인한다.
  void begin_target(const TargetContext context, const std::string & reason)
  {
    target_context_ = context;
    target_hold_confirm_started_at_.reset();
    approach_close_started_at_.reset();
    if (context == TargetContext::LANE) {
      if (!active_lane_index_) {
        transition_to(State::FAILSAFE, "lane target selected without an active lane");
        return;
      }
      target_departure_progress_m_ = active_lane_progress();
    }
    transition_to(State::TARGET_HOLD, reason);
  }

  // buoy 검출 직후 수평 이동을 멈추고 동일 타깃의 안정 검출을 확인한다.
  void run_target_hold(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    if (!recent(buoy_)) {
      buoy_.reset();
      target_hold_confirm_started_at_.reset();
      transition_to(State::REACQUIRE_BUOY, "buoy lost while stopped");
      return;
    }

    if (buoy_->consecutive_hits < target_confirm_hits_) {
      target_hold_confirm_started_at_.reset();
      return;
    }
    if (!target_hold_confirm_started_at_) {
      target_hold_confirm_started_at_ = now();
      return;
    }
    if ((now() - *target_hold_confirm_started_at_).seconds() >= target_confirm_sec_) {
      transition_to(State::APPROACH_BUOY, "stable buoy confirmed while stopped");
    }
  }

  // TARGET_HOLD에서 놓친 부표를 짧게 역방향 yaw로 다시 탐색한다.
  void run_reacquire_buoy(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    if (state_age_sec() < reacquire_yaw_duration_sec_) {
      set_channel(channels, yaw_channel_, reacquire_yaw_pwm_);
    }
    if (recent(buoy_)) {
      target_hold_confirm_started_at_.reset();
      transition_to(State::TARGET_HOLD, "buoy reacquired");
      return;
    }
    if (state_age_sec() >= reacquire_timeout_sec_) {
      finish_target(true, "buoy was not reacquired within timeout");
    }
  }

  // 목표점 (0.50, 0.30)으로 정렬하며 접근하고, 목표 면적을 2초 유지하면 강한 전진한다.
  void run_approach_buoy(std::array<uint16_t, 18> & channels)
  {
    if (!recent(buoy_)) {
      buoy_.reset();
      transition_to(State::REACQUIRE_BUOY, "buoy lost during approach");
      return;
    }
    if (target_excursion_limit_reached()) {
      finish_target(true, "target incapable within allowed excursion");
      return;
    }

    const bool close_enough = detection_area_ratio(*buoy_) >= approach_area_ratio_;
    const int forward_pwm =
      close_enough ? neutral_pwm_ : approach_forward_pwm_from_area(*buoy_);
    apply_visual_tracking(
      channels, *buoy_, forward_pwm, approach_target_x_, approach_target_y_);

    if (close_enough) {
      if (!approach_close_started_at_) {
        approach_close_started_at_ = now();
      } else if (
        (now() - *approach_close_started_at_).seconds() >= approach_close_hold_sec_)
      {
        transition_to(State::STRONG_FORWARD, "close buoy held for approach delay");
      }
    } else {
      approach_close_started_at_.reset();
    }
  }

  // 테스트 패키지와 동일한 강한 전진 펄스.
  void run_strong_forward(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    if (state_age_sec() >= strong_forward_duration_sec_) {
      transition_to(State::STRONG_BACKOFF, "strong forward pulse complete");
      set_channel(channels, forward_channel_, strong_backoff_pwm_);
      return;
    }
    set_channel(channels, forward_channel_, strong_forward_pwm_);
  }

  // 강한 후진 뒤 기존 미션 문맥의 레인 탐색 흐름으로 복귀한다.
  void run_strong_backoff(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    if (state_age_sec() < strong_backoff_duration_sec_) {
      set_channel(channels, forward_channel_, strong_backoff_pwm_);
      return;
    }
    finish_target(false, "strong forward/backoff cycle complete");
  }

  // bbox 면적비에 따라 멀리서는 빠르게, 가까워질수록 저속으로 접근한다.
  int approach_forward_pwm_from_area(const Detection & buoy) const
  {
    const double ratio = detection_area_ratio(buoy);
    const double fraction = std::clamp(
      1.0 - ratio / approach_area_ratio_, 0.0, 1.0);
    return approach_forward_pwm_ + static_cast<int>(std::lround(
      fraction * static_cast<double>(approach_forward_max_pwm_ - approach_forward_pwm_)));
  }

  // 초기 반경 또는 활성 레인 오프셋 한계에 도달했는지 확인한다.
  bool target_excursion_limit_reached() const
  {
    if (!lane_planner_->inside_safe_bounds(current_position_)) {
      return true;
    }
    if (target_context_ == TargetContext::INITIAL) {
      return distance(current_position_, handoff_position_) >= initial_search_radius_m_;
    }
    if (!active_lane_index_) {
      return true;
    }
    return lane_planner_->cross_track_distance(
      current_position_, *active_lane_index_) >= lane_search_offset_m_;
  }

  // 부표 처리 기록을 정리하고 시작점 이동 또는 활성 레인 복귀를 준비한다.
  void finish_target(const bool skip_repeated_detection, const std::string & reason)
  {
    buoy_.reset();
    detection_confirm_started_at_.reset();
    if (target_context_ == TargetContext::INITIAL) {
      select_next_lane(reason);
      return;
    }
    if (!active_lane_index_) {
      transition_to(State::FAILSAFE, "cannot return because active lane is missing");
      return;
    }
    if (skip_repeated_detection) {
      const double lane_length = lane_planner_->lane_length(*active_lane_index_);
      ignore_detections_until_progress_m_ = std::min(
        lane_length, target_departure_progress_m_ + incapable_skip_distance_m_);
    }
    reset_waypoint_pid();
    transition_to(State::RETURN_TO_ACTIVE_LANE, reason);
  }

  // 현재 위치에서 가장 가까운 미완료 레인 끝점을 다음 시작점으로 선택한다.
  void select_next_lane(const std::string & reason, const bool lane_to_lane = false)
  {
    const auto choice =
      lane_planner_->closest_uncompleted_endpoint(current_position_, lane_completed_);
    if (!choice) {
      active_lane_index_.reset();
      transition_to(State::COMPLETE, "all generated lanes completed");
      return;
    }

    active_lane_index_ = choice->lane_index;
    active_lane_start_at_a_ = choice->start_at_a;
    active_lane_start_ = choice->start;
    active_lane_finish_ = choice->finish;
    waypoint_ = active_lane_start_;
    lane_transfer_enabled_ = lane_to_lane;
    lane_transfer_phase_ = LaneTransferPhase::ALIGN_TRANSFER;
    lane_transfer_start_ = current_position_;
    lane_transfer_heading_stable_started_at_.reset();
    ignore_detections_until_progress_m_.reset();
    reset_waypoint_pid();
    transition_to(State::MOVE_TO_LANE_START, reason);
    RCLCPP_INFO(
      get_logger(),
      "Selected lane %zu start=(%.2f, %.2f) finish=(%.2f, %.2f)",
      *active_lane_index_, active_lane_start_.x, active_lane_start_.y,
      active_lane_finish_.x, active_lane_finish_.y);
  }

  // 선택된 레인 끝점으로 이동한 뒤 레인 주행을 시작한다.
  void run_move_to_lane_start(std::array<uint16_t, 18> & channels)
  {
    if (lane_transfer_enabled_) {
      run_lane_transfer(channels);
      return;
    }
    if (follow_waypoint(channels, waypoint_, lane_forward_pwm_)) {
      reset_waypoint_pid();
      detection_confirm_started_at_.reset();
      transition_to(State::LANE_FOLLOWING_WITH_SEARCH, "active lane start reached");
    }
  }

  // 설정에 따라 동적 LOS 합류 또는 기존 3단계 레인 전환을 수행한다.
  void run_lane_transfer(std::array<uint16_t, 18> & channels)
  {
    if (lane_transfer_dynamic_rejoin_) {
      run_dynamic_lane_transfer(channels);
      return;
    }

    set_neutral_control(channels);
    hold_mission_depth(channels);
    const Vec2 transfer_delta = waypoint_ - lane_transfer_start_;
    const double transfer_length = norm(transfer_delta);
    if (transfer_length <= 1.0e-9) {
      lane_transfer_phase_ = LaneTransferPhase::ALIGN_NEW_LANE;
    }
    const Vec2 transfer_direction = transfer_length > 1.0e-9 ?
      transfer_delta * (1.0 / transfer_length) : active_lane_direction();
    const double transfer_heading = std::atan2(
      transfer_direction.y, transfer_direction.x);

    if (lane_transfer_phase_ == LaneTransferPhase::ALIGN_TRANSFER) {
      set_heading_control(channels, transfer_heading);
      if (lane_transfer_heading_stable(transfer_heading)) {
        lane_transfer_phase_ = LaneTransferPhase::TRACK_TRANSFER;
        lane_transfer_heading_stable_started_at_.reset();
        reset_waypoint_pid();
        RCLCPP_INFO(get_logger(), "Lane transfer aligned; tracking connector");
      }
      return;
    }

    if (lane_transfer_phase_ == LaneTransferPhase::TRACK_TRANSFER) {
      const double endpoint_distance = distance(current_position_, waypoint_);
      if (endpoint_distance <= waypoint_reach_tolerance_m_) {
        if (current_horizontal_speed_mps_ > waypoint_settle_speed_mps_) {
          waypoint_settle_started_at_.reset();
          return;
        }
        if (!waypoint_settle_started_at_) {
          waypoint_settle_started_at_ = now();
          return;
        }
        if (
          (now() - *waypoint_settle_started_at_).seconds() <
          waypoint_settle_hold_sec_)
        {
          return;
        }
        lane_transfer_phase_ = LaneTransferPhase::ALIGN_NEW_LANE;
        lane_transfer_heading_stable_started_at_.reset();
        reset_waypoint_pid();
        RCLCPP_INFO(get_logger(), "Lane transfer endpoint reached; aligning new lane");
        return;
      }
      waypoint_settle_started_at_.reset();

      const double projected_progress = std::clamp(
        dot(current_position_ - lane_transfer_start_, transfer_direction),
        0.0, transfer_length);
      const double target_progress = std::min(
        transfer_length, projected_progress + lane_lookahead_distance_m_);
      const Vec2 los_target =
        lane_transfer_start_ + transfer_direction * target_progress;
      const Vec2 los_delta = los_target - current_position_;
      const double desired_yaw = norm(los_delta) > 1.0e-9 ?
        std::atan2(los_delta.y, los_delta.x) : transfer_heading;
      const double heading_error = wrap_pi(desired_yaw - current_yaw_rad_);
      set_heading_control(channels, desired_yaw);

      double distance_scale = 1.0;
      if (
        endpoint_distance < lane_transfer_slowdown_distance_m_ &&
        lane_transfer_slowdown_distance_m_ > waypoint_reach_tolerance_m_)
      {
        distance_scale = std::clamp(
          (endpoint_distance - waypoint_reach_tolerance_m_) /
          (lane_transfer_slowdown_distance_m_ - waypoint_reach_tolerance_m_),
          0.0, 1.0);
      }
      const int distance_limited_pwm = static_cast<int>(std::lround(
        lane_forward_slow_pwm_ + distance_scale *
        static_cast<double>(lane_transfer_forward_pwm_ - lane_forward_slow_pwm_)));
      set_channel(
        channels, forward_channel_,
        forward_pwm_for_heading(std::abs(heading_error), distance_limited_pwm));
      return;
    }

    const Vec2 lane_direction = active_lane_direction();
    const double lane_heading = std::atan2(lane_direction.y, lane_direction.x);
    set_heading_control(channels, lane_heading);
    if (lane_transfer_heading_stable(lane_heading)) {
      lane_transfer_enabled_ = false;
      lane_transfer_heading_stable_started_at_.reset();
      reset_waypoint_pid();
      detection_confirm_started_at_.reset();
      transition_to(
        State::LANE_FOLLOWING_WITH_SEARCH,
        "lane transfer complete and new lane aligned");
    }
  }

  // 새 레인의 앞쪽 LOS 지점으로 바로 향해 두 번의 제자리 yaw 정렬을 없앤다.
  void run_dynamic_lane_transfer(std::array<uint16_t, 18> & channels)
  {
    if (!active_lane_index_) {
      transition_to(State::FAILSAFE, "dynamic lane transfer without an active lane");
      return;
    }

    apply_active_lane_los(channels, lane_transfer_forward_pwm_);
    const double cross_track = lane_planner_->cross_track_distance(
      current_position_, *active_lane_index_);
    const Vec2 lane_direction = active_lane_direction();
    const double lane_heading = std::atan2(lane_direction.y, lane_direction.x);
    const double lane_heading_error = wrap_pi(lane_heading - current_yaw_rad_);
    if (
      cross_track > lane_rejoin_cross_track_tolerance_m_ ||
      std::abs(lane_heading_error) > lane_rejoin_heading_tolerance_rad_)
    {
      return;
    }

    lane_transfer_enabled_ = false;
    lane_transfer_heading_stable_started_at_.reset();
    reset_waypoint_pid();
    detection_confirm_started_at_.reset();
    transition_to(
      State::LANE_FOLLOWING_WITH_SEARCH,
      "new lane dynamically acquired without a second yaw alignment");
  }

  // 지정 방향 오차가 허용 범위 안에서 설정 시간 유지되었는지 확인한다.
  bool lane_transfer_heading_stable(const double desired_yaw)
  {
    const double error = std::abs(wrap_pi(desired_yaw - current_yaw_rad_));
    if (error > lane_transfer_heading_tolerance_rad_) {
      lane_transfer_heading_stable_started_at_.reset();
      return false;
    }
    if (!lane_transfer_heading_stable_started_at_) {
      lane_transfer_heading_stable_started_at_ = now();
      return lane_transfer_heading_hold_sec_ <= 0.0;
    }
    return
      (now() - *lane_transfer_heading_stable_started_at_).seconds() >=
      lane_transfer_heading_hold_sec_;
  }

  // 활성 레인의 끝점을 향해 주행하면서 부표를 계속 확인한다.
  void run_lane_following(std::array<uint16_t, 18> & channels)
  {
    if (!active_lane_index_) {
      transition_to(State::FAILSAFE, "lane following without an active lane");
      return;
    }
    const double endpoint_distance = distance(current_position_, active_lane_finish_);
    if (
      lane_end_transition_distance_m_ > 0.0 &&
      endpoint_distance <= lane_end_transition_distance_m_)
    {
      RCLCPP_INFO(
        get_logger(),
        "Starting next-lane rejoin %.2f m before the active lane endpoint",
        endpoint_distance);
      lane_completed_[*active_lane_index_] = true;
      active_lane_index_.reset();
      select_next_lane("active lane early-transition point reached", true);
      return;
    }
    if (
      endpoint_distance <= lane_lookahead_distance_m_ &&
      follow_waypoint(channels, active_lane_finish_, lane_forward_pwm_))
    {
      lane_completed_[*active_lane_index_] = true;
      active_lane_index_.reset();
      select_next_lane("active lane completed", true);
      return;
    }
    if (endpoint_distance > lane_lookahead_distance_m_) {
      apply_active_lane_los(channels, lane_forward_pwm_);
    }

    const double progress = active_lane_progress();
    if (
      ignore_detections_until_progress_m_ &&
      progress + waypoint_reach_tolerance_m_ >= *ignore_detections_until_progress_m_)
    {
      ignore_detections_until_progress_m_.reset();
      buoy_.reset();
    }
    if (ignore_detections_until_progress_m_) {
      detection_confirm_started_at_.reset();
      return;
    }

    if (!confirmed_buoy()) {
      detection_confirm_started_at_.reset();
      return;
    }
    if (!detection_confirm_started_at_) {
      detection_confirm_started_at_ = now();
      return;
    }
    if ((now() - *detection_confirm_started_at_).seconds() >= target_confirm_sec_) {
      begin_target(TargetContext::LANE, "stable buoy detected during lane coverage");
      set_neutral_control(channels);
      hold_mission_depth(channels);
    }
  }

  // 부표 작업 후 매 주기 갱신되는 LOS 목표로 활성 레인에 부드럽게 합류한다.
  void run_return_to_active_lane(std::array<uint16_t, 18> & channels)
  {
    if (!active_lane_index_) {
      transition_to(State::FAILSAFE, "return requested without an active lane");
      return;
    }
    apply_active_lane_los(channels, lane_forward_pwm_);
    const double cross_track = lane_planner_->cross_track_distance(
      current_position_, *active_lane_index_);
    const Vec2 lane_direction = active_lane_direction();
    const double lane_heading = std::atan2(lane_direction.y, lane_direction.x);
    const double lane_heading_error = wrap_pi(lane_heading - current_yaw_rad_);
    if (
      cross_track <= lane_rejoin_cross_track_tolerance_m_ &&
      std::abs(lane_heading_error) <= lane_rejoin_heading_tolerance_rad_)
    {
      buoy_.reset();
      detection_confirm_started_at_.reset();
      transition_to(
        State::LANE_FOLLOWING_WITH_SEARCH,
        "active lane dynamically reacquired");
    }
  }

  // 현재 레인 투영점에서 진행 방향 앞쪽의 동적 LOS 목표를 계산한다.
  Vec2 active_lane_lookahead_target() const
  {
    const double lane_length = lane_planner_->lane_length(*active_lane_index_);
    const double target_progress = std::min(
      lane_length, active_lane_progress() + lane_lookahead_distance_m_);
    return active_lane_start_ + active_lane_direction() * target_progress;
  }

  // 활성 레인의 시작점에서 종점으로 향하는 단위 방향 벡터를 반환한다.
  Vec2 active_lane_direction() const
  {
    const Vec2 delta = active_lane_finish_ - active_lane_start_;
    const double length = norm(delta);
    return length > 1.0e-9 ? delta * (1.0 / length) : Vec2{};
  }

  // 헤딩 오차가 커질수록 전진 출력을 낮추고 정지 각도 이상에서는 중립을 반환한다.
  int forward_pwm_for_heading(
    const double absolute_heading_error, const int maximum_forward_pwm) const
  {
    if (absolute_heading_error >= lane_forward_stop_heading_rad_) {
      return neutral_pwm_;
    }
    if (absolute_heading_error <= lane_forward_full_heading_rad_) {
      return maximum_forward_pwm;
    }
    const double scale = std::clamp(
      (lane_forward_stop_heading_rad_ - absolute_heading_error) /
      (lane_forward_stop_heading_rad_ - lane_forward_full_heading_rad_),
      0.0, 1.0);
    return static_cast<int>(std::lround(
      lane_forward_slow_pwm_ +
      scale * static_cast<double>(maximum_forward_pwm - lane_forward_slow_pwm_)));
  }

  // 고정 waypoint 대신 현재 위치에서 계속 갱신되는 LOS 목표로 레인을 추종한다.
  void apply_active_lane_los(
    std::array<uint16_t, 18> & channels, const int maximum_forward_pwm)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    const Vec2 target = active_lane_lookahead_target();
    const Vec2 delta = target - current_position_;
    const Vec2 lane_direction = active_lane_direction();
    const double desired_yaw = norm(delta) > 1.0e-9 ?
      std::atan2(delta.y, delta.x) :
      std::atan2(lane_direction.y, lane_direction.x);
    const double heading_error = wrap_pi(desired_yaw - current_yaw_rad_);
    set_heading_control(channels, desired_yaw);
    set_channel(
      channels, forward_channel_,
      forward_pwm_for_heading(std::abs(heading_error), maximum_forward_pwm));
  }

  // 목표 위치를 향해 방향을 맞추고 허용 오차 안에서 전진한다.
  bool follow_waypoint(
    std::array<uint16_t, 18> & channels, const Vec2 & target, const int forward_pwm)
  {
    set_neutral_control(channels);
    hold_mission_depth(channels);
    const Vec2 delta = target - current_position_;
    const double waypoint_distance = norm(delta);
    if (waypoint_distance <= waypoint_reach_tolerance_m_) {
      if (current_horizontal_speed_mps_ > waypoint_settle_speed_mps_) {
        waypoint_settle_started_at_.reset();
        return false;
      }
      if (!waypoint_settle_started_at_) {
        waypoint_settle_started_at_ = now();
        return waypoint_settle_hold_sec_ <= 0.0;
      }
      return (now() - *waypoint_settle_started_at_).seconds() >= waypoint_settle_hold_sec_;
    }
    waypoint_settle_started_at_.reset();

    const double desired_yaw = std::atan2(delta.y, delta.x);
    const double yaw_error = wrap_pi(desired_yaw - current_yaw_rad_);
    set_heading_control(channels, desired_yaw);
    if (!waypoint_heading_aligned_ &&
      std::abs(yaw_error) <= waypoint_heading_tolerance_rad_)
    {
      waypoint_heading_aligned_ = true;
    }
    if (waypoint_heading_aligned_ &&
      std::abs(yaw_error) >= waypoint_heading_release_rad_)
    {
      waypoint_heading_aligned_ = false;
    }
    if (waypoint_heading_aligned_) {
      int commanded_forward_pwm = forward_pwm;
      if (
        waypoint_distance < waypoint_slowdown_distance_m_ &&
        waypoint_slowdown_distance_m_ > waypoint_reach_tolerance_m_)
      {
        const double scale = std::clamp(
          (waypoint_distance - waypoint_reach_tolerance_m_) /
          (waypoint_slowdown_distance_m_ - waypoint_reach_tolerance_m_),
          0.0, 1.0);
        commanded_forward_pwm = static_cast<int>(std::lround(
          lane_forward_slow_pwm_ +
          scale * static_cast<double>(forward_pwm - lane_forward_slow_pwm_)));
      }
      set_channel(channels, forward_channel_, commanded_forward_pwm);
    }
    return false;
  }

  // 목표 방향과 현재 방향의 오차로 회전 채널 출력을 계산한다.
  void set_heading_control(
    std::array<uint16_t, 18> & channels, const double desired_yaw)
  {
    const auto current_time = now();
    const double error = wrap_pi(desired_yaw - current_yaw_rad_);
    double dt = 0.0;
    if (waypoint_pid_initialized_) {
      dt = std::clamp(
        (current_time - waypoint_pid_time_).seconds(), 0.0, 0.2);
      if (dt > 1.0e-6) {
        const double raw_derivative = wrap_pi(error - waypoint_previous_error_) / dt;
        waypoint_error_derivative_ =
          (1.0 - YAW_DERIVATIVE_ALPHA) * waypoint_error_derivative_ +
          YAW_DERIVATIVE_ALPHA * raw_derivative;
      }
    }
    const double candidate_integral = std::clamp(
      waypoint_integral_ + error * dt,
      -waypoint_yaw_integral_limit_, waypoint_yaw_integral_limit_);
    waypoint_pid_time_ = current_time;
    waypoint_previous_error_ = error;
    waypoint_pid_initialized_ = true;

    const double candidate_command =
      waypoint_yaw_kp_ * error +
      waypoint_yaw_ki_ * candidate_integral +
      waypoint_yaw_kd_ * waypoint_error_derivative_;
    const double yaw_limit =
      static_cast<double>(max_waypoint_yaw_delta_pwm_) / rc_pwm_span_;
    if (std::abs(candidate_command) <= yaw_limit || candidate_command * error < 0.0) {
      waypoint_integral_ = candidate_integral;
    }
    double command =
      waypoint_yaw_kp_ * error +
      waypoint_yaw_ki_ * waypoint_integral_ +
      waypoint_yaw_kd_ * waypoint_error_derivative_;
    if (waypoint_yaw_invert_) {
      command = -command;
    }
    const int delta_pwm = std::clamp(
      static_cast<int>(std::lround(command * rc_pwm_span_)),
      -max_waypoint_yaw_delta_pwm_, max_waypoint_yaw_delta_pwm_);
    set_channel(channels, yaw_channel_, neutral_pwm_ + delta_pwm);
  }

  // 화면 오차와 수심 오차를 결합해 부표 추적 출력을 계산한다.
  void apply_visual_tracking(
    std::array<uint16_t, 18> & channels, const Detection & detection,
    const int forward_pwm, const double target_x, const double target_y)
  {
    set_neutral_control(channels);
    const auto [error_x, error_y] =
      normalized_error(detection, target_x, target_y);

    const double yaw_sign = vision_yaw_invert_ ? -1.0 : 1.0;
    const int yaw_delta_pwm = std::clamp(
      static_cast<int>(std::lround(yaw_sign * error_x * vision_yaw_kp_pwm_)),
      -max_vision_yaw_delta_pwm_, max_vision_yaw_delta_pwm_);
    set_channel(channels, yaw_channel_, neutral_pwm_ + yaw_delta_pwm);

    const double vertical_sign = vertical_positive_is_up_ ? -1.0 : 1.0;
    const int vision_throttle = neutral_pwm_ + static_cast<int>(
      vertical_sign * error_y *
      static_cast<double>(max_vision_throttle_delta_pwm_));
    const int depth_throttle = mission_depth_pwm();
    const double weight = approach_vision_throttle_weight_;
    const int blended_throttle = static_cast<int>(std::lround(
      weight * static_cast<double>(vision_throttle) +
      (1.0 - weight) * static_cast<double>(depth_throttle)));
    set_channel(channels, throttle_channel_, blended_throttle);
    set_channel(channels, forward_channel_, forward_pwm);
  }

  // 검출 중심과 목표점의 차이를 정규화된 화면 오차로 변환한다.
  std::pair<double, double> normalized_error(
    const Detection & detection, const double target_x, const double target_y) const
  {
    const double x = detection.center_x / detection.image_width;
    const double y = detection.center_y / detection.image_height;
    return {
      std::clamp(2.0 * (x - target_x), -1.0, 1.0),
      std::clamp(2.0 * (y - target_y), -1.0, 1.0)};
  }

  // 부표 상자가 전체 화면에서 차지하는 면적 비율을 계산한다.
  double detection_area_ratio(const Detection & detection) const
  {
    return static_cast<double>(detection.width * detection.height) /
           static_cast<double>(detection.image_width * detection.image_height);
  }

  // 주 수심 입력을 우선하고 유효하지 않으면 보조 입력을 반환한다.
  std::optional<double> current_depth() const
  {
    const auto current_time = now();
    if (
      pose_depth_ &&
      (current_time - pose_depth_->received_at).seconds() <= depth_timeout_sec_)
    {
      return pose_depth_->depth_m;
    }
    if (
      scalar_depth_ &&
      (current_time - scalar_depth_->received_at).seconds() <= depth_timeout_sec_)
    {
      return scalar_depth_->depth_m;
    }
    return std::nullopt;
  }

  // 저장된 목표수심과 현재수심의 차이로 수직 채널 출력을 계산한다.
  int mission_depth_pwm()
  {
    const auto depth = current_depth();
    if (!depth || !mission_hold_depth_m_) {
      return neutral_pwm_;
    }

    const auto current_time = now();
    const double error = *mission_hold_depth_m_ - *depth;
    double dt = 0.0;
    double derivative = 0.0;
    if (depth_pid_initialized_) {
      dt = std::clamp((current_time - depth_pid_time_).seconds(), 0.0, 0.2);
      if (dt > 1.0e-6) {
        derivative = (error - depth_previous_error_) / dt;
      }
    }
    depth_integral_ = std::clamp(
      depth_integral_ + error * dt,
      -depth_integral_limit_m_sec_, depth_integral_limit_m_sec_);
    depth_pid_time_ = current_time;
    depth_previous_error_ = error;
    depth_pid_initialized_ = true;

    int delta = static_cast<int>(std::lround(
      depth_kp_pwm_per_m_ * error +
      depth_ki_pwm_per_m_sec_ * depth_integral_ +
      depth_kd_pwm_sec_per_m_ * derivative));
    delta += buoyancy_hold_delta_pwm_;
    delta = std::clamp(delta, -max_depth_delta_pwm_, max_depth_delta_pwm_);
    const int direction = vertical_positive_is_up_ ? -1 : 1;
    return neutral_pwm_ + direction * delta;
  }

  // 현재 채널 배열에 목표수심 유지 출력을 적용한다.
  void hold_mission_depth(std::array<uint16_t, 18> & channels)
  {
    set_channel(channels, throttle_channel_, mission_depth_pwm());
  }

  // 활성 레인의 시작점 기준 현재 진행 거리를 반환한다.
  double active_lane_progress() const
  {
    if (!active_lane_index_) {
      return 0.0;
    }
    return lane_planner_->progress_from_start(
      current_position_, *active_lane_index_, active_lane_start_at_a_);
  }

  // 검출 정보가 존재하고 허용 시간 안에 수신되었는지 확인한다.
  bool recent(const std::optional<Detection> & detection) const
  {
    return detection &&
           (now() - detection->received_at).seconds() <= detection_timeout_sec_;
  }

  // 최근 부표의 연속 검출 횟수가 확정 기준을 만족하는지 확인한다.
  bool confirmed_buoy() const
  {
    return recent(buoy_) && buoy_->consecutive_hits >= target_confirm_hits_;
  }

  // 위치 정보가 존재하고 허용 시간 안에 수신되었는지 확인한다.
  bool have_recent_odometry() const
  {
    return have_odometry_ &&
           (now() - odometry_received_at_).seconds() <= odometry_timeout_sec_;
  }

  // 현재 상태에 진입한 뒤 지난 시간을 초 단위로 반환한다.
  double state_age_sec() const
  {
    return (now() - state_entered_at_).seconds();
  }

  // 상태와 진입 시각을 갱신하고 변경 내용을 발행한다.
  void transition_to(const State next, const std::string & reason)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "State %s -> %s: %s",
      state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    state_entered_at_ = now();
    if (next == State::APPROACH_BUOY) {
      approach_close_started_at_.reset();
    }
    publish_state();
  }

  // 내부 상태 값을 외부에 표시할 문자열로 변환한다.
  static const char * state_name(const State state)
  {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::TARGET_CONFIRM: return "TARGET_CONFIRM";
      case State::WAIT_CONTROL_GRANT: return "WAIT_CONTROL_GRANT";
      case State::INITIAL_SCAN_360: return "INITIAL_SCAN_360";
      case State::TURN_TO_INITIAL_TARGET: return "TURN_TO_INITIAL_TARGET";
      case State::TARGET_HOLD: return "TARGET_HOLD";
      case State::REACQUIRE_BUOY: return "REACQUIRE_BUOY";
      case State::APPROACH_BUOY: return "APPROACH_BUOY";
      case State::STRONG_FORWARD: return "STRONG_FORWARD";
      case State::STRONG_BACKOFF: return "STRONG_BACKOFF";
      case State::MOVE_TO_LANE_START: return "MOVE_TO_LANE_START";
      case State::LANE_FOLLOWING_WITH_SEARCH: return "LANE_FOLLOWING_WITH_SEARCH";
      case State::RETURN_TO_ACTIVE_LANE: return "RETURN_TO_ACTIVE_LANE";
      case State::COMPLETE: return "COMPLETE";
      case State::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  // 현재 상태 문자열을 상태 토픽으로 발행한다.
  void publish_state()
  {
    if (!state_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    state_pub_->publish(msg);
  }

  // 부표 확정 여부를 제어권 인계용 토픽으로 발행한다.
  void publish_target_confirmed(const bool confirmed)
  {
    if (!target_confirmed_pub_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = confirmed;
    target_confirmed_pub_->publish(msg);
  }

  // 모든 채널을 변경하지 않음 값으로 채운 배열을 만든다.
  std::array<uint16_t, 18> nochange_channels() const
  {
    std::array<uint16_t, 18> channels{};
    channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    return channels;
  }

  // 제어하는 세 채널을 중립 출력으로 설정한다.
  void set_neutral_control(std::array<uint16_t, 18> & channels) const
  {
    set_channel(channels, throttle_channel_, neutral_pwm_);
    set_channel(channels, yaw_channel_, neutral_pwm_);
    set_channel(channels, forward_channel_, neutral_pwm_);
  }

  // 제어하는 세 채널의 강제 출력을 해제한다.
  void release_controlled_channels(std::array<uint16_t, 18> & channels) const
  {
    set_channel(channels, throttle_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, yaw_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, forward_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }

  // 채널 번호를 배열 위치로 바꾸고 출력 범위를 제한해 저장한다.
  void set_channel(
    std::array<uint16_t, 18> & channels, const int channel, int pwm) const
  {
    if (
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE &&
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE)
    {
      pwm = std::clamp(pwm, min_pwm_, max_pwm_);
    }
    channels[static_cast<std::size_t>(channel - 1)] = static_cast<uint16_t>(pwm);
  }

  // 같은 채널 명령을 실제 제어 토픽과 확인 토픽에 발행한다.
  void publish_channels(const std::array<uint16_t, 18> & channels)
  {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels = channels;
    rc_pub_->publish(msg);
    rc_monitor_pub_->publish(msg);
  }

  // 위치 이동에 사용하는 방향 제어 누적값을 초기화한다.
  void reset_waypoint_pid()
  {
    waypoint_pid_initialized_ = false;
    waypoint_heading_aligned_ = false;
    waypoint_integral_ = 0.0;
    waypoint_previous_error_ = 0.0;
    waypoint_error_derivative_ = 0.0;
    waypoint_settle_started_at_.reset();
  }

  // 수심 유지에 사용하는 제어 누적값을 초기화한다.
  void reset_depth_pid()
  {
    depth_pid_initialized_ = false;
    depth_integral_ = 0.0;
    depth_previous_error_ = 0.0;
  }

  std::string bbox_topic_;
  std::string depth_topic_;
  std::string depth_pose_topic_;
  std::string odometry_topic_;
  std::string start_frame_topic_;
  std::string vision_search_request_topic_;
  std::string target_confirmed_topic_;
  std::string vision_control_granted_topic_;
  std::string emergency_stop_topic_;
  std::string state_topic_;
  std::string rc_override_topic_;
  std::string rc_monitor_topic_;

  double arena_length_m_{15.0};
  double arena_width_m_{16.0};
  double arena_offset_x_m_{-0.3};
  double arena_offset_y_m_{0.3};
  double arena_safety_margin_m_{0.5};
  std::string arena_start_corner_{"bottom_left"};
  double lane_search_offset_m_{2.0};
  double incapable_skip_distance_m_{2.0};
  double waypoint_reach_tolerance_m_{0.15};

  double control_rate_hz_{20.0};
  double odometry_timeout_sec_{0.5};
  double detection_timeout_sec_{1.0};
  double depth_timeout_sec_{1.0};
  double depth_pose_scale_{-1.0};
  double depth_pose_offset_m_{0.0};
  double max_depth_m_{10.5};
  int buoy_class_id_{0};
  int target_confirm_hits_{3};
  double target_confirm_sec_{0.2};
  double buoy_confidence_similar_delta_{0.05};
  double buoy_same_target_center_ratio_{0.12};
  double lpf_tau_sec_{0.3};
  double initial_search_radius_m_{2.0};
  int initial_scan_yaw_pwm_{1560};
  double initial_scan_completion_tolerance_rad_{0.15};
  double initial_target_reacquire_timeout_sec_{5.0};
  int reacquire_yaw_pwm_{1470};
  double reacquire_yaw_duration_sec_{0.5};
  double reacquire_timeout_sec_{1.0};
  double approach_target_x_{0.50};
  double approach_target_y_{0.30};
  double approach_area_ratio_{0.03};
  double approach_close_hold_sec_{2.0};
  int strong_forward_pwm_{1700};
  double strong_forward_duration_sec_{5.0};
  int strong_backoff_pwm_{1300};
  double strong_backoff_duration_sec_{3.0};

  int throttle_channel_{3};
  int yaw_channel_{4};
  int forward_channel_{5};
  int neutral_pwm_{1500};
  int min_pwm_{1300};
  int max_pwm_{1700};
  double rc_pwm_span_{400.0};
  int lane_forward_pwm_{1680};
  int lane_forward_slow_pwm_{1560};
  double lane_lookahead_distance_m_{1.0};
  double lane_rejoin_cross_track_tolerance_m_{0.25};
  double lane_rejoin_heading_tolerance_rad_{0.2618};
  double lane_forward_full_heading_rad_{0.1745};
  double lane_forward_stop_heading_rad_{0.5236};
  int lane_transfer_forward_pwm_{1680};
  bool lane_transfer_dynamic_rejoin_{true};
  double lane_end_transition_distance_m_{3.0};
  double lane_transfer_slowdown_distance_m_{0.75};
  double lane_transfer_heading_tolerance_rad_{0.0873};
  double lane_transfer_heading_hold_sec_{0.3};
  double waypoint_slowdown_distance_m_{0.25};
  double waypoint_settle_speed_mps_{0.15};
  double waypoint_settle_hold_sec_{0.3};
  int approach_forward_pwm_{1560};
  int approach_forward_max_pwm_{1700};
  double waypoint_heading_tolerance_rad_{0.1745};
  double waypoint_heading_release_rad_{0.3491};
  double waypoint_yaw_kp_{0.8};
  double waypoint_yaw_ki_{0.1};
  double waypoint_yaw_kd_{0.02};
  double waypoint_yaw_integral_limit_{0.7};
  int max_waypoint_yaw_delta_pwm_{160};
  bool waypoint_yaw_invert_{true};
  double vision_yaw_kp_pwm_{100.0};
  int max_vision_yaw_delta_pwm_{100};
  int max_vision_throttle_delta_pwm_{100};
  bool vision_yaw_invert_{false};
  bool vertical_positive_is_up_{true};
  double approach_vision_throttle_weight_{0.4};
  double depth_kp_pwm_per_m_{120.0};
  double depth_ki_pwm_per_m_sec_{20.0};
  double depth_kd_pwm_sec_per_m_{16.0};
  double depth_integral_limit_m_sec_{2.0};
  int buoyancy_hold_delta_pwm_{40};
  int max_depth_delta_pwm_{160};

  ArenaFrameTransform arena_transform_;
  std::unique_ptr<LanePlanner> lane_planner_;
  State state_{State::IDLE};
  TargetContext target_context_{TargetContext::INITIAL};
  rclcpp::Time state_entered_at_{0, 0, RCL_ROS_TIME};
  bool vision_has_control_{false};
  bool have_odometry_{false};
  Vec2 current_position_{};
  double current_yaw_rad_{0.0};
  double current_horizontal_speed_mps_{0.0};
  rclcpp::Time odometry_received_at_{0, 0, RCL_ROS_TIME};
  std::optional<DepthSample> pose_depth_;
  std::optional<DepthSample> scalar_depth_;
  std::optional<double> mission_hold_depth_m_;
  std::optional<Detection> buoy_;
  std::optional<rclcpp::Time> target_confirm_started_at_;
  std::optional<rclcpp::Time> target_hold_confirm_started_at_;
  std::optional<rclcpp::Time> approach_close_started_at_;
  std::optional<rclcpp::Time> detection_confirm_started_at_;

  Vec2 handoff_position_{};
  std::optional<ScanCandidate> scan_candidate_;
  double scan_previous_yaw_rad_{0.0};
  double scan_accumulated_yaw_rad_{0.0};
  double initial_target_yaw_rad_{0.0};

  std::vector<bool> lane_completed_;
  std::optional<std::size_t> active_lane_index_;
  bool active_lane_start_at_a_{true};
  Vec2 active_lane_start_{};
  Vec2 active_lane_finish_{};
  Vec2 waypoint_{};
  bool lane_transfer_enabled_{false};
  LaneTransferPhase lane_transfer_phase_{LaneTransferPhase::ALIGN_TRANSFER};
  Vec2 lane_transfer_start_{};
  std::optional<rclcpp::Time> lane_transfer_heading_stable_started_at_;
  double target_departure_progress_m_{0.0};
  std::optional<double> ignore_detections_until_progress_m_;

  bool waypoint_pid_initialized_{false};
  bool waypoint_heading_aligned_{false};
  double waypoint_integral_{0.0};
  double waypoint_previous_error_{0.0};
  double waypoint_error_derivative_{0.0};
  std::optional<rclcpp::Time> waypoint_settle_started_at_;
  rclcpp::Time waypoint_pid_time_{0, 0, RCL_ROS_TIME};
  bool depth_pid_initialized_{false};
  double depth_integral_{0.0};
  double depth_previous_error_{0.0};
  rclcpp::Time depth_pid_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    depth_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_frame_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_search_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_control_granted_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_monitor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_confirmed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace auv_lane_vision_control

// 통신을 초기화하고 제어 노드를 실행한 뒤 종료 시 채널을 해제한다.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auv_lane_vision_control::LaneVisionControllerNode>();
  rclcpp::spin(node);
  node->publish_release_once();
  rclcpp::shutdown();
  return 0;
}
