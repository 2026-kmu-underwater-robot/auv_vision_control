# AUV Vision Control

Hydrophone 제어기로부터 제어권을 인계받은 뒤, 초기 부표를 처리하고 수조 좌표계에
자동 생성한 레인을 따라가며 남은 부표를 탐색·제거하는 ROS 2 패키지입니다.

이 패키지는 기존 `auv_buoy_vision_control`과 동일한 Vision/RC 토픽을 사용하는
교체형 제어기입니다. 두 제어기를 동시에 실행하면 `/mavros/rc/override`가
충돌하므로 반드시 하나만 실행해야 합니다.

## 전제 조건

- ROS 2 Humble과 MAVROS가 설치되어 있어야 합니다.
- `/start_frame`은 `hydrophone_ctrl`의 시작 좌표 규약으로 발행되어야 합니다.
- `/odometry/filtered`에서 유효한 위치와 yaw를 받을 수 있어야 합니다.
- 포함된 YOLO 검출기가 `/vision/buoy_bbox`에 buoy bbox를 발행합니다.
- 실제 RC 채널 방향과 PWM은 낮은 추력에서 먼저 확인해야 합니다.
- 이 제어기는 `COMPLETE`에서 자동 부상하지 않고 RC override를 해제합니다.

## 전체 동작

```text
IDLE
  -> TARGET_CONFIRM
  -> WAIT_CONTROL_GRANT
  -> (확정 buoy 있음) TARGET_HOLD
  -> (확정 buoy 없음) INITIAL_SCAN_360
       -> 가장 큰 bbox 후보 선택
       -> TURN_TO_INITIAL_TARGET
       -> TARGET_HOLD
  -> TARGET_HOLD
       -> 안정 검출: APPROACH_BUOY
       -> bbox 유실: REACQUIRE_BUOY
          -> 재검출: TARGET_HOLD
          -> 재검출 실패: 레인 진행/복귀
  -> APPROACH_BUOY
       -> bbox 면적비 0.03 이상을 2초 연속 유지
  -> STRONG_FORWARD
  -> STRONG_BACKOFF
       -> 가장 가까운 레인 끝점으로 이동
  -> MOVE_TO_LANE_START
  -> LANE_FOLLOWING_WITH_SEARCH
       -> buoy 발견: TARGET_HOLD → APPROACH_BUOY → STRONG_FORWARD → STRONG_BACKOFF
       -> 처리/포기 후 RETURN_TO_ACTIVE_LANE
       -> 기존 레인 주행 재개
  -> 모든 레인 완료
  -> COMPLETE
```

odom·수심 유실, 최대수심 초과 또는 비상정지 시 `FAILSAFE`로 전환합니다.

## Hydrophone handshake

```text
/homing/vision_search_active
    Hydrophone -> Vision 탐색/확인 요청

/vision/target_confirmed
    Vision -> Hydrophone 안정적인 buoy 검출 알림

/homing/vision_control_granted
    Hydrophone -> Vision RC 제어권 승인
```

승인 전에는 RC override를 발행하지 않습니다.

Hydrophone의 수조 경계 또는 timeout으로 buoy 확정 없이 제어권을 받으면 제자리에서
360도 회전합니다. 회전 중 관측한 buoy 중 bbox 면적이 가장 큰 대상을 가장 가까운
대상으로 선택합니다. 후보가 없으면 기다리지 않고 가장 가까운 레인 끝점으로
이동합니다.

## 수조 좌표계

좌표 변환은 `hydrophone_ctrl`과 동일합니다.

1. `/start_frame`의 위치와 yaw를 arena 원점과 `+X` 방향으로 사용합니다.
2. `/odometry/filtered`의 위치와 yaw를 arena 좌표로 변환합니다.
3. `arena_start_corner=bottom_left`이면 수조 내부 너비 방향은 `-Y`입니다.
4. 레인은 arena `X`축과 평행하게 생성됩니다.

대회장 기본 설정:

```text
arena_length_m       = 15.0
arena_width_m        = 16.0
arena_offset_x_m     = -0.3
arena_offset_y_m     = 0.3
arena_safety_margin_m = 0.5
arena_start_corner   = bottom_left
```

## 레인 자동 생성

```text
usable_width  = arena_width - 2 * arena_safety_margin
coverage_width = 2 * lane_search_offset
lane_count    = ceil(usable_width / coverage_width)
actual_spacing = usable_width / lane_count
```

기본 `lane_search_offset_m=2.0`일 때:

```text
usable_width  = 15.0 m
lane_count    = 4
actual_spacing = 3.75 m
```

초기 부표 처리가 끝나면 모든 미완료 레인의 끝점 중 현재 위치에서 가장 가까운
끝점을 선택합니다. 한 레인을 완료하면 다시 가장 가까운 미완료 레인 끝점을
선택합니다.

## 부표 접근과 판정

대상을 발견하면 먼저 `TARGET_HOLD`에서 yaw/forward를 중립으로 두고 안정 검출을
확인합니다. bbox가 끊기면 `REACQUIRE_BUOY`에서 yaw `1470`을 0.5초 보낸 뒤,
남은 시간은 yaw 중립으로 유지하며 총 1초 동안 재검출을 기다립니다.

`APPROACH_BUOY`는 buoy bbox를 화면 목표점 `(0.50, 0.30)`으로 추적하며, bbox가
전체 화면의 `approach_area_ratio` 이상을 차지할 때까지 면적 기반으로 감속 전진합니다.
목표 면적에 도달하면 전진 채널을 중립으로 두되 목표점 정렬과 수심 제어는 계속합니다.
목표 면적을 `approach_close_hold_sec` 동안 연속 유지하면 `STRONG_FORWARD`로
전환합니다. 유지 중 면적이 기준 아래로 내려가면 2초 타이머는 처음부터 다시 셉니다.

접근 허용 범위:

- 초기 부표: Hydrophone 제어권 인계 위치로부터 반경 `2.0 m`
- 레인 탐색 부표: 현재 활성 레인으로부터 수직 거리 `2.0 m`
- arena 안전 경계 밖으로 이동할 수 없음

허용 범위 끝까지 접근해도 목표 면적에 도달하지 못하면 `INCAPABLE`로 판단합니다.
활성 레인의 가장 가까운 지점으로 복귀한 뒤, 기존 진행 방향으로 `2.0 m` 이동할
때까지 bbox를 무시해 같은 부표를 반복 추적하지 않습니다.

## 강한 전진과 후진

기본 PWM과 지속 시간:

| 동작 | PWM | 시간 |
|---|---:|---:|
| 강한 전진 | 1700 | 5.0 s |
| 강한 후진 | 1300 | 3.0 s |

강한 전진과 강한 후진을 각각 한 번 수행한 뒤 초기 대상이면 다음 레인으로,
레인 주행 중 대상이면 기존 활성 레인으로 복귀합니다.

## 수심 제어

수심 입력 우선순위:

1. `/depth/pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
2. `/auv/depth` (`std_msgs/msg/Float64`)

기본 Pose 변환:

```text
depth = -pose.position.z
```

`depth_pose_topic`을 빈 문자열로 설정하면 Pose 입력을 비활성화할 수 있습니다.

제어권 승인 순간의 현재 수심을 `mission_hold_depth`로 저장하고 전체 미션 동안
유지합니다.

- 레인 이동·회전·강한 전진·후진: 수심 PID만 사용
- `APPROACH_BUOY`: 비전 상하 오차 40% + 수심 PID 60%
- 양성부력 보상과 PID 출력 제한 적용

## ROS 토픽

### 입력

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/vision/buoy_bbox` | `std_msgs/msg/Float32MultiArray` | YOLO buoy bbox |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 위치와 yaw |
| `/start_frame` | `geometry_msgs/msg/PoseStamped` | arena 시작 좌표 |
| `/depth/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 주 수심 입력 |
| `/auv/depth` | `std_msgs/msg/Float64` | 보조 수심 입력 |
| `/homing/vision_search_active` | `std_msgs/msg/Bool` | Vision 확인 요청 |
| `/homing/vision_control_granted` | `std_msgs/msg/Bool` | RC 제어권 승인 |
| `/mission/emergency_stop` | `std_msgs/msg/Bool` | 비상정지 |

### 출력

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/vision/target_confirmed` | `std_msgs/msg/Bool` | 안정적인 buoy 확정 |
| `/mission/state` | `std_msgs/msg/String` | 현재 상태, `COMPLETE` 포함 |
| `/mavros/rc/override` | `mavros_msgs/msg/OverrideRCIn` | 실제 RC override |
| `/mission/rc_command` | `mavros_msgs/msg/OverrideRCIn` | 모니터링용 동일 RC |

### bbox 형식

검출 하나당 10개 값입니다.

```text
[stamp_sec, detected, class_id, confidence,
 center_x, center_y, width, height, image_width, image_height]
```

`detected >= 0.5`이고 `class_id == buoy_class_id`인 검출만 사용합니다.

여러 부표가 동시에 보이면 `auv_test_vision`과 같은 순서로 타깃을 고릅니다.

1. bbox 면적이 다르면 항상 면적이 큰 부표
2. 면적이 같고 confidence 차이가 `buoy_confidence_similar_delta`보다 크면
   confidence가 높은 부표
3. 면적이 같고 confidence도 비슷하면 화면에서 더 오른쪽에 있는 부표

탐색·정지 확인·재탐색 중에는 위 조건에 따라 더 좋은 후보로 바꿀 수 있습니다.
`APPROACH_BUOY` 이후에는 중심 위치가 같은 타깃으로 판정되는 bbox만 갱신하므로,
접근 도중 다른 부표로 타깃이 바뀌지 않습니다.

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `lane_search_offset_m` | 2.0 | 활성 레인 좌우 접근 한계와 레인 계산 기준 |
| `incapable_skip_distance_m` | 2.0 | 포기 후 bbox 무시 주행 거리 |
| `initial_search_radius_m` | 2.0 | 초기 인계 위치 기준 접근 반경 |
| `initial_scan_yaw_pwm` | 1560 | 초기 부표가 없을 때 360도 탐색 Yaw PWM |
| `approach_target_x` | 0.50 | APPROACH에서 추종할 buoy 중심 X |
| `approach_target_y` | 0.30 | APPROACH에서 추종할 buoy 중심 Y |
| `approach_area_ratio` | 0.03 | 전진을 멈추고 근접 정렬을 시작할 bbox 면적비 |
| `approach_close_hold_sec` | 2.0 | 목표 면적을 연속 유지한 뒤 강한 전진할 시간 |
| `approach_forward_max_pwm` | 1700 | 멀리 있는 buoy 접근 최대 PWM |
| `approach_forward_pwm` | 1560 | 가까운 buoy 접근 최소 PWM |
| `strong_forward_pwm` | 1700 | 근접 정렬 완료 후 강한 전진 PWM |
| `strong_forward_duration_sec` | 5.0 | 강한 전진 시간 |
| `strong_backoff_pwm` | 1300 | 강한 후진 PWM |
| `strong_backoff_duration_sec` | 3.0 | 강한 후진 시간 |
| `vision_yaw_kp_pwm` | 100.0 | 정규화 수평 오차 1.0당 Yaw PWM gain |
| `max_vision_yaw_delta_pwm` | 100 | 중립 기준 Yaw 보정량 제한 |
| `reacquire_yaw_pwm` | 1470 | 부표 유실 뒤 재탐색 yaw PWM |
| `reacquire_yaw_duration_sec` | 0.5 | 위 yaw를 유지하는 시간 |
| `reacquire_timeout_sec` | 1.0 | 재검출 실패 시 레인 진행/복귀까지의 시간 |
| `target_confirm_hits` | 3 | 타깃 확정에 필요한 연속 검출 횟수 |
| `target_confirm_sec` | 0.2 | 연속 검출 후 추가 유지 시간 |
| `buoy_confidence_similar_delta` | 0.05 | 두 confidence를 비슷하다고 보는 차이 |
| `buoy_same_target_center_ratio` | 0.12 | 같은 타깃으로 보는 축별 중심 거리 비율 |
| `lane_forward_pwm` | 1680 | LOS 레인 추종/waypoint 최대 전진 PWM |
| `lane_forward_slow_pwm` | 1560 | LOS 정렬 및 waypoint 감속 최소 전진 PWM |
| `lane_lookahead_distance_m` | 1.0 | 현재 레인 투영점 앞의 LOS 목표 거리 |
| `lane_rejoin_cross_track_tolerance_m` | 0.25 | 동적 레인 복귀 완료 횡오차 |
| `lane_rejoin_heading_tolerance_rad` | 0.2618 | 동적 레인 복귀 완료 헤딩 오차 |
| `lane_forward_full_heading_rad` | 0.1745 | 최대 레인 전진을 허용하는 헤딩 오차 |
| `lane_forward_stop_heading_rad` | 0.5236 | 레인 전진을 중단하는 헤딩 오차 |
| `lane_transfer_forward_pwm` | 1680 | 다음 레인 동적 LOS 합류 최대 전진 PWM |
| `lane_transfer_slowdown_distance_m` | 0.75 | 다음 레인 시작점 접근 감속 거리 |
| `lane_transfer_heading_tolerance_rad` | 0.0873 | 레인 간 이동 전후 제자리 정렬 허용 오차 |
| `lane_transfer_heading_hold_sec` | 0.3 | 정렬 완료 전 헤딩 안정 유지 시간 |
| `lane_transfer_dynamic_rejoin` | true | 다음 레인의 앞쪽 LOS로 바로 합류하여 두 번째 제자리 yaw 정렬 생략 |
| `lane_end_transition_distance_m` | 3.0 | 현재 레인 종점까지 이 거리만큼 남으면 다음 레인 동적 합류 시작; 0이면 종점까지 주행 |
| `odometry_timeout_sec` | 0.5 | odom FAILSAFE 시간 |
| `depth_timeout_sec` | 1.0 | 수심 FAILSAFE 시간 |
| `max_depth_m` | 10.5 | 최대 허용 수심 |
| `approach_vision_throttle_weight` | 0.4 | 접근 중 비전 throttle 비중 |

모든 RC 출력은 기본적으로 `1300~1700` 범위로 제한됩니다.

## 빌드

```bash
cd ~/auv_ws
colcon build --packages-select auv_vision_control --symlink-install
source install/setup.bash
```

## 실행

IMX219 카메라 두 대, 원본 영상 H.264 WebRTC, YOLO 검출 영상 H.264
WebRTC, 레인 제어기를 기본 config로 함께 실행합니다.

```bash
ros2 launch auv_vision_control vision_control.launch.py
```

기본 포트는 YOLO 검출 영상 `8090`, camera0 원본 `8091`, camera1 원본
`8092`입니다. 이미 카메라 노드를 따로 실행 중이면 중복 실행을 막기 위해 다음처럼
카메라 시작만 비활성화합니다.

```bash
ros2 launch auv_vision_control vision_control.launch.py start_imx219:=false
```

다른 모델이나 config를 사용하려면 launch 인자로 지정합니다.

```bash
ros2 launch auv_vision_control vision_control.launch.py \
  model_path:=/absolute/path/to/best.pt \
  config_file:=/absolute/path/to/vision_control.yaml
```

수조 좌우 방향 설정은 hydrophone_ctrl과 자동으로 공유되지 않습니다. 두 패키지가
같은 프로필을 사용하도록 비전 launch의 `arena_config_file`을 명시합니다.
`bottom_left`는 시작 좌표계의 -Y 방향이 수조 안쪽이고,
`bottom_right`는 +Y 방향이 수조 안쪽입니다.

```bash
# bottom_left (기본값)
ros2 launch auv_vision_control vision_control.launch.py \
  arena_config_file:=$(ros2 pkg prefix auv_vision_control)/share/auv_vision_control/config/arena_bottom_left.yaml

# bottom_right
ros2 launch auv_vision_control vision_control.launch.py \
  arena_config_file:=$(ros2 pkg prefix auv_vision_control)/share/auv_vision_control/config/arena_bottom_right.yaml
```

대회장 프로필의 수조 치수는 hydrophone_ctrl과 동일하게
`15.0 m x 16.0 m`, X 오프셋 `-0.3 m`, 안전 여유 `0.5 m`로 두었습니다.
Y 오프셋은 hydrophone_ctrl의 규칙에 맞춰 `bottom_left=+0.3 m`,
`bottom_right=-0.3 m`입니다.

카메라와 YOLO 없이 최신 제어기만 실행할 때는 전용 launch를 사용합니다. 모든
controller 파라미터를 launch argument로 변경할 수 있습니다.

```bash
ros2 launch auv_vision_control lane_vision_controller.launch.py
ros2 launch auv_vision_control lane_vision_controller.launch.py --show-args
```

전용 controller launch의 파라미터 변경 예:

```bash
ros2 launch auv_vision_control lane_vision_controller.launch.py \
  initial_scan_yaw_pwm:=1560 \
  approach_target_x:=0.50 \
  approach_target_y:=0.30 \
  approach_area_ratio:=0.03 \
  strong_forward_duration_sec:=5.0 \
  strong_backoff_duration_sec:=3.0 \
  vision_yaw_kp_pwm:=100.0
```

YOLO만 실행하거나 카메라 두 대를 H.264로 전송할 수도 있습니다.

```bash
ros2 launch auv_vision_control laptop_yolo_detection.launch.py
ros2 launch auv_vision_control dual_imx219_h264.launch.py
```

기본 설정 파일은 `config/vision_control.yaml`입니다. 검출기는
`target_class_id=0`, `publish_per_class=false`로 설정해 buoy 하나만
`/vision/buoy_bbox`에 내보냅니다.

상태 확인:

```bash
ros2 topic echo /mission/state
ros2 topic echo /vision/target_confirmed
ros2 topic echo /mission/rc_command
```

## 안전 주의사항

- 실제 장비에서 실행하기 전에 `waypoint_yaw_invert`, `vision_yaw_invert`,
  `vertical_positive_is_up` 방향을 확인합니다.
- 처음에는 낮은 approach/strong-forward PWM으로 시험합니다.
- `/start_frame`, odom, depth, bbox 수신 상태를 확인한 뒤 제어권을 승인합니다.
- `FAILSAFE`와 `COMPLETE`에서는 throttle/yaw/forward 채널에
  `CHAN_RELEASE`를 발행합니다.
