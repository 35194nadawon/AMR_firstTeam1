# ROS Interfaces

이 문서는 팀 간 충돌을 줄이기 위한 공통 계약입니다. 현재 프로젝트는 역할 분리를 명확히 하기 위해 `/hide/*`와 `/guide/*` 네임스페이스를 사용합니다.

## 네임스페이스 원칙

| Namespace | 담당 | 설명 |
| --- | --- | --- |
| `/guide/*` | 안내팀 | 음성/LLM 명령, 점자 블록 추종, 안내 주행, 목적지 도착 처리 |
| `/hide/*` | 숨는팀 | 위장 대기, 사람 감지, 시야각 회피, Freeze, 복귀, 도킹 |
| `/mcu/*` | MCU/하드웨어 | IMU, OLED, LED, 모터 락, 하드웨어 출력 |
| `/dashboard/*` | 웹 대시보드 | 웹 UI 이벤트, 경고, 데모 상태 표시 |
| Nav2 기본 토픽 | 공통 | `/cmd_vel`, `/goal_pose`, `/navigate_to_pose`, `/scan`, `/tf` |

## 안내팀 토픽

| Topic | Type | Publisher | Subscriber | 설명 |
| --- | --- | --- | --- | --- |
| `/guide/command` | `std_msgs/String` | dashboard/app/llm | guide mission | `start_guide`, `resume`, `arrived`, `return_home` |
| `/guide/state` | `std_msgs/String` | guide mission | dashboard, hide | 안내 상태: `IDLE`, `GUIDE_READY`, `GUIDING`, `ARRIVED`, `DONE` |
| `/guide/goal_pose` | `geometry_msgs/PoseStamped` | guide mission/nav | Nav2 bridge | 안내 목적지 |
| `/guide/yellow_line_error` | `geometry_msgs/Pose2D` | guide vision | guide nav | 점자 블록 중심선 기준 `x`, `y`, `theta` 오차 |
| `/guide/mission_done` | `std_msgs/Bool` | guide mission | hide mission | 안내 완료 후 숨는팀 복귀 시작 트리거 |

## 숨는팀 토픽

| Topic | Type | Publisher | Subscriber | 설명 |
| --- | --- | --- | --- | --- |
| `/hide/takeover_start` | `std_msgs/Bool` | guide/llm/dashboard | hide mission | 대기 상태 로봇 기상/안내 교대 시작 |
| `/hide/state` | `std_msgs/String` | hide mission | dashboard, guide, mcu | 숨는팀 상태: `IDLE_DOCKED`, `WAKE`, `FREEZE`, `RETURN`, `DOCKING` |
| `/hide/person_detected` | `std_msgs/Bool` | hide vision | hide mission/nav | 사람 감지 여부 |
| `/hide/person_pose` | `geometry_msgs/PoseStamped` | hide vision | hide nav | `map` 또는 `base_link` 기준 사람 위치 |
| `/hide/visibility_zone` | `geometry_msgs/PolygonStamped` | hide nav/perception | RViz/Nav2 layer | 사람 시야각 120도 위험 구역 |
| `/hide/freeze` | `std_msgs/Bool` | hide mission/nav | guide nav, mcu | 즉시 정지/대기 필요 여부 |
| `/hide/apriltag_pose` | `geometry_msgs/PoseStamped` | hide docking vision | hide docking | 대기석 AprilTag 상대 pose |
| `/hide/dock_complete` | `std_msgs/Bool` | hide docking | hide mission, mcu | 도킹 완료 |

## MCU 토픽

| Topic | Type | 설명 |
| --- | --- | --- |
| `/mcu/imu` | `sensor_msgs/Imu` | IMU 원본 또는 보정 데이터 |
| `/mcu/display_text` | `std_msgs/String` | OLED 또는 음성 안내 문구 |
| `/mcu/led_mode` | `std_msgs/String` | `off`, `idle`, `guide`, `warning`, `arrived` |
| `/mcu/motor_lock` | `std_msgs/Bool` | 도킹/위장 상태 모터 락 |

## Nav2 / 주행 제어 규칙

| Topic/Action | Type | 설명 |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 목표점 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 액션 |
| `/cmd_vel` | `geometry_msgs/Twist` | 최종 속도 명령 |

`/cmd_vel`은 여러 노드가 동시에 직접 발행하면 위험합니다. 안내팀의 점자 블록 보정, 숨는팀의 Freeze, Nav2 출력은 반드시 하나의 mux/safety layer 또는 명확한 제어권 규칙을 통해 최종 `/cmd_vel`로 나가야 합니다.

## 좌표계

| Frame | 설명 |
| --- | --- |
| `map` | SLAM/AMCL 기준 지도 |
| `odom` | 로봇 odometry |
| `base_link` | 로봇 기준 중심 |
| `camera_front_link` | 전방 카메라 |
| `camera_down_link` | 점자 블록 인식용 하방 카메라 |
| `dock_tag` | 대기석 AprilTag |

## 상태 전이 초안

```text
/hide/state: IDLE_DOCKED
  <- /hide/takeover_start
  -> WAKE
  -> GUIDE_HANDOFF

/guide/state: GUIDE_READY
  -> GUIDING
  <- /hide/freeze true
  -> PAUSED
  <- /hide/freeze false
  -> GUIDING
  -> ARRIVED
  -> DONE
  -> /guide/mission_done true

/hide/state:
  <- /guide/mission_done true
  -> RETURN
  -> DOCKING
  -> IDLE_DOCKED
```

## 기존 문서에서 변경된 이름

| 이전 이름 | 현재 이름 |
| --- | --- |
| `/toy_guide/mission_state` | `/guide/state`, `/hide/state` |
| `/toy_guide/mission_command` | `/guide/command` |
| `/toy_guide/trigger_return` | `/guide/mission_done` |
| `/toy_guide/safety_freeze` | `/hide/freeze` |
| `/toy_guide/vision/blind_block_offset` | `/guide/yellow_line_error` |
| `/toy_guide/nav/visibility_zone` | `/hide/visibility_zone` |
| `/toy_guide/vision/apriltag_pose` | `/hide/apriltag_pose` |
