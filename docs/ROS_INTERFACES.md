# ROS Interfaces

이 문서는 팀 간 충돌을 줄이기 위한 초안입니다. 실제 구현 중 바뀌면 PR에서 같이 수정합니다.

## 공통 상태 토픽

| Topic | Type | Publisher | Subscriber | 설명 |
| --- | --- | --- | --- | --- |
| `/toy_guide/mission_state` | `std_msgs/String` | mission | dashboard, navigation, mcu | 현재 임무 상태 |
| `/toy_guide/mission_command` | `std_msgs/String` | dashboard/llm/app | mission | `start_guide`, `freeze`, `resume`, `return_home` |
| `/toy_guide/trigger_return` | `std_msgs/Bool` | mission | navigation | 대기석 복귀 트리거 |
| `/toy_guide/safety_freeze` | `std_msgs/Bool` | navigation/vision | mission, mcu | 즉시 정지 필요 여부 |

## 비전 토픽

| Topic | Type | 설명 |
| --- | --- | --- |
| `/toy_guide/vision/person_detected` | `std_msgs/Bool` | 사람 감지 여부 |
| `/toy_guide/vision/person_pose` | `geometry_msgs/PoseStamped` | map 또는 base_link 기준 사람 위치 |
| `/toy_guide/vision/blind_block_offset` | `std_msgs/Float32` | 점자 블록 중심선과 로봇 중심의 오차 |
| `/toy_guide/vision/apriltag_pose` | `geometry_msgs/PoseStamped` | 대기석 AprilTag 상대 pose |

## 내비게이션 토픽

| Topic | Type | 설명 |
| --- | --- | --- |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 목표점 |
| `/cmd_vel` | `geometry_msgs/Twist` | 최종 속도 명령 |
| `/toy_guide/nav/freeze_cmd` | `std_msgs/Bool` | navigation safety layer 정지 명령 |
| `/toy_guide/nav/visibility_zone` | `geometry_msgs/PolygonStamped` | 사람 시야각 120도 위험 구역 |

## MCU 토픽

| Topic | Type | 설명 |
| --- | --- | --- |
| `/toy_guide/mcu/imu` | `sensor_msgs/Imu` | IMU 원본 또는 보정 데이터 |
| `/toy_guide/mcu/display_text` | `std_msgs/String` | OLED 또는 음성 안내 문구 |
| `/toy_guide/mcu/led_mode` | `std_msgs/String` | `off`, `idle`, `guide`, `warning`, `arrived` |
| `/toy_guide/mcu/motor_lock` | `std_msgs/Bool` | 도킹/위장 상태 모터 락 |

## 좌표계 초안

| Frame | 설명 |
| --- | --- |
| `map` | SLAM/AMCL 기준 지도 |
| `odom` | 로봇 odometry |
| `base_link` | 로봇 기준 중심 |
| `camera_front_link` | 전방 카메라 |
| `camera_down_link` | 점자 블록 인식용 하방 카메라 |
| `dock_tag` | 대기석 AprilTag |

## 명령 문자열 초안

- `start_guide`: 안내 임무 시작
- `freeze`: 즉시 정지
- `resume`: 안내 재개
- `arrived`: 목적지 도착 처리
- `return_home`: 대기석 복귀
- `dock_complete`: 도킹 완료
