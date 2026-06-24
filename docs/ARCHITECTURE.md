# Architecture

## 기본 전략

기존 `storagy` 패키지는 로봇 하드웨어와 Nav2 기준선으로 유지한다. Toy Guide 기능은 새 패키지로 추가하여 실제 로봇 구동 안정성을 보호한다.

## 패키지 설계

```text
src/
  storagy/
  toy_guide_bringup/
  toy_guide_interfaces/
  toy_guide_mission/
  toy_guide_vision/
  toy_guide_navigation/
  toy_guide_mcu_bridge/
  toy_guide_dashboard_bridge/
```

## 패키지별 책임

### `toy_guide_bringup`

- 전체 launch 파일 제공
- 기존 `storagy` bringup 이후 추가 기능 노드 실행
- 데모 모드별 launch argument 관리

예상 launch:

- `toy_guide_demo.launch.py`
- `toy_guide_vision.launch.py`
- `toy_guide_navigation.launch.py`

### `toy_guide_interfaces`

- 팀 간 topic/msg 계약 고정
- custom message가 필요할 때만 추가
- 초기에는 표준 메시지(`std_msgs`, `geometry_msgs`, `sensor_msgs`) 우선 사용

### `toy_guide_mission`

- 임무 상태 머신 관리
- 대기, 안내 시작, 안내 중, Freeze, Resume, 복귀, 도킹 완료 상태 관리
- LLM/앱/웹 명령을 ROS 명령으로 변환

### `toy_guide_vision`

- YOLOv8 사람 감지
- YOLOv8 점자 블록 감지
- AprilTag pose 추정
- 감지 결과를 ROS 토픽으로 발행

### `toy_guide_navigation`

- Nav2 goal 발행
- Freeze/Resume 안전 제어
- 사람 시야각 120도 위험 구역 생성
- Costmap Filter 적용 후보 검증
- 대기석 복귀와 도킹 접근 경로 생성

### `toy_guide_mcu_bridge`

- MCU serial 통신
- IMU pitch/accel 수신
- OLED/LED 상태 출력
- 모터 보정 또는 제동 명령 전달

### `toy_guide_dashboard_bridge`

- ROS 상태를 웹 대시보드로 전달
- 웹/앱 명령을 ROS 명령으로 전달
- 데모용 경고, 심박수, 무전 메시지 표시

## 상태 머신 초안

```text
IDLE_DOCKED
  -> WAKE_UP
  -> MOVE_TO_ENTRANCE
  -> GUIDE_READY
  -> GUIDING
  -> FREEZE
  -> GUIDING
  -> ARRIVED
  -> RETURN_TO_DOCK
  -> DOCKING
  -> IDLE_DOCKED
```

## 통합 원칙

- 기존 launch 파일을 직접 고치기 전, 새 launch에서 include하여 검증한다.
- `/cmd_vel`을 여러 노드가 동시에 쓰지 않도록 mux 또는 safety controller를 둔다.
- 카메라/YOLO가 불안정해도 LiDAR 기반 정지 로직은 독립적으로 동작하게 한다.
- 실제 로봇 테스트 전에는 RViz에서 topic, TF, costmap을 먼저 확인한다.
