# Architecture

## 기본 전략

기존 `storagy` 패키지는 로봇 하드웨어와 Nav2 기준선으로 유지한다. Toy Guide 기능은 역할별 패키지로 추가하여 실제 로봇 구동 안정성을 보호한다.

토픽은 안내팀 `/guide/*`, 숨는팀 `/hide/*`, MCU `/mcu/*`, 웹 `/dashboard/*` 기준으로 나눈다.

## 패키지 설계

```text
src/
  storagy/
  storagy_interfaces/
  storagy_hide/
  storagy_llm/
  storagy_guide/
  storagy_mcu_bridge/
```

## 패키지별 책임

### `storagy`

- 기존 로봇 모델, 지도, Nav2, Cartographer, RViz 기준선
- 실제 로봇/시뮬레이션에서 검증된 launch와 설정 유지
- 큰 변경은 최소화

### `storagy_interfaces`

- 팀 간 srv/msg 계약 고정
- custom message가 필요할 때만 추가
- 초기에는 표준 메시지(`std_msgs`, `geometry_msgs`, `sensor_msgs`) 우선 사용

### `storagy_hide`

- `/hide/*` 네임스페이스 담당
- 위장 대기, 기상, Freeze, 복귀, 도킹 상태 머신
- 사람 감지, 시야각 120도 위험 구역, AprilTag 도킹
- 안내팀의 `/guide/mission_done`을 받아 복귀 시작

### `storagy_guide`

- `/guide/*` 네임스페이스 담당
- 점자 블록 인식, 경로 보정, 안내 상태 관리
- 목적지 도착 처리 및 `/guide/mission_done` 발행
- 필요 시 `storagy_llm`의 명령 결과를 주행 목표로 변환

### `storagy_llm`

- LLM agent, 웹 대시보드, 음성/텍스트 명령 처리
- `/guide/command`, `/hide/takeover_start` 등 상위 이벤트 발행
- 교수님 실습 API가 포함될 수 있으므로 별도 검증 후 통합

### `storagy_mcu_bridge`

- `/mcu/*` 네임스페이스 담당
- MCU serial 통신
- IMU pitch/accel 수신
- OLED/LED 상태 출력
- 모터 보정 또는 제동 명령 전달

## 상태 머신 초안

```text
/hide/state: IDLE_DOCKED
  -> WAKE_UP
  -> GUIDE_HANDOFF

/guide/state: GUIDE_READY
  -> GUIDING
  -> PAUSED
  -> GUIDING
  -> ARRIVED
  -> DONE

/hide/state:
  -> RETURN_TO_DOCK
  -> DOCKING
  -> IDLE_DOCKED
```

## 통합 원칙

- 기존 launch 파일을 직접 고치기 전, 새 launch에서 include하여 검증한다.
- `/cmd_vel`을 여러 노드가 동시에 쓰지 않도록 mux 또는 safety controller를 둔다.
- 카메라/YOLO가 불안정해도 LiDAR 기반 정지 로직은 독립적으로 동작하게 한다.
- 실제 로봇 테스트 전에는 RViz에서 topic, TF, costmap을 먼저 확인한다.
