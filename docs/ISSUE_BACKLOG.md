# Issue Backlog

GitHub 앱 권한이 이슈 생성까지 허용되지 않을 때를 대비한 티켓 초안입니다. 아래 항목을 GitHub Issues에 하나씩 등록해서 사용합니다.

## [통합] Toy Guide ROS2 패키지 구조 생성

### 목표

기존 `storagy` 패키지를 크게 바꾸지 않고 Toy Guide 기능을 추가할 ROS2 패키지 뼈대를 만든다.

### 작업 내용

- [ ] `toy_guide_bringup` 패키지 생성
- [ ] `toy_guide_interfaces` 패키지 생성 여부 결정
- [ ] `toy_guide_mission`, `toy_guide_vision`, `toy_guide_navigation` 패키지 생성
- [ ] 기존 `storagy` launch를 include하는 통합 launch 초안 작성
- [ ] `colcon build --symlink-install` 확인

### 완료 기준

- [ ] `ros2 pkg list | grep toy_guide`로 패키지가 보인다.
- [ ] 기존 `ros2 launch storagy bringup.launch.py`가 깨지지 않는다.
- [ ] 구조가 `docs/ARCHITECTURE.md`와 일치한다.

## [Mission] 안내/대기/복귀 상태 머신 구현

### 목표

Toy Guide 전체 시나리오를 관리하는 ROS2 mission state node를 구현한다.

### 작업 내용

- [ ] `/toy_guide/mission_command` 구독
- [ ] `/toy_guide/mission_state` 발행
- [ ] `start_guide`, `freeze`, `resume`, `return_home` 명령 처리
- [ ] 상태 전이 로그 출력
- [ ] 웹/MCU/navigation 팀이 구독할 수 있게 테스트 publisher 작성

### 완료 기준

- [ ] 명령 토픽 입력에 따라 상태가 바뀐다.
- [ ] `ros2 topic echo /toy_guide/mission_state`로 상태가 확인된다.

## [Vision] YOLOv8 사람 감지 노드 구현

### 목표

카메라 영상에서 사람을 감지하고 Freeze 판단에 사용할 토픽을 발행한다.

### 작업 내용

- [ ] 카메라 토픽 확인
- [ ] YOLOv8 모델 실행 방식 결정: Python node 또는 외부 detector bridge
- [ ] 사람 감지 시 `/toy_guide/vision/person_detected` 발행
- [ ] 가능하면 `/toy_guide/vision/person_pose` 추정 초안 작성
- [ ] 감지 FPS와 지연 시간 기록

### 완료 기준

- [ ] 사람이 들어오면 Bool 토픽이 true가 된다.
- [ ] 감지 실패 시 false 또는 timeout 처리가 된다.
- [ ] Freeze 이슈와 연동 가능한 최소 토픽이 나온다.

## [Navigation] Freeze/Resume 안전 제어 구현

### 목표

사람 또는 장애물 감지 시 로봇을 즉시 정지시키고, 안전 상태가 되면 안내를 재개할 수 있게 한다.

### 작업 내용

- [ ] `/toy_guide/safety_freeze` 구독
- [ ] `/cmd_vel` 제어 구조 확인
- [ ] Nav2 cancel, velocity mux, zero cmd_vel 중 실제 로봇에 안전한 방식 결정
- [ ] Freeze 상태에서 모터 명령이 나가지 않는지 확인
- [ ] Resume 시 기존 goal 또는 mission state와 연결

### 완료 기준

- [ ] freeze 명령 후 0.5초 이내 정지한다.
- [ ] resume 명령 후 mission state가 정상 복귀한다.
- [ ] 기존 storagy bringup을 깨지 않는다.

## [Vision] 점자 블록 인식 및 중심 오차 발행

### 목표

하방 카메라로 점자 블록을 인식하고 로봇 중심과의 오차를 발행한다.

### 작업 내용

- [ ] 하방 카메라 토픽과 장착 각도 확인
- [ ] 선형 블록/점형 블록 데이터셋 또는 테스트 이미지 확보
- [ ] YOLOv8 detection 결과에서 중심선 계산
- [ ] `/toy_guide/vision/blind_block_offset` 발행
- [ ] 조명 변화에 따른 인식률 기록

### 완료 기준

- [ ] RViz 또는 로그에서 offset 값이 안정적으로 확인된다.
- [ ] navigation 팀이 조향 보정에 사용할 수 있다.

## [Navigation] 사람 시야각 120도 위험 구역 생성

### 목표

사람 위치와 방향을 기준으로 120도 부채꼴 위험 구역을 생성하고 Nav2 회피 로직에 연결한다.

### 작업 내용

- [ ] 사람 pose 입력 기준 frame 결정
- [ ] 120도 polygon 생성
- [ ] `/toy_guide/nav/visibility_zone` 발행
- [ ] Costmap Filter 적용 가능성 검증
- [ ] RViz에서 위험 구역 시각화

### 완료 기준

- [ ] 사람 pose가 바뀌면 polygon도 실시간 갱신된다.
- [ ] Nav2 경로가 위험 구역을 피하는지 확인한다.

## [Docking] AprilTag 기반 대기석 정밀 도킹

### 목표

대기석 AprilTag를 인식해 로봇이 일정 거리와 각도로 정렬되도록 한다.

### 작업 내용

- [ ] AprilTag 카메라 토픽 확인
- [ ] tag id와 `dock_tag` frame 정의
- [ ] `/toy_guide/vision/apriltag_pose` 발행
- [ ] 저속 후진/정렬 제어 로직 구현
- [ ] 도킹 완료 조건 정의

### 완료 기준

- [ ] 도킹 완료 상태가 mission node로 전달된다.
- [ ] OLED/LED가 대기 모드로 전환된다.

## [MCU] IMU, OLED, LED 브릿지 구현

### 목표

ROS2와 MCU 사이의 serial bridge를 만들어 안내 상태와 안전 상태를 하드웨어에 반영한다.

### 작업 내용

- [ ] MCU 포트와 baudrate 확인
- [ ] IMU 데이터를 `/toy_guide/mcu/imu`로 발행
- [ ] `/toy_guide/mcu/display_text` 구독
- [ ] `/toy_guide/mcu/led_mode` 구독
- [ ] `/toy_guide/mcu/motor_lock` 구독

### 완료 기준

- [ ] 안내/경고/대기 상태가 OLED 또는 LED에 표시된다.
- [ ] serial reconnect 또는 실패 로그가 처리된다.

## [Dashboard] 웹 대시보드와 LLM 무전 연동

### 목표

로봇 상태를 웹에서 보고, 안내 시작/복귀/무전 메시지를 ROS로 전달한다.

### 작업 내용

- [ ] ROS bridge 방식 결정: rosbridge, FastAPI bridge, websocket 중 선택
- [ ] mission state 표시
- [ ] 사람 감지 경고 표시
- [ ] LLM 무전 문구 출력
- [ ] `start_guide`, `resume`, `return_home` 명령 버튼 연결

### 완료 기준

- [ ] 웹에서 mission command를 보내면 ROS 토픽이 발행된다.
- [ ] 로봇 상태 변화가 웹에 표시된다.

## [Demo] 전체 시나리오 리허설 체크리스트 작성

### 목표

발표 전 실제 로봇 시연 순서와 실패 대응 절차를 정리한다.

### 작업 내용

- [ ] 시작 전 센서 체크리스트 작성
- [ ] 각 터미널 실행 순서 작성
- [ ] Freeze 수동 테스트 절차 작성
- [ ] 안내 시작부터 복귀까지 영상 촬영 동선 작성
- [ ] 실패 시 수동 정지 방법 작성

### 완료 기준

- [ ] 팀원이 문서만 보고 같은 순서로 시연을 재현할 수 있다.
