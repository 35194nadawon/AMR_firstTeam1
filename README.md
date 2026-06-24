# AMR_firstTeam1

자율주행전문가교육 팀 프로젝트: **Toy Guide - 안내견을 위한 시크릿 보조 로봇**

기존 Storagy 로봇의 ROS2 Humble, SLAM, Nav2, LiDAR, 카메라 구성을 최대한 유지하면서 다음 기능을 단계적으로 얹습니다.

- 평상시에는 구석 대기석에서 인형처럼 위장 대기
- 안내견/시각장애인 손님 입장 시 LLM 또는 앱 명령으로 안내 임무 교대
- YOLOv8로 사람, 점자 블록, 위험 상황 인식
- Nav2와 Costmap Filters로 사람 시야각/위험 구역 회피
- AprilTag 기반 대기석 정밀 도킹
- 장애물 감지 시 Freeze, 위험 해소 후 Resume
- OLED/LED/음성/웹 대시보드로 상태 안내

## 현재 기준 코드

현재 레포는 Storagy 실습 코드를 기준으로 개발합니다. 실제 로봇 구동에 필요한 기존 launch, URDF, Nav2 파라미터, SLAM 흐름은 크게 바꾸지 않습니다.

기준 구성:

- ROS2 Humble
- Storagy ROS 패키지
- Cartographer SLAM
- Nav2 Navigation
- LiDAR 기반 `/scan`
- RViz / Gazebo 또는 실제 로봇 환경
- Ubuntu 22.04 기반 Storagy 워크스페이스

## 개발 원칙

1. 기존 `storagy` 패키지는 가능한 한 보존합니다.
2. 새 기능은 `toy_guide_*` 패키지로 분리해서 추가합니다.
3. 실제 로봇에서 바로 돌려야 하므로 launch와 topic 이름은 문서화 후 변경합니다.
4. 큰 변경은 Pull Request로 리뷰 후 병합합니다.
5. 로봇에서 테스트한 명령, 성공/실패 로그, 센서 조건은 이슈에 남깁니다.

## 추천 패키지 구조

```text
src/
  storagy/                         # 기존 로봇 기준 패키지, 최소 수정
  toy_guide_bringup/               # 통합 launch, lifecycle, 실행 entrypoint
  toy_guide_interfaces/            # msg/srv/action 정의
  toy_guide_mission/               # 임무 상태 머신, LLM 명령 처리
  toy_guide_vision/                # YOLOv8, AprilTag, 점자 블록 인식
  toy_guide_navigation/            # Freeze/Resume, Costmap Filter, 회피/복귀
  toy_guide_mcu_bridge/            # MCU, IMU, OLED, LED, 모터 제어 브릿지
  toy_guide_dashboard_bridge/      # 웹 대시보드, 상태 송수신 API
```

자세한 설계는 아래 문서를 기준으로 진행합니다.

- [프로젝트 기획](docs/PROJECT_PLAN.md)
- [아키텍처](docs/ARCHITECTURE.md)
- [ROS 인터페이스](docs/ROS_INTERFACES.md)
- [개발 워크플로우](docs/DEVELOPMENT_WORKFLOW.md)

## Storagy 로봇 접속 및 기존 실행 흐름

팀 내부에서 공유받은 Storagy 터미널 또는 Ubuntu 22.04 환경에서 진행합니다.

```bash
cd ~/Desktop/storagy_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

터미널 1: 기본 bringup

```bash
ros2 launch storagy bringup.launch.py
```

터미널 2: AMCL TF 설정 후 Cartographer 실행

```bash
ros2 param set /amcl tf_broadcast false
ros2 launch storagy cartographer.launch.py
```

터미널 3: 수동 조작

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> 접속 계정, 비밀번호, 로봇 IP 같은 민감 정보는 public README에 직접 적지 말고 팀 내부 채널에서 공유합니다.

## Git 협업 방식

```bash
git checkout main
git pull origin main
git checkout -b feat/vision-human-detection
# 작업 후
git add .
git commit -m "feat: add human detection node scaffold"
git push origin feat/vision-human-detection
```

PR 제목 예시:

```text
feat(vision): add YOLO human detection node
feat(nav): add freeze resume state controller
feat(mission): add Toy Guide mission state machine
```

## 초기 마일스톤

1. 프로젝트 기반 구축
2. Freeze / Resume와 사람 감지
3. 점자 블록 추종 및 안내 주행
4. AprilTag 대기석 복귀
5. 웹/LLM/MCU 통합 데모
