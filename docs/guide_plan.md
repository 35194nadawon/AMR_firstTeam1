## 🟦 [안내팀 R3] Nav2 기반 자율주행 및 점자 블록 추종 경로 보정 플래너 구현

Markdown

```
### 🎯 개발 목표
LiDAR SLAM 기반의 대역적인 전역 경로(Nav2 Navigation) 정보와 R2가 계산한 바닥 점자 블록 비전 오차 데이터를 실시간으로 융합(Sensor Fusion)하여, 로봇이 점자 블록을 이탈하지 않고 목적지 테이블까지 안전하게 주행하도록 로컬 플래너 및 조향 보정 노드를 구현합니다.

### 🛠️ 세부 작업 및 구현 체크리스트
- [ ] **정본 맵 빌드 및 Waypoints 매핑 (`points.yaml`)**
  - 카페/교실 가제보 환경을 SLAM(Cartographer/Nav2)으로 정밀 맵핑하여 `map/1206_sim_1.yaml` 정본 파일 고정 및 AMCL 파라미터 튜닝
  - `storagy_llm/params/points.yaml` 내 각 테이블 및 카운터, 입구의 X/Y/Yaw 가이드 좌표계 최종 확정 (숨는팀 공유용)
- [ ] **Nav2 Local Costmap 및 플래너 연동**
  - R1 백엔드로부터 목적지(`Maps_to_pose`) 액션 요청을 받았을 때의 자율주행 베이스 라인 구동
- [ ] **비전 피드백 기반 조향 보정 알고리즘 구현**
  - R2가 발행하는 `/guide/yellow_line_error`를 구독
  - Nav2 로컬 플래너가 계산한 `cmd_vel` 제어 명령에 비전 오차 기반 PID 제어 보정치($\Delta \theta$)를 주입하여, 최종 모터 명령 `cmd_vel`을 부드럽게 가공해 로봇 하드웨어 및 시뮬레이션에 전달하는 중간 제어기 노드 작성
- [ ] **돌발 상황(인간 진입) 시 급정지 및 대기 로직**
  - 주행 중 숨는팀의 코스트맵 필터로 인해 경로가 막히거나 직접 장애물이 포착될 시, 부드러운 감속 및 정지(Freeze) 제어 연동

### 📌 데이터 인터페이스 & 인계 규칙
- **구독 토픽 (Input)**: `/guide/yellow_line_error`, `/navigate_to_pose` 액션 서버
- **발행 토픽 (Output)**: `/cmd_vel` (`geometry_msgs/msg/Twist`)
```
