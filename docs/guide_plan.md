# Nav2 Guide Plan

## 담당 범위

`feature/nav2-guide`는 안내팀 R3 범위입니다. Nav2를 통해 목적지로 이동하고, 점자 블록 오차와 숨는팀 Freeze 신호를 받아 안내 주행 상태를 관리합니다.

## 현재 MVP 구현

추가 패키지:

```text
src/storagy_guide/
```

추가 노드:

```text
guide_nav_node
```

현재 동작:

- `/guide/person_arrived`가 `true`가 되면 고정 목적지로 Nav2 goal 전송
- `/guide/command`가 `start_guide`이면 고정 목적지로 Nav2 goal 전송
- `/hide/freeze=true`이면 Nav2 goal cancel 및 `/cmd_vel` 0 발행
- `/hide/freeze=false`이면 필요 시 같은 목적지로 goal 재전송
- Nav2 도착 성공 시 `/guide/state=ARRIVED`, `/guide/mission_done=true` 발행
- `/guide/yellow_line_error`는 구독만 해두고, 실제 cmd_vel 보정은 다음 단계에서 구현

## 토픽

구독:

```text
/guide/person_arrived       std_msgs/Bool
/guide/command              std_msgs/String
/hide/freeze                std_msgs/Bool
/guide/yellow_line_error    geometry_msgs/Pose2D
```

발행:

```text
/guide/state                std_msgs/String
/guide/mission_done         std_msgs/Bool
/cmd_vel                    geometry_msgs/Twist
```

Action client:

```text
/navigate_to_pose           nav2_msgs/action/NavigateToPose
```

## 실행

Nav2가 먼저 실행되어 있어야 합니다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch storagy_guide guide_nav.launch.py
```

테스트용으로 바로 사람 도착 신호를 켜려면:

```bash
ros2 launch storagy_guide guide_nav.launch.py demo_person_arrived:=true
```

또는 토픽으로 시작:

```bash
ros2 topic pub --once /guide/person_arrived std_msgs/msg/Bool "{data: true}"
```

고정 목적지 변경:

```bash
ros2 launch storagy_guide guide_nav.launch.py target_x:=1.0 target_y:=0.0 target_yaw:=0.0
```

## 교실 맵

교실 맵 파일을 `src/storagy/map`에 추가했습니다.

```text
src/storagy/map/2026_amr.yaml
src/storagy/map/2026_amr.pgm
```

맵 메타데이터:

```yaml
resolution: 0.05
origin: [-4.66, -10.3, 0]
```

Nav2에서 이 맵을 쓰려면 navigation launch의 `map` 인자로 넘깁니다.

```bash
ros2 launch storagy launch/navigation2/navigation2.launch.py \
  map:=/path/to/install/storagy/share/storagy/map/2026_amr.yaml
```

## 다음 단계

- `/guide/yellow_line_error` 기반 조향 보정 구현
- Nav2 `/cmd_vel` 출력과 보정 노드 출력이 충돌하지 않게 cmd_vel mux/safety layer 구성
- 교실 맵에서 문 위치와 목표 좌표를 실제 좌표로 튜닝
- `/guide/command`에 목적지 이름을 실어 `table_1`, `counter` 등으로 확장
