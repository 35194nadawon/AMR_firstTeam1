# Development Workflow

## 기본 흐름

1. 최신 main을 받는다.
2. 이슈 번호를 확인한다.
3. 기능 브랜치를 만든다.
4. 작업 후 로컬 또는 로봇에서 최소 테스트를 한다.
5. PR을 올리고 테스트 결과를 적는다.

```bash
git checkout main
git pull origin main
git checkout -b feat/3-mission-state-machine
```

## 브랜치 이름

- `feat/<issue-number>-<short-name>`
- `fix/<issue-number>-<short-name>`
- `docs/<issue-number>-<short-name>`
- `test/<issue-number>-<short-name>`

예시:

```text
feat/3-mission-state-machine
feat/4-yolo-human-detection
fix/7-freeze-cmd-vel-lock
docs/10-demo-checklist
```

## 커밋 메시지

```text
feat(vision): add YOLO human detection scaffold
feat(nav): add freeze resume controller
docs: add ROS topic contract
fix(mcu): handle serial reconnect
```

## PR 체크리스트

- 어떤 이슈를 해결했는지 적는다.
- 실제 로봇 테스트 여부를 적는다.
- 실행한 명령을 적는다.
- topic 이름, launch 이름, 파라미터가 바뀌면 문서도 같이 수정한다.
- 기존 `storagy` launch가 깨지지 않는지 확인한다.

## Storagy에서 코드 반영

로봇 워크스페이스에서 pull 후 빌드한다.

```bash
cd ~/Desktop/storagy_ws
git pull origin main
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

기존 bringup이 먼저 정상인지 확인한다.

```bash
ros2 launch storagy bringup.launch.py
```

새 기능은 통합 launch가 만들어진 뒤 실행한다.

```bash
ros2 launch toy_guide_bringup toy_guide_demo.launch.py
```

## 안전 규칙

- 실제 로봇 테스트 전 바퀴가 떠 있는 상태 또는 넓은 공간에서 `/cmd_vel`을 확인한다.
- Freeze 기능은 Vision보다 먼저 단독 테스트한다.
- 여러 노드가 `/cmd_vel`을 동시에 발행하지 않도록 한다.
- 로봇 IP, 계정, 비밀번호는 public 문서에 쓰지 않는다.
