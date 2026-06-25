# 안내팀 1단계 (R1: LLM/Voice & WAKE 트리거) 구현 계획

안내팀의 1단계(R1) 요구사항인 음성/LLM 인텐트 연동, 숨는팀 기상(WAKE) 트리거 발행, 그리고 대시보드 도슨트 모드 UI 동적 전환을 구현하고 깃허브의 `feature/llm-voice` 브랜치에 반영하는 작업 계획입니다.

## User Review Required

> [!IMPORTANT]
> - **브랜치 격리**: 1단계(`feature/llm-voice`)와 4단계(`feature/mission-control`) 기능이 섞이지 않도록, 이 브랜치에서는 오직 1단계 관련 코드(WAKE 트리거 발행 및 대시보드 `/hide/state` 연동)만 구현하여 푸시합니다.
> - **맵 변경 사항 안내**: 이전 작업에서 AI 어시스턴트가 맵 파일(`.pgm`, `.yaml`)이나 월드 파일(`.sdf`)을 직접 수정하지는 않았습니다. 책상 제거 및 맵 갱신은 팀원분이 `origin/main`에 반영해 두신 최신 커밋을 병합하는 과정에서 자동으로 반영되었습니다.

## Open Questions (숨는팀 확인 필요 사항)

> [!WARNING]
> 숨는팀과의 원활한 연동을 위해 다음 사항들을 확인해 주셔야 합니다:
> 1. **숨는팀 기상 상태 문자열**: 숨는팀 로봇이 깨어났을 때 `/hide/state` 토픽으로 발행하는 상태값 문자열이 `"WAKE"`와 `"GUIDE"`가 맞는지 확인이 필요합니다. (이 값이 일치해야 대시보드가 '도슨트 모드'로 자동 전환됩니다.)
> 2. **기상(WAKE) 트리거 토픽 수신 여부**: 숨는팀 로봇이 `/hide/takeover_start` (`std_msgs/msg/Bool`, True 펄스) 토픽을 정상적으로 구독하여 깨어나는 상태로 진입하는지 확인해 주세요.

## Proposed Changes

1단계 기능을 온전히 담아 `src/AMR_firstTeam1/src/storagy_llm/` 패키지를 복구 및 구현합니다.

### storagy_llm (ROS 2 패키지)

#### [NEW] [robot_tools.py](file:///home/jin/storagy-simulation-system-docker/src/AMR_firstTeam1/src/storagy_llm/storagy_llm/robot_tools.py)
- `navigate_to_pose` 메서드 시작 시 `/hide/takeover_start` 퍼블리셔(`std_msgs/msg/Bool`)를 생성 및 `True` 펄스를 발행하여 숨는팀 로봇을 깨우는 로직 구현.
- 4단계 기능(임무 완료, OLED 감정 변경)은 이 브랜치의 소스코드에서 제외합니다.

#### [NEW] [web_dashboard.py](file:///home/jin/storagy-simulation-system-docker/src/AMR_firstTeam1/src/storagy_llm/storagy_llm/web_dashboard.py)
- 숨는팀의 상태 토픽 `/hide/state` (`std_msgs/msg/String`)를 구독하는 서브스크라이버 추가.
- 수신된 상태 정보(`hide_state`)를 Flask SSE 스트림인 `/api/stream`을 통해 프론트엔드로 실시간 전송하도록 `snapshot()` 데이터에 포함.
- SSE 이벤트를 통해 상태 변경 로그를 자동으로 기록하도록 `add_event` 연동.

#### [NEW] [index.html](file:///home/jin/storagy-simulation-system-docker/src/AMR_firstTeam1/src/storagy_llm/web/index.html)
- 대시보드 상단 배지 영역에 숨는팀 상태 및 도슨트 모드 여부를 나타내는 배지 추가 (`#hide-badge`).
- 서비스 완료 버튼은 이 브랜치에서는 숨겨두거나 제외합니다.

#### [NEW] [app.js](file:///home/jin/storagy-simulation-system-docker/src/AMR_firstTeam1/src/storagy_llm/web/app.js)
- SSE `/api/stream`에서 `hide_state` 속성을 파싱하여 대시보드 UI를 갱신.
- `hide_state`가 `"WAKE"` 또는 `"GUIDE"`일 때 대시보드 전체에 시각적으로 부각되는 '도슨트 모드 활성화' 스타일을 적용하고, 배지 상태 변경.

#### [NEW] [style.css](file:///home/jin/storagy-simulation-system-docker/src/AMR_firstTeam1/src/storagy_llm/web/style.css)
- 도슨트 모드 진입 시 대시보드 테두리나 헤더에 골드/그라디언트 테마의 프리미엄 디자인을 적용하는 CSS 스타일 추가.

#### [NEW] (기타 패키지 기본 파일들)
- `setup.py`, `setup.cfg`, `package.xml`, `agent_service.py`, `agent_client.py`, `yolo_detector.py`, `guide_controller.py`, `params/prompt.yaml`, `params/points.yaml` 등의 필요한 소스 파일을 outer workspace 및 backup에서 복사 및 복구하여 패키지 빌드가 가능하도록 구성합니다.

---

## Verification Plan

### Automated / CLI Verification
1. Docker 컨테이너 내에서 워크스페이스 빌드:
   ```bash
   docker compose exec storagy-sim rebuild_ws.sh
   ```
2. 대시보드 및 LLM 서비스 실행 후, 테스트 터미널에서 가상의 숨는팀 상태 발행 후 UI 확인:
   ```bash
   # 가상의 숨는팀 기상(WAKE) 상태 발행
   ros2 topic pub -1 /hide/state std_msgs/msg/String "{data: 'WAKE'}"
   ```
   - 대시보드 웹 UI가 '도슨트 모드'로 즉시 전환되는지 확인합니다.
3. LLM 에이전트에 목적지 이동 명령 전송 후 토픽 확인:
   ```bash
   # LLM 이동 명령 수행 시 WAKE 트리거 발행 모니터링
   ros2 topic echo /hide/takeover_start
   ```
   - `std_msgs/msg/Bool` 타입으로 `data: true`가 단일 펄스로 정상 출력되는지 검증합니다.

### Manual Verification
- 브라우저에서 `http://localhost:8090` 대시보드에 접속하여 실시간 동기화 상태와 도슨트 모드 UI 전환의 비주얼 완성도를 직접 검토합니다.
