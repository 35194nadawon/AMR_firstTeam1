# AMR_firstTeam1 — 토이 가이드(Toy Guide)

자율주행전문가교육(xyz아카데미) 팀 프로젝트.

> 평소엔 카페/음식점 구석에 인형처럼 숨어(Freeze) 있다가, 안내견과 함께 온 시각장애인
> 손님이 입장하면 안내 임무를 교대(Take-over) 받아 매장 안내 및 점자 블록 추종을 수행하는
> 자율주행 보조 로봇.

이 레포는 `storagy-simulation-system-docker`(Gazebo + Nav2 + YOLO + LLM + 웹 대시보드)를
베이스로 fork 하여, 팀 고유 패키지 `storagy_hide`(숨는팀)를 얹어 개발하는 워크스페이스입니다.

## 4단계 시나리오 요약
1. 위장 및 대기(Freeze): 구석 대기석(ArUco 도킹)에서 인형처럼 숨음. LED/OLED off, 모터 잠금.
2. 무전 수신 및 임무 교대: 입구 비콘/앱(LLM) 명령으로 기상 → 입구로 Nav2 자율주행.
3. 점자 블록 인식 및 안내: 하방 카메라 YOLO로 점자 블록 추종 + IMU 기반 주행 보정.
4. 돌발 회피 및 미션 클리어: 장애물 감지 시 Freeze, OLED/음성 안내 후 재개 → 착석 시 구석으로 복귀.

## 팀 분업
- 안내팀(`storagy_llm` 중심): 음성/LLM 명령 처리, Nav2 주행, 점자 블록 인식, 도착/서비스 제어.
- 숨는팀(`storagy_hide` 중심): 인간 감지 + 동적 코스트맵(120도 keepout), ArUco 정밀 도킹/복귀,
  위장 상태제어(Freeze: 모터 잠금/LED·OLED off).

자세한 숨는팀 시작 플랜과 4인 분업은 별도 플랜 문서를 참고.

## 빠른 시작 (시뮬레이션)
```bash
cd AMR_firstTeam1

# (선택) LLM 에이전트를 쓰려면 OpenAI API 키 설정
cp .env.example .env       # .env 를 열어 본인 키 입력. 없어도 시뮬 자체는 동작

docker compose up -d        # 이미지 빌드/실행 (첫 빌드는 수 분 소요)
```
접속:
- http://localhost:6080 — 리눅스 데스크탑 (Gazebo / RViz)
- http://localhost:8090 — 웹 대시보드

컨테이너가 시작되면 `ros2 launch storagy full_bringup.launch.py` 가 자동 실행됩니다.

## 코드 수정 / 재빌드
호스트의 `./src` 가 컨테이너 `/opt/storagy_sim_origin_ws/src` 에 마운트됩니다.
- `src/**/scripts/*.py` (YOLO, 배회 노드 등): 시뮬 재시작만 하면 반영.
- 런치/월드/맵/URDF/메시지 정의, **신규 패키지(`storagy_hide`) 포함**: 재빌드 필요.

```bash
docker compose exec storagy-sim rebuild_ws.sh
docker compose restart
```

## 패키지 구성 (`src/`)
- `storagy` — 로봇 모델/월드/맵/런치, YOLO·배회 스크립트, Nav2 파라미터.
- `storagy_interfaces` — 커스텀 서비스(`SetLamp`=LED, `Emotion`=OLED, `Agent`=LLM Q&A).
- `storagy_llm` — 안내팀 LLM 에이전트 + 웹 대시보드.
- `storagy_hide` — 숨는팀 패키지(상태제어 FSM / ArUco 도킹 / 사람 감지+코스트맵). 신규.

기존 인터페이스(토픽 `/cmd_vel`, `/wander_enabled`, `/llm_active`, `/goal_pose`, `/yolo/*`,
`/camera/*`, `/scan` 및 서비스 `SetLamp`/`Emotion`)는 그대로 재사용·확장합니다.

## 팀 ↔ 숨는팀 인계 인터페이스
- `/hide/takeover_start` (안내팀→숨는팀, 교대 시작)
- `/hide/mission_done` (안내팀→숨는팀, 서비스 완료 → 복귀 트리거)
- `/hide/state` (숨는팀→안내팀 대시보드, 현재 상태)
- `/hide/dock_done`, `/hide/persons`, `/hide/keepout_zones` (숨는팀 내부)

## 실제 로봇 접속 (참고)
실하드웨어(storagy) 연동 메모는 [README_realrobot.md](README_realrobot.md) 참고.
