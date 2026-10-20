<div align="center">

# TurtleBot4 RC카 탐색 · 추적

**고정 웹캠이 사각지대에서 RC카를 발견하면 TurtleBot4가 스스로 출발해 회전 탐색한 뒤 접근 · 추종하는 것을 목표로 한 ROS 2 팀 미니 프로젝트입니다.**

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)
![TurtleBot4](https://img.shields.io/badge/TurtleBot4-OAK--D_%2B_RPLIDAR-2E7D32?style=flat-square)
![Nav2](https://img.shields.io/badge/Nav2-AMCL-3C6E91?style=flat-square)
![YOLO11n](https://img.shields.io/badge/Ultralytics-YOLO11n_%2B_ByteTrack-111F68?style=flat-square)
![Python](https://img.shields.io/badge/Python-rclpy-3776AB?style=flat-square&logo=python&logoColor=white)

<img src="docs/images/perception_demo.gif" width="480" alt="로봇을 세워 둔 상태에서 OAK-D 영상에 커스텀 YOLO와 ByteTrack을 적용해 RC카와 더미 박스에 추적 ID를 단 화면"><br>
<sub>▲ 인식 단계 단독 테스트(09-01) — 로봇을 세워 두고 OAK-D 영상에 커스텀 YOLO + ByteTrack을 적용한 화면입니다. 미션(탐색 · 추종) 실행 영상이 아닙니다.</sub>

</div>

<br>

## 🧭 개요

| 항목 | 내용 |
|---|---|
| 구분 | ROKEY 9기 지능-1 팀 미니 프로젝트 |
| 기간 | 2026년 8월 말 ~ 9월 초, 약 1주 |
| 인원 · 담당 | 4인 팀 · 담당: 객체 탐지 — RC카 · 더미 박스 인식 모델(웹캠 · 로봇 카메라 공용), 웹캠 감지 · 트리거 노드, 로봇 카메라 인식 프로토타입 |
| 환경 | ROS 2 Jazzy · TurtleBot4 (OAK-D, RPLIDAR) · Nav2 · USB 웹캠 · Ultralytics YOLO11n + ByteTrack |
| 상태 | 🟡 개선 과제 남음 — 통합 테스트에서 탐색 위치까지 이동을 확인했고, 회전 탐색 중 RC카 목표 고정(lock)은 다음 단계로 남았습니다. 원인을 분리하기 위해 탐색 방식만 바꾼 v1~v4를 만들었고, 로봇 검증이 다음 단계입니다. |

## 🎯 배경과 목표

강의 과제(Team Exercise)는 "이벤트를 받으면 AMR이 언도킹하고, 장애물을 피하며 더미는 무시하고, 목표를 추적 · 접근한다"였습니다. 우리 팀은 이를 아래 시나리오로 구체화했습니다.

1. 고정 웹캠이 도킹된 로봇이 볼 수 없는 구역을 감시합니다.
2. 웹캠이 RC카를 인식하면 트리거 토픽을 한 번 보냅니다.
3. 로봇이 언도킹 → 웹캠 감시 구역(`search_pose`)으로 이동 → 제자리 360° 회전 탐색 → RC카 추적 ID 고정 → 0.3 m 거리를 유지하며 접근 · 추종합니다.
4. 더미 박스는 클래스로 거르고, 다른 RC카는 처음 고정한 추적 ID로 구분하는 것이 설계 의도였습니다(로봇 시험은 다음 단계).

## 🏗 구성

```mermaid
flowchart LR
    subgraph WPC["웹캠 PC"]
        CAM["USB 웹캠"] --> WD["webcam_detector<br/>YOLO11n"]
        WD -->|"rc_car_detected (Bool, 1회)"| MT["mission_trigger"]
    end
    subgraph GPU["GPU 노트북"]
        S["search_and_track_server<br/>커스텀 액션 서버"]
        NAV["Nav2 + AMCL"]
    end
    subgraph TB4["TurtleBot4"]
        OAK["OAK-D RGB + depth"]
        BASE["Create 3 + RPLIDAR"]
    end
    MT -->|"SearchAndTrack goal"| S
    OAK -->|"compressed RGB / compressedDepth"| S
    S -->|"goToPose / spin"| NAV
    S -->|"undock"| BASE
    NAV --> BASE
```

액션 서버 상태: `UNDOCKING → NAV_TO_SEARCH → SEARCHING(360° 회전 + lock) → APPROACHING / FOLLOWING ↔ HOLDING` (목표를 놓치면 `SEARCHING`으로 돌아감).
통합 테스트는 `SEARCHING`에서 멈췄습니다.

## 🙋 담당 역할

팀은 시스템 통합 · 미션 관리(기술 리드), 내비게이션 · 회피, 브링업 · SLAM, 객체 탐지로 역할을 나눴습니다. 저는 객체 탐지를 맡아 RC카 · 더미 박스 인식(데이터 수집 · 라벨링 · 모델 선정 · 학습 · 평가)과 이를 쓰는 웹캠 · 로봇 카메라 쪽 인식 노드를 담당했습니다.

1. **RC카 · 더미 박스 인식 모델** — 팀과 함께 RC카와 더미 박스를 촬영 · 라벨링하고, YOLO11n을 선정해 파인튜닝했습니다(`rc_dummy_y11n.pt` — 웹캠 감지 노드와 로봇 카메라 미션 서버가 같은 모델을 사용). → [`src/rc_tracker/models/`](src/rc_tracker/models/)
2. **인식 프로토타입 노드 (09-01)** — 로봇 카메라 영상에 YOLO + ByteTrack을 붙여 RC카 · 더미 박스에 추적 ID를 부여하는 노드를 만들고, 로봇 정지 상태에서 동작을 확인했습니다. → [`src/tb4_yolo_tracker/`](src/tb4_yolo_tracker/), [`docs/perception_prototype.md`](docs/perception_prototype.md)
3. **웹캠 감지 · 트리거 노드** — 웹캠 YOLO가 `RC`를 N프레임 연속으로 검출하면 latched `Bool`을 한 번 보내고, 이를 받아 액션 goal을 보내는 노드입니다(`webcam_detector`, `mission_trigger`).
4. **통합 테스트와 현장 디버깅** — 09-03 통합 테스트에서 미션 서버(`search_and_track_server`)를 실제 로봇에 올려 시험하며 팀과 함께 찾은 문제를 반영했습니다(→ [트러블슈팅](#-트러블슈팅)). 언도킹 위치(x, y)를 `initial_pose`로, RViz에서 지정한 웹캠 감시 구역 중심을 `search_pose`로 넣었습니다(yaw는 추정치).
5. **목표 고정이 안 된 원인을 분리하기 위한 탐색 변형 v1~v4** — 회전 속도, depth 토픽 지연, 모션 블러라는 가설 3개를 하나씩 검증할 수 있도록 탐색 방식만 바꾼 변형 4개와 판별용 `[SEARCH]` 로그를 만들었습니다. → [`src/search_variants/`](src/search_variants/), [`docs/search_variants.md`](docs/search_variants.md)

**팀원 기여**: 미션 서버의 RGB-depth 시간 동기화(`ApproximateTimeSynchronizer`), `compressedDepth` 직접 디코드, 목표 앞 정지 위치(stand-off goal) 계산은 팀원이 추가했습니다. 로봇 TF 프레임 확인과 SLAM 가이드도 팀원 작업입니다.

## 🧩 트러블슈팅

09-03 통합 테스트에서 실제 로봇에 올려 시험하며 드러난, **팀이 함께 푼 문제**입니다.

| 문제 | 원인 | 어떻게 해결했나 | 결과 |
|---|---|---|---|
| 도킹 상태에서 AMCL이 위치를 잡지 못함 | 도킹 중에는 RPLIDAR가 멈춰 `/scan`이 없음 | **언도킹 → 초기 위치 설정 → Nav2 대기** 순서로 변경 | 탐색 위치까지 이동 확인 |
| 좁은 구역에서 경로가 계산되지 않음 | costmap 팽창 반경(0.45 m)이 통로보다 큼 | `inflation_radius` 0.45 → 0.25 | 경로 생성 |
| 회전 탐색 중 RC카가 탐지돼도 목표 고정(lock)이 안 걸림 | 회전 속도 · depth 토픽 지연 · 모션 블러 가설 3개 | 가설마다 결과가 달라지도록 탐색 변형 v1~v4와 판별 로그 설계 | 로봇 시험은 다음 단계 |

> **현장 관찰(09-03, 측정 기록 없음)** — 터미널 ping이 약 3,000 ~ 10,000 ms까지 튀었고, 로봇 카메라 토픽이 1분 이상 늦게 도착했습니다. 회전 탐색 중 전달되는 프레임이 너무 늦어 lock 판정이 걸리지 않은 것으로 보이며, 세 가설 중 depth · 이미지 토픽 지연 가설을 뒷받침하는 관찰입니다. ping · topic hz 같은 측정값은 남기지 않았습니다.

## 📊 결과

| 단계 | 결과 | 근거 |
|---|---|---|
| 인식 단독 (로봇 정지) | `RC` · `dummy` 구분 검출, 추적 ID 대체로 유지. 가림 시 RC를 놓치거나 ID가 바뀜 | 상단 GIF (09-01) |
| 웹캠 인식 → 트리거 → 액션 goal → 탐색 위치 이동 | 동작 | 당시 작업 기록 (09-03) |
| 탐색 위치에서 360° 회전 중 lock | 🟡 개선 과제 (탐지는 되지만 고정 판정이 걸리지 않음) | 당시 작업 기록 (09-03) |
| 접근 · 추종, 다른 RC카(디코이) 구분 | 다음 단계(로봇 시험 예정) | — |
| 탐색 변형 v1~v4 | 다음 단계(로봇 시험 예정) | — |

통합 테스트의 근거는 당시 정리한 작업 기록이며, 영상 · rosbag · 서버 로그는 남아 있지 않습니다.

## 🌱 배운 점 · 추후 개선 과제

- 실제 로봇에서는 인식 알고리즘보다 기동 순서와 파라미터에서 먼저 문제가 생겼습니다. 도킹 중에는 라이다가 멈춰 위치 추정이 안 됐고, 좁은 구역에서는 costmap 팽창 반경 때문에 경로가 나오지 않았습니다.
- 로봇을 세워 두고 통과한 인식도, 로봇이 회전하는 동안에는 회전 속도 · 이미지 지연 · 블러 같은 다른 조건에 놓입니다. 정지 상태 테스트만으로는 움직이는 로봇의 인식을 보장할 수 없었습니다.
- 문제가 생긴 직후 바로 고치기보다 가설을 나누고, 가설마다 결과가 달라지도록 변형과 로그를 먼저 설계했습니다. 로봇 시험 시간 확보가 다음 단계입니다.

**추후 개선 과제**

- 목표 고정(lock)이 안 된 원인은 가설 3개 중 확정하지 않았고, v1~v4와 접근 · 추종 단계의 로봇 시험이 다음 단계입니다.

- 무선 통신 지연 측정 · 기록(ping, `ros2 topic hz`, 프레임 수신 시각과 촬영 시각의 차이)을 시험 절차에 넣어, 지연 가설을 수치로 가릴 수 있게 합니다.
- 지연이 큰 환경에서는 인식을 로봇 쪽(라즈베리파이)에서 돌리거나, 이미지 압축 · 프레임 수 제한으로 무선 대역 사용을 줄이는 방향을 검토합니다.
- `relock_on_reacquire: true`라서 목표를 놓친 뒤 재탐색하면 다른 RC카를 고정할 수 있습니다. 더미는 추적 ID가 아니라 클래스 필터로 걸러집니다.
- v1~v4의 수동 회전(`cmd_vel`) 중 예외가 나면 정지 명령을 보내는 처리가 없습니다.
- `search_pose`, `initial_pose`는 교육장 지도 기준 좌표이고 `initial_pose`의 yaw는 추정치입니다. 맵 파일은 저장소에 없습니다.
- 그 밖에 확인된 개선 항목: [`docs/search_variants.md`](docs/search_variants.md) 6절

<details>
<summary>📂 폴더 구조</summary>

```text
tb4-rc-car-search-track/
├── README.md
├── src/
│   ├── rc_tracker_interfaces/      # SearchAndTrack.action
│   ├── rc_tracker/                 # 미션 패키지 기준판(v0): 노드 3개, launch 3개, config, 모델
│   ├── search_variants/            # 탐색 변형 v1~v4 — 기준판과 다른 파일만
│   └── tb4_yolo_tracker/           # 09-01 인식 프로토타입: YOLO + ByteTrack 추적 노드 (ROS 2 패키지)
└── docs/
    ├── images/
    │   └── perception_demo.gif     # 인식 단독 테스트 화면
    ├── run.md                      # 실행 순서와 문제 해결
    ├── search_variants.md          # v1~v4 변경 내역과 시험 방법
    ├── perception_prototype.md     # 인식 프로토타입 노드 설명과 실행
    ├── design_initial_0902.md      # 초기 인터페이스 설계 (최종 구현과 다른 점 표시)
    └── flowchart_initial_0902.drawio
```

</details>

## 🚀 실행 방법

맵은 저장소에 없습니다. 직접 만든 맵을 준비하고 `search_pose`, `initial_pose`를 그 맵 좌표로 바꿔야 합니다. 자세한 절차는 [`docs/run.md`](docs/run.md)에 있습니다.

```bash
# 빌드 (두 PC 모두)
pip install ultralytics opencv-python numpy
# 이 프로젝트 src/의 rc_tracker_interfaces, rc_tracker를 워크스페이스 src/에 넣고(프로토타입 tb4_yolo_tracker는 제외)
colcon build --symlink-install && source install/setup.bash

# ① 로봇 언도킹 (도킹 중에는 라이다가 멈춰 AMCL이 동작하지 않음)
ros2 action send_goal /robot2/undock irobot_create_msgs/action/Undock "{}"
# ② Nav2 + localization (GPU 노트북)
ros2 launch rc_tracker nav.launch.py namespace:=/robot2 map:=/path/to/your_map.yaml
# ③ 액션 서버 (GPU 노트북)
ros2 launch rc_tracker server.launch.py namespace:=/robot2
# ④ 웹캠 감지 + 트리거 (웹캠 PC)
ros2 launch rc_tracker station.launch.py
```

- 인식 프로토타입 노드 단독 실행: [`docs/perception_prototype.md`](docs/perception_prototype.md)
- 탐색 변형 v1~v4 시험: [`docs/search_variants.md`](docs/search_variants.md) 5절

## 📜 출처와 라이선스

- 코드는 저장소 루트 [LICENSE](../LICENSE)(Apache-2.0)를 따릅니다.
- `src/rc_tracker/config/nav2.yaml`: [turtlebot4_navigation](https://github.com/turtlebot/turtlebot4) 기본 설정(Apache-2.0)에서 `inflation_radius`만 바꿨습니다. 파일 상단에 출처를 적었습니다.
- 인식 모델 `rc_dummy_y11n.pt`: Ultralytics YOLO11n 파생 모델이라 AGPL-3.0을 따릅니다. 노드들도 Ultralytics 라이브러리(AGPL-3.0)를 사용합니다.
- depth → 3D → map 변환과 언도킹 · 초기 위치 설정 흐름은 강의 예제의 패턴을 참고했습니다. 강의 자료, 공식 패키지 코드, 팀원 개인 패키지는 넣지 않았습니다.
