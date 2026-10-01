> **초기 설계 문서 (2026-09-02 작성).** 구현 전에 정리한 인터페이스 설계라 최종 코드와 다른 부분이 있다.
> 플로우차트 원본은 [`flowchart_initial_0902.drawio`](flowchart_initial_0902.drawio)(draw.io로 열기)이며, 같은 시점의 설계다.
>
> 최종 구현과 다른 점
>
> | 항목 | 이 문서(초기 설계) | 최종 구현 (`02_search_and_track/`) |
> |---|---|---|
> | PC 배치 | GPU PC 2대, `mission_trigger`는 PC B | GPU 노트북(Nav2·서버) + GPU 없는 웹캠 PC. `mission_trigger`는 웹캠 PC에서 `station.launch.py`로 실행 |
> | 로봇 카메라 입력 | `rgb/preview` + `image_transport republish`로 depth 복원, 드라이버 해상도 맞춤(옵션 B) | 풀 RGB `rgb/image_raw/compressed` + `stereo/image_raw/compressedDepth`를 서버 안에서 직접 디코드하고 `ApproximateTimeSynchronizer`로 짝지음. 드라이버 설정은 바꾸지 않음 |
> | 인식 클래스 | COCO `car` / `rc_car` | 커스텀 모델 클래스 `RC`, `dummy` (`target_class: RC`) |
> | 시작 순서 | 도킹 상태에서 초기 위치 설정 → 언도킹 | **언도킹 → 초기 위치 설정 → Nav2 대기** (도킹 중에는 라이다가 멈춰 AMCL이 동작하지 않음) |
> | 추종 goal | target 좌표 자체 | target 앞 `stop_distance` 지점(stand-off) |
> | 기타 | `/webcam/detections`, `use_sim` 파라미터 | 구현하지 않음 |
>
> 본문의 `3_3_d`는 강의 day3 예제(depth → 3D → map 변환)를 가리킨다.

---

# 미니 프로젝트 인터페이스 정의 (웹캠 + TurtleBot4 RC카 추적)

맵: `teleop_map` (resolution 0.05, origin `[-4.658, -14.507, 0]`)
로봇 네임스페이스: `<ns>` = `/robot2`
`webcam_detector_node` · `mission_trigger_node` · `search_and_track_server` 는 **모두 PC 에서 실행** (로봇엔 새 노드 미배포, OAK-D 드라이버 파라미터만 로봇에서 수정 → §3.3 옵션 B).

## 1. 노드 구성 및 배치

**터틀봇 WiFi 에는 GPU PC 2대만 연결.** 가용 PC = 이 2대.

| 노드 | 실행 머신 | 역할 |
|---|---|---|
| `webcam_detector_node` | **GPU PC A** (USB 웹캠 DRGO-WC720 연결) | 웹캠 프레임 YOLO11n `car` 감지 → 트리거 1회 발행 |
| `mission_trigger_node` | **GPU PC B** | 트리거 수신 시 `SearchAndTrack` 액션 goal 1회 전송, feedback/result 로깅 |
| `search_and_track_server` | **GPU PC B** | 액션 서버. undock → search_pose 이동 → 360° 회전 탐색 → target 고정 → 접근/추종 |
| `image_transport republish` | **GPU PC B** | `<ns>/oakd/stereo/image_raw/compressedDepth` → 로컬 raw `Image` |

### 배치 (Deployment)

전제: 2대 PC + 로봇이 **같은 `ROS_DOMAIN_ID`, 터틀봇 WiFi 동일망, `ROS_LOCALHOST_ONLY=0`**.

| 머신 | 실행 |
|---|---|
| GPU PC A | `webcam_detector_node` (+ 웹캠)  ·  (부하 분산용) RViz / rqt 모니터링 |
| GPU PC B | `mission_trigger_node` + `search_and_track_server` + `image_transport republish` + **Nav2 + localization(AMCL, `teleop_map`) + `map_server`** |
| TurtleBot4 (RPi/Create3) | `turtlebot4` bringup: oakd 드라이버(preview·stereo 416 정렬), rplidar, create3 — **새 노드 미배포**, 드라이버 파라미터만 수정 |

- **`search_and_track_server` + Nav2 를 같은 PC(B)** → `goToPose` 재전송 · `/tf`(map→camera) 조회 · 액션 feedback 이 전부 로컬. 지연/`tf` 네트워크 부담 없음.
- **`mission_trigger_node` 도 B** → 액션 client↔server 로컬. 네트워크를 타는 건 웹캠 트리거 `Bool`(A→B) 하나뿐.
- **WiFi 구간(로봇 → PC B)**: `/compressed`(RGB) · `/compressedDepth`(depth) 로 받아 **PC B 에서 디코드/republish**. raw 를 WiFi 로 끌지 않는다.
- 자원: Nav2 = CPU 위주, YOLO 트래킹 = GPU 위주 → PC B 경합 적음. RViz 는 PC A 로 빼도 정상.

외부 스택(기존): Nav2 (`navigate_to_pose`, `spin`), `turtlebot4_navigation` docking (`dock`/`undock`), TF (`map`↔`odom`↔`base_link`↔`oakd_*_optical_frame`).

---

## 2. 커스텀 인터페이스 패키지: `rc_tracker_interfaces`

```
rc_tracker_interfaces/
└── action/
    └── SearchAndTrack.action
```

> 커스텀 msg 는 두지 않는다. 추적 결과는 `search_and_track_server` 가 **바운딩박스 + 상단에 `id / conf / class` 텍스트를 그려 넣은 이미지** (`<ns>/search_and_track/debug_image`) 로만 발행하고 rqt_image_view 로 확인한다. 추적 상태(현재 phase, target 거리, target map 좌표)는 액션 **feedback** 으로 전달된다.

### 2.1 action/SearchAndTrack.action

```
# ================= Goal =================
geometry_msgs/PoseStamped search_pose         # 웹캠이 바라보는 장소의 대략 좌표 (map). 도착 후 이 자리에서 회전 탐색
float32 stop_distance          # [m] RC카 정지/유지 거리 (기본 0.30)
float32 spin_angle             # [rad] 탐색 회전각 (기본 6.2832 = 360°)
float32 goal_update_distance   # [m] target 이 이만큼 움직여야 Nav2 goal 재전송 (기본 0.25)
float32 target_lost_timeout    # [s] target 미검출 지속 시 재탐색/종료 판단 (기본 5.0)
bool    return_to_dock_on_finish
---
# ================= Result =================
uint8 OUTCOME_UNKNOWN=0
uint8 OUTCOME_STOPPED_AT_TARGET=1   # stop_distance 도달·유지 후 정상 종료 (cancel 포함)
uint8 OUTCOME_TARGET_NOT_FOUND=2    # 360° 회전 완료했으나 rc_car 미발견
uint8 OUTCOME_TARGET_LOST=3         # 고정 후 timeout 이상 재검출 실패
uint8 OUTCOME_NAV_FAILED=4          # search_pose 이동 실패
uint8 OUTCOME_ABORTED=5

uint8                     outcome
int32                     target_track_id   # 고정된 ID (-1 = 끝까지 미고정)
geometry_msgs/PoseStamped final_pose        # 종료 시 로봇 pose (map)
float32                   final_distance     # 마지막으로 측정한 RC카 거리 [m]
string                    message
---
# ================= Feedback =================
uint8 PHASE_IDLE=0
uint8 PHASE_UNDOCKING=1
uint8 PHASE_NAV_TO_SEARCH=2
uint8 PHASE_SEARCHING=3       # 360° 회전 + 감지
uint8 PHASE_APPROACHING=4     # target 고정, 접근 주행
uint8 PHASE_FOLLOWING=5       # 이동하는 target 추종 (stop_distance 근처)
uint8 PHASE_HOLDING=6         # stop_distance 이내, target 정지 → 대기
uint8 PHASE_DOCKING=7

uint8                phase
int32                target_track_id
float32              distance_remaining     # 현재 Nav2 goal 까지 남은 거리 [m]
float32              target_distance        # depth 기준 RC카까지 거리 [m] (-1 = 미상)
geometry_msgs/Point  target_position_map    # target 현재 map 좌표
float32              search_progress        # 0.0 ~ 1.0 (회전 진행률)
```

---

## 3. 토픽

| 토픽 | 타입 | 발행 → 구독 | QoS | 설명 |
|---|---|---|---|---|
| `/webcam/rc_car_detected` | `std_msgs/Bool` | `webcam_detector` → `mission_trigger` | `transient_local`, depth 1 | RC카 최초 감지 시 `data=true` 1회 latch (도킹 해제 트리거) |
| `/webcam/image_raw/compressed` | `sensor_msgs/CompressedImage` | `webcam_detector` → (rqt) | sensor data | 디버그용 원본/오버레이 |
| `/webcam/detections` | `vision_msgs/Detection2DArray` | `webcam_detector` → (rqt) | default | 디버그용 감지 결과 (선택) |
| `<ns>/oakd/rgb/preview/image_raw/compressed` | `sensor_msgs/CompressedImage` (jpeg) | OAK-D → `search_and_track` | sensor data | RGB (YOLO 입력). preview = 정사각 저해상도 (예: 250×250) |
| `<ns>/oakd/stereo/image_raw/compressed` | `sensor_msgs/CompressedImage` (`compressedDepth`, 16UC1 mm) | OAK-D → `search_and_track` | sensor data | Depth. `png` 기반 compressedDepth 포맷 → 전용 디코드 필요 |
| `<ns>/oakd/rgb/preview/camera_info` | `sensor_msgs/CameraInfo` | OAK-D → `search_and_track` | default | preview RGB 기준 내부 파라미터 K (K도 preview 해상도에 맞음) |
| `<ns>/search_and_track/debug_image` | `sensor_msgs/Image` | `search_and_track` → (rqt) | sensor data | 전체 감지 bbox + 상단 `id·conf·class` 텍스트. target 은 색 강조 |
| `<ns>/search_and_track/target_point` | `geometry_msgs/PointStamped` | `search_and_track` → (RViz) | default | (선택) target 의 map 좌표 마커. 커스텀 아님 |

### 3.1 RGB(preview) ↔ Depth(stereo) 해상도 불일치 주의

preview RGB 와 stereo depth 는 **해상도·화각이 다르다.** YOLO 는 preview RGB 에서 돌리므로 박스 중심 픽셀 `(u_rgb, v_rgb)` 은 preview 좌표계 값이다. depth 이미지에서 거리를 뽑으려면 둘 중 하나:

1. **픽셀 스케일 변환** — `u_d = u_rgb * (W_depth / W_rgb)`, `v_d = v_rgb * (H_depth / H_rgb)` 로 대략 매핑 후 `z = depth[v_d, u_d]`. (두 스트림 화각이 같고 중심이 정렬됐다는 가정. 간단하지만 오차 있음)
2. **정렬된 depth 사용** — 드라이버에서 depth 를 RGB 프레임에 align 해주는 토픽이 있으면 그걸 구독 (해상도·좌표계가 RGB 와 동일해져 `(u_rgb, v_rgb)` 를 그대로 사용).

역투영에 쓰는 `fx, fy, cx, cy` 는 **픽셀을 읽은 이미지의 camera_info** 와 짝이 맞아야 한다:
- 방법 1: preview 픽셀 → preview `camera_info` 로 역투영, z 만 depth 에서 스케일 매핑해 가져옴.
- 방법 2: 정렬 depth 면 그 토픽의 `camera_info` 사용.

`compressedDepth` 디코드: `cv_bridge` 의 `compressed_imgmsg_to_cv2` 는 `compressedDepth` 를 바로 못 여는 경우가 있어, `image_transport` republish 로 `<ns>/oakd/stereo/image_raw` (raw `sensor_msgs/Image`) 를 같이 띄워 쓰는 편이 안전하다.

### 3.2 TurtleBot4 OAK-D 토픽 트리 (참고)

`ros2 topic list | grep oakd` 로 실제 확인. 드라이버(`depthai_ros_driver`) 기본 발행:

| 토픽 | 내용 | 비고 |
|---|---|---|
| `<ns>/oakd/rgb/image_raw` (+`/compressed`) | 풀 RGB (ISP) | 해상도 `rgb.i_width/i_height` 파라미터 |
| `<ns>/oakd/rgb/preview/image_raw` (+`/compressed`) | 정사각 preview | 크기 `rgb.i_preview_size` (기본 250) · 온카메라 NN 입력 |
| `<ns>/oakd/rgb/camera_info`, `.../preview/camera_info` | 각 스트림 K | preview K 는 preview 해상도 기준 |
| `<ns>/oakd/stereo/image_raw` (+`/compressedDepth`) | depth 16UC1 mm | 기본 `stereo.i_align_depth:=true` → **RGB(CAM_A) 프레임에 정렬**, 해상도는 stereo(기본 400P) |
| `<ns>/oakd/stereo/camera_info` | depth K | 정렬 depth 픽셀 역투영엔 이 K 사용 |
| `<ns>/oakd/left|right/image_raw` | 모노 | |
| `<ns>/oakd/nn/spatial_detections` | `depthai_ros_msgs/SpatialDetectionArray` | 온카메라 NN+depth 융합 3D 검출 (기본 mobilenet) |
| `<ns>/oakd/imu/data`, `<ns>/oakd/points` | IMU / 포인트클라우드 | points 는 기본 off |

### 3.3 RGB↔Depth 해상도 불일치 대응 옵션 (지연 최소화 우선)

| # | 방법 | 장점 | 비용 / 리스크 |
|---|---|---|---|
| **A** | **온카메라 spatial detection** — `nn/` 블롭을 yolo11n 으로 교체, `nn/spatial_detections` (+ 온카메라 `ObjectTracker` 로 지속 id) 구독. 이미지 대신 검출 메시지(수십 byte)만 흐름 | 이미지 대역폭 0, 해상도 문제 소멸, id 트래킹이 하드웨어 | yolo11n → OpenVINO → `.blob` 변환 필요. rqt 확인용으로 preview/compressed 만 저 fps 로 별도 발행 |
| **B** | **드라이버에서 RGB·depth 해상도 일치** — `rgb.i_preview_size` (또는 `rgb.i_width/i_height`) 와 `stereo.i_width/i_height` 를 같은 값(예 416²)으로. depth 는 이미 RGB 프레임 정렬 | 코드가 `3_3_d` 와 동일, `(u,v)` 로 depth 직접 인덱싱, 압축 jpeg 도 작음 | bringup yaml / `ros2 param` 로 드라이버 파라미터 수정 |
| **C** | **preview 로 YOLO + 해석적 매핑** — `u_d = u_prev · W_d/W_prev`, `v_d = v_prev · H_d/H_prev` (preview·depth 모두 동일 FOV 리사이즈일 때). 역투영 K 는 **`stereo/camera_info`**. bbox 영역 depth **중앙값** 사용 | 드라이버 설정 안 건드림, preview 대역폭 유지 | preview `keepAspectRatio=true` 면 크롭 오프셋 보정 필요(‌preview cx,cy 비교). 정렬·FOV 가정 어긋나면 오차 |
| **D** | **거리는 2D 라이다 `/scan`, 카메라는 방위각만** — bearing `θ=atan2(u−cx, fx)`, 그 방향 `/scan` 거리 사용 | 라이다는 빠르고 가볍고 근거리 정확, 해상도 무관 | RC카 높이 < 라이다 평면(≈0.15–0.2 m) 이면 미검출. depth 획득과 병용 권장 |

공통 지연 완화: `SensorDataQoS`(best-effort, depth 1) + 항상 최신 프레임만 처리(백로그 드롭), YOLO 는 별도 스레드, RGB+Depth 는 `ApproximateTimeSynchronizer(slop≈0.1)`.

**결정: 옵션 B.** 로봇 SSH 접근 가능 → OAK-D 드라이버 yaml 에서 `rgb.i_preview_size` 와 `stereo.i_width/i_height` 를 동일 크기(예 `416` / `416×416`)로 맞춤. depth 는 이미 RGB 프레임 정렬이므로 `(u,v)` 로 depth 직접 인덱싱, 역투영 K 한 개(`rgb/preview/camera_info` 또는 맞춘 `stereo/camera_info`)만 사용 → 코드는 `3_3_d` 와 거의 동일.
depth 압축 토픽(`stereo/image_raw/compressedDepth`)은 대역폭 위해 유지하되, launch 에서 `image_transport republish compressedDepth` 로 PC 로컬에서 raw 로 풀어 구독 (직접 헤더 디코드 불필요).

> 시뮬레이션 모드(`use_sim:=true`)에서는 카메라 토픽을 `/oakd/rgb/preview/image_raw`, `/oakd/rgb/preview/depth`, `/oakd/rgb/preview/camera_info` (모두 raw, 동일 해상도, depth 단위 m) 로 스위치.

---

## 4. 액션 / 서비스

| 이름 | 타입 | 클라이언트 → 서버 | 설명 |
|---|---|---|---|
| `/search_and_track` | `rc_tracker_interfaces/SearchAndTrack` | `mission_trigger` → `search_and_track_server` | 미션 전체 수행 |
| `<ns>/navigate_to_pose` | `nav2_msgs/NavigateToPose` | `search_and_track_server` → Nav2 | search_pose 이동 + 추종 중 goal 갱신 |
| `<ns>/spin` | `nav2_msgs/Spin` | `search_and_track_server` → Nav2 | 제자리 360° 탐색 회전 |
| `<ns>/undock` | `irobot_create_msgs/Undock` | `search_and_track_server` → create3 | 도킹 해제 (via `TurtleBot4Navigator.undock()`) |
| `<ns>/dock` | `irobot_create_msgs/Dock` | `search_and_track_server` → create3 | 종료 시 복귀 (선택) |

TF 요구: `search_and_track_server` 는 `camera optical frame → map` 변환 필요 (`tf2_ros.Buffer` + `TransformListener`).

---

## 5. 파라미터 (`search_and_track_server`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `use_sim` | `false` | 카메라 토픽 실기/시뮬 스위치 |
| `model_path` | `yolo11n.pt` | YOLO 가중치 경로 |
| `tracker_cfg` | `bytetrack.yaml` | `model.track(persist=True, tracker=...)` |
| `target_class` | `car` | 고정 대상 클래스명 (빨간 스포츠카형 RC카 → COCO `car`) |
| `conf_threshold` | `0.5` | 감지 신뢰도 하한 |
| `lock_consecutive_frames` | `3` | target 고정에 필요한 연속 감지 프레임 수 |
| `depth_min_m` / `depth_max_m` | `0.2` / `5.0` | 유효 depth 범위 |
| `stop_distance_m` | `0.30` | RC카 정지 거리 (Goal `stop_distance` 로 override) |
| `stop_hysteresis_m` | `0.10` | HOLDING → APPROACHING 복귀 여유 |
| `goal_update_distance_m` | `0.25` | Nav2 goal 재전송 임계 이동량 |
| `follow_goal_period_s` | `0.5` | 추종 루프 주기 |
| `spin_angle_rad` | `6.2832` | 탐색 회전각 |
| `target_lost_timeout_s` | `5.0` | 재탐색/종료 판단 시간 |
| `map_frame` | `map` | 목표 좌표계 |

파라미터 (`webcam_detector_node`): `camera_index`, `model_path`, `target_class=car`, `conf_threshold=0.5`, `consecutive_frames`, `publish_debug_image`.

파라미터 (`mission_trigger_node`): `search_pose` = `[x, y, yaw]` (teleop_map 상 웹캠 감시 지점 — **추후 확정**), `stop_distance=0.30`, `spin_angle=6.2832`, `return_to_dock_on_finish`.

---

## 6. 디코이 처리 규칙 (진짜 RC카 선정 = "최초 감지 고정")

1. `PHASE_SEARCHING` 회전 중, `class == "car"` && `confidence >= conf_threshold` 인 감지가 `lock_consecutive_frames` 연속 발생하면 그 `track_id` 를 `target_track_id` 로 **고정**. (여러 대가 동시에 보이면 화면 중심에 가장 가까운 것)
2. 고정 이후 모든 단계에서 **`track_id == target_track_id` 인 객체만** 위치/거리 계산에 사용. 다른 `car` track_id(다른 RC카), 더미박스, 기타 장애물은 무시(주행 회피는 Nav2 costmap 이 담당).
3. `target_track_id` 가 `target_lost_timeout_s` 이상 프레임에서 사라지면 → `PHASE_SEARCHING` 재진입(제자리 재탐색). 재탐색에서도 못 찾으면 `OUTCOME_TARGET_LOST` 로 종료.
