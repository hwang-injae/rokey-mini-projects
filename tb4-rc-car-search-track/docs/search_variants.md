# 탐색 변형 v1~v4 — 변경 내역 (2026-09-04 작성, 로봇 미검증)

> 이 문서는 통합 테스트 실패 직후 다음 로봇 테스트를 위해 정리한 계획과 변경 내역이다.
> 시간이 부족해 **v1~v4 모두 로봇에서 비교 테스트를 하지 못했다.** 아래의 "언제 유리한가", "기대" 같은 표현은 가설이다.

## 0. 배경 (2026-09-03 통합 테스트)

- AMR이 액션 goal을 받고 `search_pose`(웹캠이 보는 구역)까지 이동하는 것은 **성공**
- 그 자리에서 **360° 회전하며 RC카를 lock(목표 고정)하는 데 실패** (탐지는 되는데 "찾았다" 판정이 걸리지 않음)
- 접근·추종(APPROACHING/FOLLOWING) 단계는 **테스트하지 못함**

원인 후보

1. 회전이 빨라 RC카가 화면을 금방 지나감
2. depth 이미지 토픽 지연으로 인식 파이프라인이 새 프레임을 못 받음
3. 회전 중 모션 블러로 YOLO 신뢰도 하락

→ 탐색 방식만 다른 변형 4개를 만들어 하나씩 비교하려고 했다. 추종 단계에도 이미지 지연 방어를 추가했다.

## 1. 변형 목록

| 폴더 | 이름 | 핵심 아이디어 | 가르려던 가설 |
|---|---|---|---|
| `rc_tracker/` (기준판) | v0 | Nav2 `/spin` 360°, "화면 중앙에 가장 가까운 id 3연속" | 비교 기준 |
| `search_variants/v1_slow_spin/` | v1 느린 회전 | `cmd_vel`로 직접 0.3 rad/s 저속 연속 회전 | 회전 속도 |
| `search_variants/v2_step_scan/` | v2 끊어보기 | 30° 돌고 → 멈춰서 0.8 s 관찰 → 반복(12스텝) | 모션 블러·지연 |
| `search_variants/v3_detect_confirm/` | v3 보이면 정지 | 저속 회전 중 RC가 처음 보이면 멈추고 최대 3 s 확인 | 빠른 탐지 + 정지 상태 확정 |
| `search_variants/v4_rgb_only/` | v4 depth 없이 lock | v1과 같은 회전 + 탐색 중에는 RGB만으로 lock | depth 토픽 지연 (v1과 A/B) |

각 변형은 기준판과 **`search_and_track_server.py`, `mission_params.yaml` 두 파일만** 다르다
(`webcam_detector`, `mission_trigger`, `rc_tracker_interfaces`, launch, 모델은 동일).

## 2. 공통 변경점 (v1~v4)

### 2-1. lock 판정 완화 — `_update_lock()`

| 항목 | 기준판 | 변경 |
|---|---|---|
| 후보 선택 | 화면 **중앙에 가장 가까운** RC | **신뢰도(conf)가 가장 높은** RC (회전 중에는 정중앙을 거의 지나지 않음) |
| 연속 프레임 수 | 3 | **2** (`lock_consecutive_frames`) |
| streak 유지 | 후보 id가 바뀌면 즉시 리셋 | 진행 중인 streak의 id가 아직 보이면 유지 (1프레임 흔들림에 리셋되지 않음) |

### 2-2. 탐색 중 YOLO 신뢰도 하향

- `perceive()`가 phase에 따라 conf를 바꾼다: SEARCHING = `search_conf_threshold`(0.30), 그 외 = `conf_threshold`(0.5)
- 회전으로 흐려진 프레임도 잡되, 접근·추종은 기존대로 엄격하게.

### 2-3. 탐색 상세 로그 — `perceive()`

SEARCHING 동안 0.4초마다 한 줄씩 남긴다.

```
[SEARCH] frame_age=0.12s rgb_only=False conf_th=0.30 n=1 ids=[3] conf=[0.71] ctr=[(402,331)] streak=3x2 turned=90deg
```

- `frame_age`: 지금 쓰는 이미지가 몇 초 전 것인가 (크면 토픽 지연)
- `n` / `ids`: RC 탐지 수 / 추적 id (줄마다 id가 바뀌면 tracker 불안정)
- `streak`: lock까지 몇 프레임 쌓였나
- 프레임이 아예 없으면 `[SEARCH] no frame yet (...)`

실패해도 로그만 보고 "프레임이 안 온 건지 / YOLO가 못 잡은 건지 / id가 튀는 건지"를 가를 수 있게 하려는 목적이다.

### 2-4. 회전을 `cmd_vel`로 직접 제어

- Nav2 `/spin`(behavior_server 설정으로 약 1.0 rad/s) 대신 `<ns>/cmd_vel`에 직접 발행해 속도를 파라미터로 조절한다.
- 추가한 헬퍼: `_ang_diff` `_current_yaw` `_send_yaw` `_stop_motion` `_search_reset` `_locked` `_sighted` `_set_progress` `_turn_by` `_dwell` `_search_finalize`
- 회전 각도는 `map -> base_link` TF의 yaw 변화로 적산한다. TF가 없으면 `속도 × 시간`으로 근사한다.
- `use_stamped_cmd_vel`(기본 true): TurtleBot4 Jazzy는 `TwistStamped`를 쓴다. 로봇이 돌지 않으면 `false`(plain `Twist`)로 바꾼다.

### 2-5. 접근·추종 이미지 지연 방어 — `_run_mission()`, `perceive()`, `_publish_fb()`

이미지가 밀릴 때 **옛날 좌표로 달려가지 않게** 했다.

| 상황 | 기준판 | 변경 |
|---|---|---|
| 최신 프레임이 `perception_stale_s`(1.5 s)보다 오래됨 | 낡은 좌표로 계속 goal 전송 | 새 goal을 보내지 않고 현재 방향 유지. 신선해지면 자동 재개 |
| 이미지 완전 정지 | 낡은 프레임에서도 "보고 있다"고 판단해 재탐색이 안 걸림 | `last_target_seen`을 신선한 프레임에서만 갱신 → `target_lost_timeout`(5 s) 후 재탐색 |
| target은 보이는데 depth 좌표를 5 s 넘게 못 구함 | 무한 대기 | 재탐색으로 복구 (`no depth fix on target`) |
| feedback `target_distance` | 낡은 값 노출 | 낡은 동안 `-1` |

`perception_stale_s`를 크게(예: 999) 두면 기준판 동작과 같아진다.

### 2-6. `_spin_search()` 교체

변형마다 본문이 다르다. 시그니처(`goal_handle, spin_angle, fresh`)와 반환값(획득 시 True)은 기준판과 같아서,
`_run_mission`의 첫 탐색·놓친 뒤 재탐색 호출부는 고치지 않았다.

## 3. 변형별 차이

- **v1 느린 연속 회전**: `_turn_by(전체 각도)` 한 번. 0.3 rad/s로 360°에 약 21초.
- **v2 끊어보기**: `scan_step_deg`(30°) 회전 → 정지 → `scan_dwell_s`(0.8 s) 관찰을 12번 반복. 탐색 중 RC카가 멈춰 있는 시나리오라 블러·지연 영향이 가장 작을 것으로 보고 1순위 후보로 두었다.
- **v3 보이면 정지 후 확인**: 15°씩 저속 회전하다가 RC가 한 프레임이라도 보이면 즉시 정지 → `confirm_timeout_s`(3 s) 동안 lock 시도 → 실패하면 회전 재개. 무한 반복을 막는 전체 시간 상한이 있다.
- **v4 depth 없이 lock**: 회전은 v1과 같고 `search_rgb_only: true`만 다르다. 탐색 중에는 RGB 단독 구독(`on_rgb_only`)으로 lock을 판정한다. v1은 실패하고 v4는 성공하면 depth 토픽 지연이 원인이라고 판단하려 했다.

## 4. 파라미터 (각 변형 `mission_params.yaml` 하단 블록)

| 파라미터 | v1 | v2 | v3 | v4 | 의미 |
|---|---|---|---|---|---|
| `lock_consecutive_frames` | 2 | 2 | 2 | 2 | lock까지 필요한 연속 프레임 |
| `search_conf_threshold` | 0.30 | 0.30 | 0.30 | 0.30 | 탐색 중 YOLO conf |
| `search_yaw_rate` [rad/s] | 0.30 | 0.50 | 0.30 | 0.30 | 회전 속도 (v2는 스텝 이동 속도) |
| `scan_step_deg` | – | 30 | – | – | v2: 스텝당 각도 |
| `scan_dwell_s` | – | 0.8 | – | – | v2: 스텝 후 관찰 시간 |
| `confirm_timeout_s` | – | – | 3.0 | – | v3: 감지 후 확인 시간 |
| `search_rgb_only` | false | false | false | true | v4 스위치 |
| `use_stamped_cmd_vel` | true | true | true | true | 로봇이 안 돌면 false |
| `perception_stale_s` [s] | 1.5 | 1.5 | 1.5 | 1.5 | 접근·추종: 이보다 낡은 프레임이면 goal 고정 |

## 5. 변형 시험 방법

1. 기준판으로 빌드되는 워크스페이스를 준비한다 ([`run.md`](run.md) 1절).
2. 시험할 변형의 파일 2개를 덮어쓴다.
   ```bash
   V=<repo>/tb4-rc-car-search-track/02_search_and_track/search_variants/v2_step_scan
   cp $V/search_and_track_server.py ~/rc_ws/src/rc_tracker/rc_tracker/
   cp $V/mission_params.yaml        ~/rc_ws/src/rc_tracker/config/
   cd ~/rc_ws && colcon build --symlink-install && source install/setup.bash
   ```
3. [`run.md`](run.md)의 순서대로 실행하고, 서버 로그를 파일로 남긴다.
   ```bash
   ros2 launch rc_tracker server.launch.py namespace:=/robot2 2>&1 | tee v2_run1.log
   ```
4. 런마다 기록하려던 것: 변형·파라미터 값 / lock 성공 여부와 걸린 시간 / `[SEARCH]` 줄의 `frame_age`·`n`·`ids` / `debug_image`에서 회전 중 RC 박스가 보이는지와 conf

수동 `cmd_vel` 관련 주의

- 로봇이 아예 안 돎 → `use_stamped_cmd_vel: false`로 바꾸고 재빌드
- 로봇이 끊기며 돎 → Nav2 `collision_monitor`/`velocity_smoother`와 `cmd_vel`이 경합할 수 있다. 이때는 기준판(Nav2 `/spin`)을 쓰고 `behavior_server`의 `max_rotational_vel`을 1.0 → 0.4로 낮춰 시험
- 회전 각도가 안 맞음 → `map -> base_link` TF 확인, 로그의 `turned=...deg` 확인

## 6. 발견했지만 고치지 않은 것

1. `waitUntilNav2Active()`가 영구 대기할 수 있다. Nav2/localization이 떠 있지 않거나 AMCL 초기 위치가 없으면 `execute()`가 끝나지 않고, 이후 goal은 모두 거절된다.
2. 언도킹 후 `time.sleep(3.0)` 고정. 라이다가 늦게 돌기 시작하면 AMCL이 실패할 수 있다.
3. `initial_pose`의 yaw `2.382448`은 도킹 상태 yaw를 쓴 추정치다. 틀리면 탐색 위치로 엉뚱하게 갈 수 있다.
4. 추종 단계 `conf_threshold: 0.5`가 엄격하다. RC conf가 5 s 동안 0.5 미만이면 놓친 것으로 판정한다.
5. `camera_frame`이 `oakd_rgb_camera_optical_frame`으로 고정돼 있다. 로봇의 실제 프레임명이 다르면 map 좌표 변환이 모두 실패한다.
6. `webcam_detector`는 GPU 없는 노트북에서 돌았다. `frame_rate: 15`여도 실제로는 더 느릴 수 있어, `consecutive_frames: 5`면 RC카가 빨리 지나갈 때 트리거가 안 걸릴 수 있다.
7. `lock_on_start: true`라 미션 goal 전에도 RC가 보이면 lock한다(goal이 오면 리셋되므로 미션에는 영향 없음). 단독 테스트에서는 "일찍 잡힌 것처럼" 보인다.
8. v1~v4의 수동 회전 중 예외가 나면 정지 명령을 보내는 처리가 없다(`execute()`의 `finally`에 정지 호출 없음).
