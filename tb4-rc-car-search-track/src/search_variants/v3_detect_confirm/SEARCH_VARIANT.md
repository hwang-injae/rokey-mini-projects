# v3 「보이면 멈추고 확인 (detect-then-confirm)」

전체 비교표와 공통 변경점은 [`docs/search_variants.md`](../../../docs/search_variants.md)를 볼 것.

> 상태: **로봇 미검증.** 2026-09-04에 작성했지만 실제 로봇 비교 테스트는 하지 못했다.
> 이 폴더에는 기준판(`../../rc_tracker/`)과 다른 파일 2개(`search_and_track_server.py`, `mission_params.yaml`)와 이 설명만 있다.

## 아이디어

저속으로 계속 돌다가 **RC가 한 프레임이라도 잡히는 순간 즉시 정지**,
그 자리에서 최대 3초 가만히 보며 lock(2연속) 시도. 3초 안에 못 잡으면 다시 회전 재개.
v1보다 빠르게 반응하면서 v2처럼 정지 상태에서 확정.

## 무엇이 바뀌었나

- `_spin_search()` = 15°씩 저속 회전(`stop_on_any`) → 감지 시 `_dwell(confirm_timeout_s)`.
- 공통 변경(원본 대비): lock "최고 conf RC 2연속", 탐색 중 conf 0.3, 상세 `[SEARCH]` 로그.
- 회전은 `/robot2/cmd_vel` 직접 발행. 전체 시간 상한(약 total/rate×4 + 60s) 있음.

## 튜닝 포인트 (`config/mission_params.yaml`)

| 파라미터 | 기본 | 방향 |
|---|---|---|
| `confirm_timeout_s` | 3.0 | id가 잘 안 붙으면 4~5로 |
| `search_yaw_rate` | 0.30 | 감지 전 회전 속도 |
| `search_conf_threshold` | 0.30 | 첫 감지가 늦으면 0.2 |
| `lock_consecutive_frames` | 2 | 오탐이면 3 |
| `use_stamped_cmd_vel` | true | 로봇이 안 돌면 false |
| `perception_stale_s` | 1.5 | 접근/추종: 이보다 낡은 프레임이면 goal 고정. 999=끔 |

## 접근/추종 이미지 지연 방어 (v1~v4 공통, 포함됨)

낡은 프레임이면 새 Nav2 goal을 안 보내고 방향 유지 → 신선해지면 재개, 5s 넘게 끊기면 재탐색.
`last_target_seen`도 신선한 프레임에서만 갱신. depth 좌표를 5s 넘게 못 구해도 재탐색.
자세한 내용은 [`docs/search_variants.md`](../../../docs/search_variants.md) 2-5절.

## 주의

RC가 계속 보이는데 id가 매 프레임 튀어서 lock이 안 되면 → confirm이 반복되며 회전이 거의 안 나아감.
로그에 `[SEARCH v3] confirm timed out` 이 여러 번 찍히면 `confirm_timeout_s`를 늘리거나 v2로 전환.
