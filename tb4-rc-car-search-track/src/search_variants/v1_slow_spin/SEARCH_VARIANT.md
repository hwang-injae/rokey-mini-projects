# v1 「느린 연속 회전」

전체 비교표와 공통 변경점은 [`docs/search_variants.md`](../../../docs/search_variants.md)를 볼 것.

> 상태: **로봇 미검증.** 2026-09-04에 작성했지만 실제 로봇 비교 테스트는 하지 못했다.
> 이 폴더에는 기준판(`../../rc_tracker/`)과 다른 파일 2개(`search_and_track_server.py`, `mission_params.yaml`)와 이 설명만 있다.

## 무엇이 바뀌었나 (원본 대비)

- 탐색 단계 회전을 Nav2 `/spin`(약 1.0 rad/s) 대신 **`/robot2/cmd_vel` 직접 발행으로 0.3 rad/s 저속 연속 회전**.
  → 360°에 약 21초. RC카가 화면에 오래 머물러 lock 확률 ↑.
- lock 판정: "화면중앙 최근접 3연속" → **"최고 conf RC 2연속(흔들림 허용)"**.
- 탐색 중 YOLO conf 0.3.
- 파일: 이 폴더의 `search_and_track_server.py`(`_spin_search()` + 헬퍼, `_update_lock()`, `perceive()` 로깅),
  설정은 `mission_params.yaml` 맨 아래 블록.

## 튜닝 포인트 (`config/mission_params.yaml`)

| 파라미터 | 기본 | 방향 |
|---|---|---|
| `search_yaw_rate` | 0.30 | 더 느리게 0.2 / 더 빠르게 0.5 |
| `lock_consecutive_frames` | 2 | 오탐 잦으면 3 |
| `search_conf_threshold` | 0.30 | 못 잡으면 0.2, 오탐이면 0.4 |
| `use_stamped_cmd_vel` | true | 로봇이 안 돌면 false |
| `perception_stale_s` | 1.5 | 접근/추종: 이보다 낡은 프레임이면 goal 고정. 999=끔 |

## 접근/추종 이미지 지연 방어 (v1~v4 공통, 포함됨)

낡은 프레임이면 새 Nav2 goal을 안 보내고 방향 유지 → 신선해지면 재개, 5s 넘게 끊기면 재탐색.
`last_target_seen`도 신선한 프레임에서만 갱신. depth 좌표를 5s 넘게 못 구해도 재탐색.
자세한 내용은 [`docs/search_variants.md`](../../../docs/search_variants.md) 2-5절.
