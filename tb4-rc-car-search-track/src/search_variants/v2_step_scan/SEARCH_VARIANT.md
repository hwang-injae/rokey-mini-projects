# v2 「끊어서 보기 (step-and-scan)」 — 당시 1순위 후보

전체 비교표와 공통 변경점은 [`docs/search_variants.md`](../../../docs/search_variants.md)를 볼 것.

> 상태: **로봇 미검증.** 2026-09-04에 작성했지만 실제 로봇 비교 테스트는 하지 못했다.
> 이 폴더에는 기준판(`../../rc_tracker/`)과 다른 파일 2개(`search_and_track_server.py`, `mission_params.yaml`)와 이 설명만 있다.

## 아이디어

30° 회전 → **완전히 멈춤** → 0.8초 동안 가만히 보며 탐지 → 다음 30°. 12스텝으로 360°.
RC카가 탐색 중 정지해 있는 우리 시나리오에서는 **모션 블러도 없고 프레임 지연도 상쇄돼서** 가장 잘 잡힐 것.

## 무엇이 바뀌었나

- `_spin_search()` = 스텝 회전 + 스텝마다 `_dwell()` 정지 관찰.
- 공통 변경(원본 대비): lock "최고 conf RC 2연속", 탐색 중 conf 0.3, 상세 `[SEARCH]` 로그.
- 회전은 `/robot2/cmd_vel` 직접 발행.

## 튜닝 포인트 (`config/mission_params.yaml`)

| 파라미터 | 기본 | 방향 |
|---|---|---|
| `scan_step_deg` | 30 | RC카가 좁게 보이면 20으로 촘촘히 |
| `scan_dwell_s` | 0.8 | 못 잡으면 1.2~1.5로 늘리기 (제일 효과 큼) |
| `search_yaw_rate` | 0.50 | 스텝 이동 속도. 각 스텝이 튀면 0.35 |
| `lock_consecutive_frames` | 2 | 정지 관찰이라 2로 충분, 오탐이면 3 |
| `use_stamped_cmd_vel` | true | 로봇이 안 돌면 false |
| `perception_stale_s` | 1.5 | 접근/추종: 이보다 낡은 프레임이면 goal 고정. 999=끔 |

## 접근/추종 이미지 지연 방어 (v1~v4 공통, 포함됨)

낡은 프레임이면 새 Nav2 goal을 안 보내고 방향 유지 → 신선해지면 재개, 5s 넘게 끊기면 재탐색.
`last_target_seen`도 신선한 프레임에서만 갱신. depth 좌표를 5s 넘게 못 구해도 재탐색.
자세한 내용은 [`docs/search_variants.md`](../../../docs/search_variants.md) 2-5절.
