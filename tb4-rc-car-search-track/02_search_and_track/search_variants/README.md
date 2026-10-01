# 탐색 변형 v1~v4 (로봇 미검증)

2026-09-03 통합 테스트에서 회전 탐색 중 RC카를 고정(lock)하지 못한 뒤, 원인 가설을 하나씩 가르려고 **탐색 방식만 바꾼** 변형이다.
각 폴더에는 기준판(`../rc_tracker/`)과 다른 파일 2개와 설명만 있다.

| 폴더 | 아이디어 | 가르려던 가설 |
|---|---|---|
| `v1_slow_spin/` | `cmd_vel` 0.3 rad/s 저속 연속 회전 | 회전 속도 |
| `v2_step_scan/` | 30° 회전 → 정지 → 0.8초 관찰 반복 | 모션 블러·지연 |
| `v3_detect_confirm/` | 저속 회전 중 처음 보이면 정지하고 3초 확인 | 빠른 탐지 + 정지 상태 확정 |
| `v4_rgb_only/` | v1과 같은 회전 + 탐색 중 RGB만으로 lock | depth 토픽 지연 (v1과 A/B) |

시험하려면 `search_and_track_server.py` → `rc_tracker/rc_tracker/`, `mission_params.yaml` → `rc_tracker/config/`에 덮어쓰고 다시 빌드한다.
공통 변경점과 파라미터는 [`docs/search_variants.md`](../../docs/search_variants.md)에 있다.
