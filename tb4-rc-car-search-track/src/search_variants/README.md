# 탐색 변형 v1~v4 (로봇 미검증)

2026-09-03 통합 테스트에서 회전 탐색 중 RC카 목표 고정(lock)이 걸리지 않은 뒤, 원인 가설을 하나씩 가르려고 **탐색 방식만 바꾼** 변형이다.
각 폴더에는 기준판(`../rc_tracker/`)과 다른 파일 2개(`search_and_track_server.py`, `mission_params.yaml`)와 설명(`SEARCH_VARIANT.md`)만 있다.

- [`v1_slow_spin/`](v1_slow_spin/SEARCH_VARIANT.md) — `cmd_vel` 0.3 rad/s 저속 연속 회전
- [`v2_step_scan/`](v2_step_scan/SEARCH_VARIANT.md) — 30° 회전 → 정지 → 0.8초 관찰 반복
- [`v3_detect_confirm/`](v3_detect_confirm/SEARCH_VARIANT.md) — 저속 회전 중 처음 보이면 정지하고 3초 확인
- [`v4_rgb_only/`](v4_rgb_only/SEARCH_VARIANT.md) — v1과 같은 회전 + 탐색 중 RGB만으로 lock

## 적용 방법

시험하려면 `search_and_track_server.py` → `rc_tracker/rc_tracker/`, `mission_params.yaml` → `rc_tracker/config/`에 덮어쓰고 다시 빌드한다.
명령 예시와 런마다 기록하려던 항목은 [`docs/search_variants.md`](../../docs/search_variants.md) 5절에 있다.

변형별로 가르려던 가설(비교표), 공통 변경점, 파라미터도 [`docs/search_variants.md`](../../docs/search_variants.md)에 정리했다.
