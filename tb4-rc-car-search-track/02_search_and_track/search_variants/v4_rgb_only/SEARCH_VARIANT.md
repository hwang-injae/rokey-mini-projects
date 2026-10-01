# v4 「depth 없이 lock (RGB-only search)」

전체 비교표와 공통 변경점은 [`docs/search_variants.md`](../../../docs/search_variants.md)를 볼 것.

> 상태: **로봇 미검증.** 2026-09-04에 작성했지만 실제 로봇 비교 테스트는 하지 못했다.
> 이 폴더에는 기준판(`../../rc_tracker/`)과 다른 파일 2개(`search_and_track_server.py`, `mission_params.yaml`)와 이 설명만 있다.

## 아이디어

회전 동작은 **v1과 완전히 동일**(0.3 rad/s 저속 연속). 딱 하나 다름:
탐색 단계에서 lock을 **RGB 이미지 하나만으로** 판정한다. 원본은 RGB와 depth(compressedDepth)가
0.1초 안에 짝이 맞은 프레임에서만 인식이 돌아가서, **depth 토픽이 늦으면 인식이 아예 멈춘다.**
v4는 탐색 중 depth를 기다리지 않는다. (거리 측정이 필요한 접근·추종 단계는 원래대로 RGB+depth 동기화 사용.)

## 사용법: v1과 A/B 비교

- v1에서 실패 + v4에서 성공 → **통합 테스트 실패 원인은 depth 토픽 지연**.
- 둘 다 실패 → 원인은 회전 속도/블러/YOLO 쪽 → v2(끊어보기)로.

## 무엇이 바뀌었나

- `mission_params.yaml`: `search_rgb_only: true` ← v1과 유일한 차이.
- 코드: `on_rgb_only()` RGB 전용 구독 + `perceive()`가 탐색 중엔 그 프레임 사용.
- 공통 변경(원본 대비): lock "최고 conf RC 2연속", 탐색 중 conf 0.3, 상세 `[SEARCH]` 로그.

## 튜닝 포인트 (`config/mission_params.yaml`)

| 파라미터 | 기본 | 방향 |
|---|---|---|
| `search_rgb_only` | true | false로 두면 v1과 동일해짐 |
| `search_yaw_rate` | 0.30 | v1과 맞춰 비교할 것 |
| `search_conf_threshold` | 0.30 | 못 잡으면 0.2 |
| `use_stamped_cmd_vel` | true | 로봇이 안 돌면 false |
| `perception_stale_s` | 1.5 | 접근/추종: 이보다 낡은 프레임이면 goal 고정. 999=끔 |

## 접근/추종 이미지 지연 방어 (v1~v4 공통, 포함됨)

낡은 프레임이면 새 Nav2 goal을 안 보내고 방향 유지 → 신선해지면 재개, 5s 넘게 끊기면 재탐색.
`last_target_seen`도 신선한 프레임에서만 갱신. depth 좌표를 5s 넘게 못 구해도 재탐색.
참고: v4는 **탐색 단계만** RGB 단독. 추종 단계는 여전히 depth 필요(위 방어가 그때 작동).
자세한 내용은 [`docs/search_variants.md`](../../../docs/search_variants.md) 2-5절.

## 로그 확인

탐색 중 `[SEARCH] frame_age=...` 값이 v1에서는 크고(예: 0.5~2s) v4에서는 작으면(0.1s 내외)
depth 지연이 병목이었다는 직접 증거.
