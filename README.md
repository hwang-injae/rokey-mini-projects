# ROKEY Mini Projects

두산로보틱스 ROKEY 9기(2026.04 ~ 2026.10)에서 큰 팀 프로젝트 사이사이에 진행한 미니프로젝트 모음입니다.
각 폴더에 목표 · 내가 한 것 · 결과 · 배운 점 · 한계를 정리했습니다.

| 미니프로젝트 | 내용 | 스택 | 상태 |
|---|---|---|---|
| [M0609 기어 조립: DRL → ROS 2 Python](m0609-gear-drl-to-ros2/) | 티치 펜던트로 만든 기어 조립 동작을 ROS 2 Python 노드로 옮기면서 달라진 점을 고쳐 실제 로봇에서 끝까지 동작 | Doosan M0609 · RG2 · DSR_ROBOT2 · 힘제어 | ✅ 실로봇 완주 1회 |
| [TurtleBot4 RC카 탐색·추적](tb4-rc-car-search-track/) | 웹캠이 RC카를 인식하면 AMR이 출동해 회전 탐색 후 추종하는 4인 팀 미니프로젝트 (담당: 웹캠 RC카·더미 박스 인식) | TurtleBot4 · Nav2 · YOLO11n · ByteTrack | 🟡 미완성 (탐색 위치 이동까지, lock 실패) |

> 메인 팀 프로젝트는 [프로필](https://github.com/hwang-injae)에서 볼 수 있습니다.

## 라이선스

- 이 저장소의 코드는 [Apache License 2.0](LICENSE)을 따릅니다.
- 다만 YOLO 모델 가중치(`rc_dummy_y11n.pt`)는 Ultralytics YOLO11n을 파인튜닝한 것이라 AGPL-3.0을 따르고, 인식 노드는 Ultralytics 라이브러리(AGPL-3.0)를 사용합니다.
- 각 폴더의 "출처와 라이선스" 절에 서드파티 출처를 적었습니다.
