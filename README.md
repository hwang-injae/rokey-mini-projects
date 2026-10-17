<div align="center">

# ROKEY Mini Projects

**두산로보틱스 ROKEY 9기(2026.04 ~ 2026.10)에서 큰 팀 프로젝트 사이사이에 진행한 미니프로젝트와 스터디 프로젝트 모음입니다.**

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=flat-square&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-rclpy-3776AB?style=flat-square&logo=python&logoColor=white)
![Doosan](https://img.shields.io/badge/Doosan-M0609-0072CE?style=flat-square)
![Isaac Sim](https://img.shields.io/badge/NVIDIA-Isaac_Sim-76B900?style=flat-square&logo=nvidia&logoColor=white)
![TurtleBot4](https://img.shields.io/badge/TurtleBot4-Nav2-2E7D32?style=flat-square)
![MediaPipe](https://img.shields.io/badge/MediaPipe-Hands-0097A7?style=flat-square&logo=mediapipe&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-D22128?style=flat-square&logo=apache&logoColor=white)

</div>

<br>

각 폴더의 README에 개요 · 배경과 목표 · 구성 · 내가 한 것 · 결과 · 배운 점 · 한계를 같은 양식으로 정리했습니다.

| 미니프로젝트 | 내용 | 스택 | 상태 |
|---|---|---|---|
| [M0609 기어 조립: DRL → ROS 2 Python](m0609-gear-drl-to-ros2/) | 티치 펜던트로 만든 기어 조립 동작을 ROS 2 Python 노드로 옮기면서 달라진 점을 고쳐 실제 로봇에서 끝까지 동작 (협동-1 사전 실습 · 개인) | Doosan M0609 · RG2 · DSR_ROBOT2 · 힘제어 | ✅ 실로봇 완주 1회 · 🎬 영상 |
| [Isaac Sim M0609 색상 분류](isaac-m0609-color-sort/) | 시뮬레이션 속 로봇팔이 큐브 색에 맞는 마커로 옮기도록 카메라 영상에서 파랑·초록을 판별하는 ROS 2 색 감지 노드 (협동-3 사전 실습 · 4인 · 담당: 반대편 PC 팀원이 만든 노드를 시나리오에 맞게 수정하고 Isaac Sim과 연동) | Isaac Sim · ROS 2 · OpenCV(HSV) | ✅ 전체 동작 확인 |
| [TurtleBot4 RC카 탐색·추적](tb4-rc-car-search-track/) | 고정 웹캠이 RC카를 발견하면 TurtleBot4가 출동해 회전 탐색 후 다가가 따라가게 하려던 ROS 2 미션 (지능-1 팀 미니프로젝트 · 4인 팀 · 담당: 객체 탐지 — 웹캠 RC카·더미 박스 인식) | TurtleBot4 · Nav2 · YOLO11n · ByteTrack | 🟡 미완성 (탐색 위치 이동까지, lock 실패) |
| [제스처로 제어하는 물체 추적 로봇](gesture-controlled-robot-tracking/) | 손 제스처(보자기 → 시작, 주먹 → 정지)로 추적을 켜고 끄면, 카메라가 찾은 빨간 공을 turtlesim · Gazebo 로봇이 따라감 (ROKEY 스터디 · 5인 팀 · 담당: 상태·이동 제어 — 비례 제어, 상태 머신, 안전 정지, pytest) | ROS 2 · OpenCV · MediaPipe · Gazebo | ✅ 팀 통합 시연 |

> 메인 팀 프로젝트는 [프로필](https://github.com/hwang-injae)에서 볼 수 있습니다.

## 라이선스

- 이 저장소의 코드는 [Apache License 2.0](LICENSE)을 따릅니다.
- 다만 YOLO 모델 가중치(`rc_dummy_y11n.pt`)는 Ultralytics YOLO11n을 파인튜닝한 것이라 AGPL-3.0을 따르고, 인식 노드는 Ultralytics 라이브러리(AGPL-3.0)를 사용합니다.
- `gesture-controlled-robot-tracking/`은 원본 스터디 팀 저장소의 라이선스인 MIT를 따릅니다.
- 각 폴더의 "출처와 라이선스" 절에 서드파티 출처를 적었습니다.
