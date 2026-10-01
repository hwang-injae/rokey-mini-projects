# 인식 모델

| 파일 | 내용 |
|---|---|
| `rc_dummy_y11n.pt` (약 5.2 MB) | RC카·더미 박스 2클래스(`RC`, `dummy`) 검출 모델. Ultralytics YOLO11n(`yolo11n.pt`)을 팀이 직접 촬영·라벨링한 데이터로 파인튜닝했다 (학습 실행 이름 `y11n_ep100_b32_img640_lr01`, 2026-09-01) |

- 팀이 함께 만든 모델이다. 웹캠 감지(`webcam_detector`)와 로봇 카메라 추적(`search_and_track_server`)이 같은 파일을 쓴다.
- config에서는 파일 이름만 적는다(`model_path: "rc_dummy_y11n.pt"`). 이름만 주면 `rc_tracker/model_paths.py`가 설치된 패키지의 `models/`에서 찾는다.

## 라이선스

이 가중치는 Ultralytics YOLO11에서 파생된 모델이라 **AGPL-3.0**을 따른다 (Ultralytics 라이선스: https://ultralytics.com/license).
코드 패키지의 `Apache-2.0` 표기는 이 가중치에는 적용되지 않는다.
