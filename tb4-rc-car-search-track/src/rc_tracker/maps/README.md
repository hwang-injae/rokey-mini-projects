# 맵

당시에는 교육장에서 SLAM으로 만든 지도(`teleop_map.yaml` + `.pgm`)를 썼다. 교육장 지도라 이 저장소에는 넣지 않았다.

- 직접 만든 맵을 쓰려면 `nav.launch.py`에 경로를 넘긴다.
  ```bash
  ros2 launch rc_tracker nav.launch.py namespace:=/robot2 map:=/path/to/your_map.yaml
  ```
- 또는 이 폴더에 `teleop_map.yaml`과 `.pgm`을 두고 다시 빌드하면 `nav.launch.py`의 기본 경로로 잡힌다.
- `config/mission_params.yaml`의 `search_pose`, `initial_pose`는 당시 지도 기준 좌표라 새 맵에서는 다시 정해야 한다 ([`docs/run.md`](../../../docs/run.md) 2절).
