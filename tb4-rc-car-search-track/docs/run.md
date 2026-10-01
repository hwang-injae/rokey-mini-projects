# 실행 방법

ROS 2 **Jazzy**, 로봇 네임스페이스 **`/robot2`** 기준. 당시 구성은 PC 2대였다.

| 표기 | 장비 | 실행하는 것 |
|---|---|---|
| GPU 노트북 | Nav2·localization·액션 서버 | `nav.launch.py`, `server.launch.py`, RViz |
| 웹캠 PC | USB 웹캠을 꽂은 일반 노트북 | `station.launch.py` (웹캠 감지 + 미션 트리거) |
| 로봇 | TurtleBot4 (OAK-D, RPLIDAR) | 기본 bringup만. 새 노드를 배포하지 않음 |

장시간 실행하는 launch는 터미널을 따로 연다. 모든 터미널에서 0절의 환경 설정을 먼저 한다.

---

## 0. 공통 환경 (모든 터미널)

```bash
source /opt/ros/jazzy/setup.bash
source /etc/turtlebot4_discovery/setup.bash   # TurtleBot4 네트워크 설정을 쓰는 경우
source ~/rc_ws/install/setup.bash             # 아래 1절에서 만든 워크스페이스
```

```bash
ros2 topic list | grep /robot2 | head          # 로봇 토픽이 보여야 함
```

## 1. 빌드 (두 PC 모두, 최초 1회)

```bash
pip install ultralytics opencv-python numpy   # rosdep으로 설치되지 않음

mkdir -p ~/rc_ws/src
cp -r <repo>/tb4-rc-car-search-track/02_search_and_track/rc_tracker_interfaces ~/rc_ws/src/
cp -r <repo>/tb4-rc-car-search-track/02_search_and_track/rc_tracker            ~/rc_ws/src/
cd ~/rc_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

확인

```bash
ros2 interface show rc_tracker_interfaces/action/SearchAndTrack | head -5
ros2 pkg executables rc_tracker
```

## 2. 맵과 좌표 준비 (최초 1회)

- **맵은 이 저장소에 없다.** 교육장에서 SLAM으로 만든 지도라 넣지 않았다. 직접 만든 맵(`.yaml` + `.pgm`)을 준비해 `map:=`로 넘긴다.
- `config/mission_params.yaml`의 좌표 두 개는 당시 교육장 지도 기준이라 **새 맵에서는 다시 정해야 한다.**
  - `mission_trigger_node.search_pose: [x, y, yaw]` — 웹캠이 보는 구역의 중심. RViz에서 찍어 정한다.
  - `search_and_track_server.initial_pose: [x, y, yaw]` — 언도킹 직후 로봇 위치. 언도킹 후 아래로 확인한다.
    ```bash
    ros2 topic echo /robot2/amcl_pose --once
    ```

## 3. 실행 순서

### ① 로봇 언도킹 (아무 PC)

도킹 중에는 RPLIDAR가 멈춰 있어 `/scan`이 없고, AMCL이 위치를 잡지 못한다. 먼저 언도킹한다.

```bash
ros2 action send_goal /robot2/undock irobot_create_msgs/action/Undock "{}"
```

액션 서버도 goal을 받았을 때 로봇이 도킹 상태면 스스로 언도킹한다. 그래도 먼저 언도킹해 두면 RViz에서 위치 추정을 확인한 뒤 시작할 수 있다.

### ② Nav2 + localization (GPU 노트북)

```bash
ros2 launch rc_tracker nav.launch.py namespace:=/robot2 map:=/path/to/your_map.yaml
```

- turtlebot4_navigation의 `localization.launch.py`와 `nav2.launch.py`를 한 번에 띄운다.
- costmap 파라미터는 `config/nav2.yaml`(공식 설정에서 `inflation_radius`만 0.45 → 0.25)을 쓴다.

초기 위치 확인 (선택)

```bash
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/robot2
```

RViz의 **2D Pose Estimate**로 현재 위치를 지정하거나 확인한다.
`set_initial_pose: true`이면 서버가 goal마다 `initial_pose`를 다시 넣는다.

```bash
ros2 lifecycle get /robot2/bt_navigator     # active 여야 함
```

### ③ 액션 서버 (GPU 노트북)

```bash
ros2 launch rc_tracker server.launch.py namespace:=/robot2
```

확인 (다른 터미널)

```bash
ros2 action list | grep search_and_track                        # /search_and_track
ros2 topic hz /robot2/oakd/rgb/image_raw/compressed             # RGB 수신
ros2 topic hz /robot2/oakd/stereo/image_raw/compressedDepth     # depth 수신
```

서버 로그 첫 부분에 실제 구독 토픽(`rgb / depth / info`)과 `base` 프레임이 찍힌다.
`Time Sync OK #...` 로그가 나오면 RGB-depth 짝이 맞고 있는 것이다.

### ④ 웹캠 감지 + 미션 트리거 (웹캠 PC)

```bash
ros2 launch rc_tracker station.launch.py
```

`webcam_detector`와 `mission_trigger`가 같이 뜬다. 웹캠 앞에 RC카를 두면 `RC`가 N프레임(기본 5) 연속으로 잡힐 때 트리거가 1회 나가고, `mission_trigger`가 액션 goal을 보낸다.

```bash
ros2 topic echo /webcam/rc_car_detected      # data: true 1회
```

## 4. 모니터링 (아무 PC)

```bash
# 로봇 카메라 디버그 영상 (bbox + id·conf, 고정된 target은 빨간 박스)
ros2 run rqt_image_view rqt_image_view /robot2/search_and_track/debug_image

# 웹캠 디버그 영상
ros2 run rqt_image_view rqt_image_view /webcam/image_raw/compressed

# 액션 feedback (phase, 거리, 탐색 진행률)
ros2 topic echo /search_and_track/_action/feedback

# target의 map 좌표
ros2 topic echo /robot2/search_and_track/target_point
```

미션 단계와 결과는 `mission_trigger` 콘솔에도 찍힌다.

## 5. 웹캠 없이 시험

goal 직접 전송

```bash
ros2 action send_goal /search_and_track rc_tracker_interfaces/action/SearchAndTrack \
"{search_pose: {header: {frame_id: map}, pose: {position: {x: -2.0, y: -0.6, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}, stop_distance: 0.3, spin_angle: 6.2832, goal_update_distance: 0.25, target_lost_timeout: 5.0, return_to_dock_on_finish: false}" \
--feedback
```

(`search_pose` 값은 당시 지도 기준이다. 자기 맵 좌표로 바꾼다. `Ctrl+C`로 취소 요청)

트리거만 흉내 내기 (`mission_trigger` → 서버 경로 시험)

```bash
ros2 topic pub --once /webcam/rc_car_detected std_msgs/msg/Bool "{data: true}" \
  --qos-durability transient_local --qos-reliability reliable
```

## 6. 탐색 변형 v1~v4 시험

[`search_variants.md`](search_variants.md) 5절 참고. 변형 폴더의 `search_and_track_server.py`와 `mission_params.yaml`을 워크스페이스에 덮어쓰고 다시 빌드한다.

## 7. 종료

각 launch 터미널에서 `Ctrl+C`. 필요하면 수동 도킹:

```bash
ros2 action send_goal /robot2/dock irobot_create_msgs/action/Dock "{}"
```

## 8. 문제 해결

| 증상 | 확인 |
|---|---|
| `mission_trigger`가 goal을 안 보냄 | `/webcam/rc_car_detected`에 `true`가 오는지. 안 오면 웹캠·조명·`conf_threshold`·`consecutive_frames` |
| 서버가 goal을 거절 | 이미 미션이 실행 중이면 거절된다(한 번에 1개) |
| `waiting for Nav2 to become active ...`에서 멈춤 | Nav2/localization이 안 떴거나 AMCL 초기 위치가 없음 |
| 탐색 위치로 경로를 못 만듦 | 좁은 구역이면 costmap inflation 확인 (`nav.launch.py`로 `config/nav2.yaml`을 쓰는지) |
| SEARCHING에서 lock이 안 걸림 | `debug_image`에서 회전 중 박스가 보이는지 확인. 이 프로젝트가 멈춘 지점이다 → [`search_variants.md`](search_variants.md) |
| APPROACHING인데 `target_distance: -1` | depth 수신·디코드 실패, 또는 `camera_frame`이 TF에 없음 |
| 엉뚱한 곳으로 감 | TF 체인(camera → map), `robot_base_frame`, `initial_pose` yaw |
