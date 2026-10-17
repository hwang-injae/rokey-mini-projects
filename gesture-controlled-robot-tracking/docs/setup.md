# Setup

## 전제 조건

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12
- `colcon`, `rosdep`

## 의존성 설치

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Python 의존성 설치

MediaPipe 및 OpenCV/NumPy 호환성을 위해 아래 특정 버전 조합을 설치합니다.
(ROS2 Jazzy `cv_bridge`는 NumPy 1.x 기반이며, MediaPipe `mp.solutions` API 지원을 위해 `0.10.14` 버전을 권장합니다.)

```bash
pip install --break-system-packages "numpy==1.26.4" "mediapipe==0.10.14"
```

## 빌드 및 환경 설정

```bash
colcon build --symlink-install
source install/setup.bash
```

## 실행 (시뮬레이션 Bringup)

### turtlesim 기반 통합 실행
```bash
ros2 launch sim_bringup turtlesim_bringup.launch.py
```

### Gazebo 3D 기반 통합 실행
```bash
ros2 launch sim_bringup gazebo_bringup.launch.py
```

## 트러블슈팅 (Troubleshooting)

1. **`AttributeError: module 'mediapipe' has no attribute 'solutions'`**
   - MediaPipe 1.0.0 이상 버전 설치 시 legacy `solutions` API가 미포함되어 발생합니다. `mediapipe==0.10.14`로 버전을 고정해 설치합니다.

2. **`KeyError: 16` 또는 `NumPy 1.x compiled module crash in NumPy 2.x` (`cv_bridge` 에러)**
   - ROS2 Jazzy의 C++ 바인딩 `cv_bridge`는 NumPy 1.x 기반이므로, NumPy 2.x와 함께 사용 시 충돌합니다. `numpy==1.26.4`로 고정합니다.

3. **`The message type 'gesture_robot_interfaces/msg/...' is invalid`**
   - 새 터미널을 열었을 때 ROS2 커스텀 인터페이스 환경이 로드되지 않아 발생합니다. `source install/setup.bash`를 실행해 주어야 합니다.

