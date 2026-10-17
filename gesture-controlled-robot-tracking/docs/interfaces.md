# Interfaces

인터페이스 정의는 `gesture_robot_interfaces/msg`가 실제 기준이며, 이 문서는 필드의
의도와 미결정 사항을 설명한다.

rosidl 빌드 후 Python 노드는 다음 경로에서 생성 메시지를 가져온다.

```python
from gesture_robot_interfaces.msg import GestureCommand, TrackedObject
```

## `GestureCommand.msg`

| 필드 | 타입 | 의미 |
|---|---|---|
| `header` | `std_msgs/Header` | 생성 시각과 좌표계 정보 |
| `command` | `uint8` | 인식된 제스처 명령 |
| `confidence` | `float32` | 인식 신뢰도 |

### 명령 상수

| 상수 | 값 | 의미 |
|---|---|---|
| `NONE` | `0` | 손 미검출 또는 명령 없음. 기존 추적 상태 유지 |
| `START` | `1` | 보자기 제스처. 추적 시작 |
| `STOP` | `2` | 주먹 제스처. 추적 정지 |

## `TrackedObject.msg`

| 필드 | 타입 | 단위 | 의미 |
|---|---|---|---|
| `header` | `std_msgs/Header` | - | 생성 시각과 좌표계 정보 |
| `detected` | `bool` | - | 객체 검출 여부 |
| `error_x` | `float32` | 픽셀 | 화면 중심과 공 중심 사이의 가로 오차 |
| `error_y` | `float32` | 픽셀 | 화면 중심과 공 중심 사이의 세로 오차 |
| `area` | `float32` | 픽셀² | 검출된 공의 면적 |

객체가 검출되지 않으면 `detected = false`로 알린다. 제어 노드는 이
값을 받으면 거북이를 정지시키되 제스처로 결정된 추적 상태는 유지한다.

## 좌표와 제어 규칙

- `error_x = object_center_x - image_center_x`로 계산한다.
- `error_y = object_center_y - image_center_y`로 계산한다.
- 화면 오른쪽과 아래쪽을 양의 방향으로 정의한다.
- `error_x`는 좌우 회전에 사용한다.
- `error_y`는 MVP 이동 제어에는 사용하지 않고 시각화와 향후 확장을 위해
  유지한다.

## 속도 명령

- 메시지 타입은 `geometry_msgs/Twist`를 사용한다.
- 회전 속도는 `angular_z = -angular_gain * error_x`로 계산한다.
- 전진 속도는 `linear_x = linear_gain * (target_area - area)`로 계산한다.
- 회전 및 면적 deadband를 적용하고 선속도와 각속도를 설정된 최댓값으로
  제한한다.
- 공의 면적이 작으면 빠르게 전진하고 목표 면적에 가까워질수록 감속한다.
- 목표 면적보다 공이 크게 검출되면 MVP에서는 후진하지 않고 정지한다.
- `STOP`, `detected = false` 또는 추적 결과 timeout 시 속도 0을 발행한다.
- 객체가 다시 검출되면 기존 추적 상태가 START인 경우 이동을 재개한다.

## 토픽 계약

| 토픽 | 송신 | 수신 | 메시지 |
|---|---|---|---|
| `/camera/image_raw` | 카메라 노드 | 제스처·객체 추적 노드 | `sensor_msgs/Image` |
| `/gesture/command` | 제스처 인식 노드 | 제어 노드 | `GestureCommand` |
| `/tracking/object` | 객체 추적 노드 | 제어 노드 | `TrackedObject` |
| `/turtle1/cmd_vel` | 제어 노드 | turtlesim | `geometry_msgs/Twist` |

노드 구현에서는 상대 토픽 이름을 사용하고, 문서에는 루트 namespace 기준의
전체 토픽 이름을 표기한다. MVP에서는 별도 namespace와 remapping을 사용하지 않는다.

확정 노드 이름은 `camera_node`, `gesture_node`, `object_tracking_node`,
`controller_node`, `simulation_node`다. 각 노드의 책임은 `docs/SOT.md`를 따른다.

## QoS 원칙

- 제스처 시작·정지 명령과 turtlesim 속도 명령은 RELIABLE을 사용한다.
- 카메라 영상과 객체 추적 결과는 BEST_EFFORT, KEEP_LAST, Depth 1을 사용한다.

## 미정 인터페이스

- 이동 제어 파라미터의 수치 초깃값
