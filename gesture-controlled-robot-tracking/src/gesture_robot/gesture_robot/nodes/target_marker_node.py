"""손 위치(추적 대상)를 turtlesim 위의 두 번째 turtle로 표시하는 노드.

turtle1은 controller_node가 계산한 속도 명령으로 이 목표를 뒤쫓아 가고,
이 노드가 관리하는 'target' turtle은 tracking/object의 손 위치를 그대로
순간이동(teleport)시켜 '따라가야 할 지점'을 시각적으로 보여준다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from turtlesim.srv import Spawn, SetPen, TeleportAbsolute

from gesture_robot_interfaces.msg import TrackedObject


class TargetMarkerNode(Node):
    """손 위치를 turtlesim의 target turtle 좌표로 변환해 순간이동시킨다."""

    _TARGET_NAME = "target"
    _CENTER = 5.544445  # turtlesim 기본 turtle1 스폰 좌표 (화면 중심)
    _WORLD_MIN, _WORLD_MAX = 0.5, 10.5

    def __init__(self) -> None:
        super().__init__("target_marker_node")

        # 화면 절반 폭(±320px) 전체를 기준으로 잡으면 실제 손 움직임 범위보다
        # 너무 둔감해서, 사람이 편하게 움직이는 범위(기본 150px)를 "꽉 찬 이동"
        # 기준으로 삼아 turtlesim 좌표 범위(중심 대비 ±5)에 맞춘다.
        self.declare_parameter("pixel_range_for_full_scale", 150.0)
        pixel_range = float(self.get_parameter("pixel_range_for_full_scale").value)
        self._scale_x = 5.0 / pixel_range
        self._scale_y = 5.0 / pixel_range

        self._spawn_client = self.create_client(Spawn, "spawn")
        self._teleport_client = None
        self._set_pen_client = None

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            TrackedObject, "tracking/object", self._on_tracking_result, best_effort_qos
        )

        self._spawn_target()

    def _spawn_target(self) -> None:
        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("turtlesim spawn 서비스에 연결할 수 없습니다.")
            return

        request = Spawn.Request()
        request.x = self._CENTER
        request.y = self._CENTER
        request.theta = 0.0
        request.name = self._TARGET_NAME
        future = self._spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        self._teleport_client = self.create_client(
            TeleportAbsolute, f"{self._TARGET_NAME}/teleport_absolute"
        )
        self._set_pen_client = self.create_client(
            SetPen, f"{self._TARGET_NAME}/set_pen"
        )
        self._teleport_client.wait_for_service(timeout_sec=5.0)
        self._set_pen_client.wait_for_service(timeout_sec=5.0)

        # target은 이동 경로를 그리지 않도록 펜을 끈다.
        pen_request = SetPen.Request(off=1)
        self._set_pen_client.call_async(pen_request)

        self.get_logger().info("target turtle 생성 완료")

    def _on_tracking_result(self, msg: TrackedObject) -> None:
        if not msg.detected or self._teleport_client is None:
            return

        x = self._clamp(self._CENTER + msg.error_x * self._scale_x)
        # 이미지 좌표(아래로 +)와 turtlesim 좌표(위로 +)의 방향이 반대라 부호를 뒤집는다.
        y = self._clamp(self._CENTER - msg.error_y * self._scale_y)

        request = TeleportAbsolute.Request(x=x, y=y, theta=0.0)
        self._teleport_client.call_async(request)

    def _clamp(self, value: float) -> float:
        return max(self._WORLD_MIN, min(self._WORLD_MAX, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
