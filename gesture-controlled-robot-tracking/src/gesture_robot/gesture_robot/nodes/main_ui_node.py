"""Tkinter MainUI를 실제 ROS2 토픽에 연결하는 노드.

MainUI 자체는 rclpy와 독립적인 표시 전용 클래스이므로(architecture.md
`ui` 책임 참고), 이 노드가 토픽 구독과 MainUI의 update_* 호출 사이의
변환만 담당한다.
"""

from collections import deque

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from gesture_robot_interfaces.msg import GestureCommand, TrackedObject
from gesture_robot.core.tracking_state import TrackingStateMachine
from gesture_robot.ui.main_ui import MainUI


class MainUINode(Node):
    """카메라·제스처·추적·속도 토픽을 구독해 MainUI 화면을 갱신한다."""

    _FPS_WINDOW = 30

    def __init__(self, ui: MainUI) -> None:
        super().__init__("main_ui")
        self._ui = ui
        self._bridge = CvBridge()
        self._state_machine = TrackingStateMachine()
        self._frame_times: deque[float] = deque(maxlen=self._FPS_WINDOW)

        # 최근 수신값을 모아뒀다가 STATUS 패널 한 번에 갱신한다.
        self._gesture_command = 0
        self._gesture_confidence = 0.0
        self._tracking_error_x = 0.0
        self._tracking_error_y = 0.0
        self._tracking_area = 0.0

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, "camera/image_annotated", self._on_camera_frame, best_effort_qos
        )
        self.create_subscription(
            GestureCommand, "gesture/command", self._on_gesture_command, reliable_qos
        )
        self.create_subscription(
            TrackedObject, "tracking/object", self._on_tracking_result, best_effort_qos
        )
        self.create_subscription(
            Twist, "turtle1/cmd_vel", self._on_velocity_command, reliable_qos
        )

        self.get_logger().info("main_ui 노드가 시작되었습니다.")

    def _on_camera_frame(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._ui.update_camera_frame(frame)
        self._ui.update_topic_activity("camera/image_raw")
        self._record_fps()
        self._publish_status()

    def _on_gesture_command(self, msg: GestureCommand) -> None:
        self._gesture_command = msg.command
        self._gesture_confidence = msg.confidence
        self._state_machine.update(msg.command)
        self._ui.update_topic_activity("gesture/command")

    def _on_tracking_result(self, msg: TrackedObject) -> None:
        self._tracking_error_x = msg.error_x if msg.detected else 0.0
        self._tracking_error_y = msg.error_y if msg.detected else 0.0
        self._tracking_area = msg.area if msg.detected else 0.0
        self._ui.update_topic_activity("tracking/object")

    def _on_velocity_command(self, msg: Twist) -> None:
        self._ui.update_velocity(msg.linear.x, msg.angular.z)
        self._ui.update_topic_activity("turtle1/cmd_vel")

    def _record_fps(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        self._frame_times.append(now)

    def _current_fps(self) -> float:
        if len(self._frame_times) < 2:
            return 0.0
        elapsed = self._frame_times[-1] - self._frame_times[0]
        if elapsed <= 0:
            return 0.0
        return (len(self._frame_times) - 1) / elapsed

    def _publish_status(self) -> None:
        self._ui.update_status(
            tracking=self._state_machine.current_state(),
            gesture=self._gesture_command,
            confidence=self._gesture_confidence,
            error_x=self._tracking_error_x,
            error_y=self._tracking_error_y,
            area=self._tracking_area,
            fps=self._current_fps(),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    ui = MainUI(use_mock=False)
    node = MainUINode(ui)

    def spin_once() -> None:
        if not rclpy.ok():
            return
        rclpy.spin_once(node, timeout_sec=0)
        ui.root.after(10, spin_once)

    ui.root.after(10, spin_once)
    try:
        ui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
