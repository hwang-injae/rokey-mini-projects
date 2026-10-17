"""단일 카메라 프레임 발행 ROS2 노드."""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from gesture_robot.adapters.opencv_camera import OpenCVCamera


class CameraNode(Node):
    """카메라 프레임을 camera/image_raw 토픽으로 발행한다."""

    def __init__(self) -> None:
        super().__init__("camera_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("device_id", 0),
                ("publish_rate", 30.0),
                ("mirror", True),
            ],
        )

        device_id = int(self.get_parameter("device_id").value)
        self._camera = OpenCVCamera(device_id=device_id)
        self._bridge = CvBridge()

        if not self._camera.open():
            self.get_logger().error(
                f"카메라 장치(device_id={device_id})를 열 수 없습니다."
            )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = self.create_publisher(Image, "camera/image_raw", qos_profile)

        publish_rate = float(self.get_parameter("publish_rate").value)
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info("camera_node 시작됨")

    def _on_timer(self) -> None:
        """주기적으로 카메라 프레임을 읽어 발행한다."""
        frame = self._camera.read()
        if frame is None:
            return
        if self.get_parameter("mirror").value:
            # 셀카 화면처럼 좌우 반전한다. 이후 모든 노드(제스처/추적/표시)가
            # 이 반전된 프레임을 기준으로 동작하므로 별도 보정이 필요 없다.
            frame = cv2.flip(frame, 1)
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self._publisher.publish(msg)

    def destroy_node(self) -> bool:
        """노드 종료 시 카메라 자원을 해제한다."""
        self._camera.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
