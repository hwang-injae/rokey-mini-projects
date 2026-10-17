"""손 제스처 인식 ROS2 노드."""

import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gesture_robot_interfaces.msg import GestureCommand as GestureCommandMsg
from gesture_robot_interfaces.msg import TrackedObject
from std_msgs.msg import Header

from gesture_robot.core.models import GestureCommand, GestureResult
from gesture_robot.adapters.mediapipe_detector import MediaPipeDetector
from gesture_robot.core.gesture_classifier import GestureClassifier

class GestureNode(Node):
    """손 랜드마크를 제스처 명령으로 변환해 발행한다."""
    def __init__(self):
        super().__init__("gesture_node")

        self.declare_parameter("max_num_hands", 1)
        self.declare_parameter("min_detection_confidence", 0.7)
        self.declare_parameter("min_tracking_confidence", 0.5)
        # 손 바운딩 박스 면적(px^2)을 controller_node의 area(공 면적 기준으로
        # 튜닝된 target_area=2000 등)와 같은 스케일로 맞추기 위한 나눔값.
        # 손이 공보다 훨씬 크게 보이므로 나눠서 비슷한 범위로 축소한다.
        self.declare_parameter("hand_area_scale", 10.0)

        self._detector = MediaPipeDetector(
            max_num_hands=self.get_parameter("max_num_hands").value,
            min_detection_confidence=self.get_parameter("min_detection_confidence").value,
            min_tracking_confidence=self.get_parameter("min_tracking_confidence").value,
        )
        self._classifier = GestureClassifier()
        self._bridge = CvBridge()

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        best_effort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._publisher = self.create_publisher(
            GestureCommandMsg, "gesture/command", reliable_qos
        )
        # object_tracking_node도 같은 토픽에 공 바운딩 박스를 그려 발행하므로
        # (각자 독립적으로 annotate), main_ui는 두 노드 중 먼저 도착한 프레임을
        # 그대로 표시한다.
        self._annotated_publisher = self.create_publisher(
            Image, "camera/image_annotated", best_effort_qos
        )
        # 손 위치를 추적 대상 좌표로 사용한다 (object_tracking_node의 HSV 추적과
        # 같은 토픽/메시지를 공유하는 대체 입력 - 데모/테스트 목적, 두 노드를
        # 동시에 띄우면 서로 다른 소스가 같은 토픽에 번갈아 발행되어 흔들릴 수
        # 있으므로 함께 실행하지 않는 것을 권장한다).
        self._tracking_publisher = self.create_publisher(
            TrackedObject, "tracking/object", best_effort_qos
        )
        self._subscriber = self.create_subscription(
            Image, "camera/image_raw", self._image_callback, best_effort_qos
        )

    def _image_callback(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        landmarks = self._detector.detect(frame)
        if not isinstance(landmarks, GestureResult):
            mp.solutions.drawing_utils.draw_landmarks(
                frame, landmarks, mp.solutions.hands.HAND_CONNECTIONS
            )
        result = self._classifier.classify(landmarks)
        self.publish_gesture_command(result, msg.header)
        self._publish_hand_position(landmarks, result, frame.shape, msg.header)
        self._publish_annotated_frame(frame, msg.header)

    def _publish_hand_position(self, landmarks, result, frame_shape, header) -> None:
        """손 랜드마크 중심 좌표를 TrackedObject로 변환해 발행한다.

        손을 편 상태(START)일 때만 추적 대상이 '보이는' 것으로 간주한다.
        주먹(STOP)이나 손 미검출(NONE)일 때는 detected=False로 발행해
        추적 대상이 사라진 것처럼 동작하게 한다.
        """
        msg = TrackedObject()
        msg.header = header

        is_open_hand = (
            not isinstance(landmarks, GestureResult)
            and result.command == GestureCommand.START
        )

        if not is_open_hand:
            msg.detected = False
            msg.error_x = 0.0
            msg.error_y = 0.0
            msg.area = 0.0
        else:
            height, width = frame_shape[0], frame_shape[1]
            xs = [lm.x * width for lm in landmarks.landmark]
            ys = [lm.y * height for lm in landmarks.landmark]
            center_x = sum(xs) / len(xs)
            center_y = sum(ys) / len(ys)

            area_scale = float(self.get_parameter("hand_area_scale").value)
            raw_area = (max(xs) - min(xs)) * (max(ys) - min(ys))

            msg.detected = True
            # object_tracking_node와 동일한 부호 규칙 (오른쪽/아래쪽 +).
            msg.error_x = float(center_x - width / 2)
            msg.error_y = float(center_y - height / 2)
            # 손 랜드마크 바운딩 박스 면적을 '거리'의 근사치로 사용하되,
            # 공 기준으로 튜닝된 target_area와 비슷한 범위가 되도록 축소한다.
            msg.area = float(raw_area / area_scale)

        self._tracking_publisher.publish(msg)

    def _publish_annotated_frame(self, frame, header) -> None:
        # mediapipe drawing_utils가 배열을 non-contiguous하게 만들 때가 있어
        # cv_bridge 변환 전에 정규화한다.
        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        annotated_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated_msg.header = header
        self._annotated_publisher.publish(annotated_msg)

    def publish_gesture_command(self, result, header=None):
        msg = GestureCommandMsg()
        if header is not None:
            msg.header = header
        else:
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = result.command.value
        msg.confidence = result.confidence
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
