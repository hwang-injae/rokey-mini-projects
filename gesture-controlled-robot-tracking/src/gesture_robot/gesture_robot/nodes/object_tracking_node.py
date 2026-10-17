"""객체 추적 ROS2 노드 모듈.

이 노드는 카메라 영상 프레임(/camera/image_raw)을 전달받거나 카메라 장치에서 프레임을 읽어
ColorTracker 핵심 알고리즘으로 빨간 공의 위치 오차(error_x, error_y)와 면적(area)을 계산한 뒤,
TrackedObject 커스텀 메시지 포맷으로 /tracking/object 토픽에 발행합니다.
"""

from typing import Optional, Any
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# OpenCV 및 ROS2 이미지 변환 bridge (필요 시 선택적 사용)
import cv2
from cv_bridge import CvBridge

# 커스텀 메시지 및 센서 메시지 불러오기
from sensor_msgs.msg import Image
from gesture_robot_interfaces.msg import TrackedObject

# 내부 코어 모듈 불러오기
from gesture_robot.core.color_tracker import ColorTracker
from gesture_robot.core.models import TrackingResult
from gesture_robot.visualization.tracking_visualizer import TrackingVisualizer


class ObjectTrackingNode(Node):
    """카메라 입력 수신, 객체 추적 알고리즘 수행 및 추적 결과 토픽 발행 노드."""

    def __init__(self, camera: Optional[Any] = None, visualizer: Optional[Any] = None) -> None:
        """노드 초기화: ROS2 파라미터, QoS, Publisher, Subscriber 및 ColorTracker 구성."""
        super().__init__("object_tracking_node")

        self.camera = camera
        self.visualizer = visualizer
        self.bridge = CvBridge()

        # 1. ROS2 파라미터 선언 및 기본값 설정 (SOT 파라미터 명세 준수)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("min_area_threshold", 500.0),
                ("hue_min1", 0),
                ("hue_max1", 10),
                ("hue_min2", 170),
                ("hue_max2", 179),
                ("sat_min", 100),
                ("sat_max", 255),
                ("val_min", 100),
                ("val_max", 255),
                ("kernel_size", 5),
                ("publish_rate", 30.0),
            ],
        )

        # 2. ColorTracker 코어 객체 생성
        self.tracker = ColorTracker(
            min_area=self.get_parameter("min_area_threshold").value,
            hue_min1=self.get_parameter("hue_min1").value,
            hue_max1=self.get_parameter("hue_max1").value,
            hue_min2=self.get_parameter("hue_min2").value,
            hue_max2=self.get_parameter("hue_max2").value,
            sat_min=self.get_parameter("sat_min").value,
            sat_max=self.get_parameter("sat_max").value,
            val_min=self.get_parameter("val_min").value,
            val_max=self.get_parameter("val_max").value,
            kernel_size=self.get_parameter("kernel_size").value,
        )
        publish_rate = float(self.get_parameter("publish_rate").value)


        # 3. QoS 프로필 설정 (SOT 계약: BEST_EFFORT, KEEP_LAST, Depth 1)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 4. Publisher 생성: tracking/object (TrackedObject 메시지 - 상대 토픽 사용)
        self.publisher_ = self.create_publisher(
            TrackedObject,
            "tracking/object",
            qos_profile,
        )

        # 4-1. 시각화된 프레임 발행: camera/image_annotated
        # gesture_node도 같은 토픽에 손 랜드마크를 그려 발행하므로(각자 독립적으로
        # annotate), main_ui는 두 노드 중 먼저 도착한 프레임을 그대로 표시한다.
        self.annotated_publisher = self.create_publisher(
            Image,
            "camera/image_annotated",
            qos_profile,
        )

        # 5. Subscriber 생성: camera/image_raw (sensor_msgs/Image 토픽 수신 시 process_image_msg 실행 - 상대 토픽 사용)
        self.subscription = self.create_subscription(
            Image,
            "camera/image_raw",
            self.image_callback,
            qos_profile,
        )

        # 6. 카메라 객체가 직접 전달된 경우 타이머 실행 (독립 모드 지원)
        if self.camera is not None:
            timer_period = 1.0 / publish_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("객체 추적 노드(ObjectTrackingNode)가 성공적으로 시작되었습니다.")

    def image_callback(self, msg: Image) -> None:
        """/camera/image_raw 토픽 수신 시 호출되는 콜백 함수."""
        try:
            # ROS Image 메시지를 OpenCV BGR 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_frame(cv_image, header=msg.header)
        except Exception as e:
            self.get_logger().error(f"이미지 변환 및 처리 중 오류 발생: {e}")

    def timer_callback(self) -> None:
        """카메라 어댑터가 직접 연결된 경우 타이머 기반 프레임 처리 콜백."""
        if self.camera is None:
            return
        ret, frame = self.camera.read()
        if ret and frame is not None:
            self.process_frame(frame)

    def process_frame(self, frame: Any, header: Optional[Any] = None) -> None:
        """단일 프레임 이미지에 대해 ColorTracker 알고리즘을 적용하고 결과를 발행합니다."""
        # 코어 추적 알고리즘 실행 (TrackingResult, debug_info 반환)
        tracking_result, debug_info = self.tracker.track(frame)

        # 시각화 어댑터가 있는 경우 디버깅 오버레이 렌더링 후 발행
        if self.visualizer is not None:
            self.visualizer.render(frame, tracking_result, debug_info)
            self._publish_annotated_frame(frame, header)

        # 추적 결과를 rosidl TrackedObject 메시지로 변환 후 토픽 발행
        self.publish_tracking_result(tracking_result, header)

    def _publish_annotated_frame(self, frame: Any, header: Optional[Any] = None) -> None:
        """바운딩 박스가 그려진 프레임을 camera/image_annotated로 발행합니다."""
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
        self.annotated_publisher.publish(msg)

    def publish_tracking_result(self, result: TrackingResult, header: Optional[Any] = None) -> None:
        """TrackingResult 코어 모델을 TrackedObject rosidl 메시지로 변환하여 발행합니다."""
        msg = TrackedObject()

        # Header 타임스탬프 설정
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"

        # 추적 결과 데이터 필드 복사
        msg.detected = result.detected
        msg.error_x = float(result.error_x)
        msg.error_y = float(result.error_y)
        msg.area = float(result.area)

        # 토픽 발행
        self.publisher_.publish(msg)
        self.get_logger().debug(
            f"[추적 결과] 검출:{msg.detected}, error_x:{msg.error_x:.1f}, error_y:{msg.error_y:.1f}, area:{msg.area:.0f}"
        )


def main(args=None) -> None:
    """노드 실행 진입점."""
    rclpy.init(args=args)
    node = ObjectTrackingNode(visualizer=TrackingVisualizer())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

