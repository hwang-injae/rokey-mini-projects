#!/usr/bin/env python3
"""Webcam RC-car detector  (runs on the webcam PC, via station.launch.py).

USB webcam (DRGO-WC720) -> YOLO11n. The webcam watches the area the docked AMR
cannot see. When `target_class` (RC) is detected with conf >= threshold for
`consecutive_frames` consecutive frames, we latch-publish std_msgs/Bool(true)
ONCE on /webcam/rc_car_detected -- that is the undock trigger for the AMR mission.

The webcam side does NOT compute any RC-car coordinate; the AMR navigates to a
pre-known search pose (mission_trigger_node param).
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from ultralytics import YOLO

from rc_tracker.model_paths import resolve_model_path


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class WebcamDetector(Node):

    def __init__(self):
        super().__init__('webcam_detector_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('target_class', 'car')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('consecutive_frames', 5)
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('trigger_topic', '/webcam/rc_car_detected')
        self.declare_parameter('debug_topic', '/webcam/image_raw/compressed')

        p = self.get_parameter
        self.target_class = p('target_class').value
        self.conf_threshold = float(p('conf_threshold').value)
        self.consecutive_frames = int(p('consecutive_frames').value)
        self.publish_debug = bool(p('publish_debug_image').value)
        cam_index = int(p('camera_index').value)
        frame_rate = float(p('frame_rate').value)

        self.model = YOLO(resolve_model_path(p('model_path').value))
        self.target_ids = [i for i, n in self.model.names.items()
                           if n == self.target_class]
        if not self.target_ids:
            self.get_logger().warn(
                f"class '{self.target_class}' not in model names -> accepting all classes")

        self.trigger_pub = self.create_publisher(
            Bool, p('trigger_topic').value, latched_qos())
        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                CompressedImage, p('debug_topic').value, 1)

        self.hit_streak = 0
        self.triggered = False

        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError(f'cannot open webcam index {cam_index}')

        self.timer = self.create_timer(1.0 / max(frame_rate, 1.0), self.tick)
        self.get_logger().info(
            f'webcam_detector_node started (cam={cam_index}, class={self.target_class}, '
            f'need {self.consecutive_frames} consecutive frames)')

    def tick(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('frame grab failed', throttle_duration_sec=2.0)
            return

        result = self.model.predict(frame, conf=self.conf_threshold, verbose=False)[0]

        seen = False
        for box in result.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if self.target_ids and cls not in self.target_ids:
                continue
            if conf < self.conf_threshold:
                continue
            seen = True
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{self.model.names[cls]} {conf:.2f}',
                        (x1, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1)

        self.hit_streak = self.hit_streak + 1 if seen else 0

        if not self.triggered and self.hit_streak >= self.consecutive_frames:
            self.trigger_pub.publish(Bool(data=True))
            self.triggered = True
            self.get_logger().info(
                f'RC car confirmed ({self.hit_streak} frames) -> '
                f'published latched trigger')

        if self.debug_pub is not None:
            status = ('TRIGGERED' if self.triggered
                      else f'streak {self.hit_streak}/{self.consecutive_frames}')
            cv2.putText(frame, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255) if self.triggered else (255, 255, 255), 2)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
            self.debug_pub.publish(msg)

    def destroy_node(self):
        if getattr(self, 'cap', None) is not None:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = WebcamDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
