#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import (
    Detection2D, Detection2DArray,
    ObjectHypothesisWithPose, BoundingBox2D,
)
from cv_bridge import CvBridge

from ultralytics import YOLO


class YoloTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker')

        # ---------------- parameters ----------------
        self.declare_parameter('model_path', 'best.pt')
        # 터틀봇4 OAK-D 기본 RGB 프리뷰 (네임스페이스 있으면 앞에 /<robot> 붙이기)
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('image_transport', 'raw')   # 'raw' | 'compressed'
        self.declare_parameter('detections_topic', '~/detections')
        self.declare_parameter('annotated_topic', '~/annotated')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('tracker', 'bytetrack.yaml')   # or 'botsort.yaml'
        self.declare_parameter('device', 'cpu')               # 'cpu' | '0'
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('classes', '')                 # "0,1" 필터, ''=전체
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('inference_rate', 15.0)        # Hz

        gp = self.get_parameter
        model_path             = gp('model_path').value
        self.image_topic       = gp('image_topic').value
        transport              = str(gp('image_transport').value)
        self.conf              = gp('conf').value
        self.iou               = gp('iou').value
        self.tracker           = gp('tracker').value
        self.device            = gp('device').value
        self.imgsz             = int(gp('imgsz').value)
        cls_str                = str(gp('classes').value).strip()
        self.classes           = [int(x) for x in cls_str.split(',')] if cls_str else None
        self.publish_annotated = gp('publish_annotated').value
        rate                   = float(gp('inference_rate').value)

        # ---------------- model ----------------
        self.get_logger().info(f'Loading YOLO: {model_path} (device={self.device})')
        self.model = YOLO(model_path)
        self.names = self.model.names            # {int: str}

        # ---------------- ROS I/O ----------------
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest = None                      # (header, bgr_image)

        if transport == 'compressed':
            self.create_subscription(
                CompressedImage, self.image_topic + '/compressed',
                self._on_compressed, qos_profile_sensor_data)
        else:
            self.create_subscription(
                Image, self.image_topic, self._on_image, qos_profile_sensor_data)

        self.pub_det = self.create_publisher(
            Detection2DArray, gp('detections_topic').value, 10)
        if self.publish_annotated:
            self.pub_img = self.create_publisher(
                Image, gp('annotated_topic').value, 10)

        self.create_timer(1.0 / max(rate, 1e-3), self._on_timer)
        self.get_logger().info(
            f'sub={self.image_topic} ({transport})  tracker={self.tracker}  rate={rate}Hz')

    # --- 최신 프레임만 보관 (추론이 카메라보다 느려도 지연 누적 방지) ---
    def _on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return
        with self._lock:
            self._latest = (msg.header, bgr)

    def _on_compressed(self, msg: CompressedImage):
        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return
        with self._lock:
            self._latest = (msg.header, bgr)

    def _on_timer(self):
        with self._lock:
            item, self._latest = self._latest, None
        if item is None:
            return
        header, bgr = item

        # persist=True 로 프레임 간 트랙 ID 유지
        results = self.model.track(
            source=bgr, persist=True, tracker=self.tracker,
            conf=self.conf, iou=self.iou, imgsz=self.imgsz,
            device=self.device, classes=self.classes, verbose=False,
        )
        r = results[0]

        out = Detection2DArray()
        out.header = header

        if r.boxes is not None and len(r.boxes) > 0:
            xywh  = r.boxes.xywh.cpu().numpy()             # cx, cy, w, h (픽셀)
            confs = r.boxes.conf.cpu().numpy()
            clss  = r.boxes.cls.cpu().numpy().astype(int)
            ids   = (r.boxes.id.cpu().numpy().astype(int)
                     if r.boxes.id is not None else [-1] * len(confs))

            for (cx, cy, w, h), score, c, tid in zip(xywh, confs, clss, ids):
                det = Detection2D()
                det.header = header
                det.id = str(int(tid))                     # 트랙 ID

                bb = BoundingBox2D()
                bb.center.position.x = float(cx)
                bb.center.position.y = float(cy)
                bb.center.theta = 0.0
                bb.size_x = float(w)
                bb.size_y = float(h)
                det.bbox = bb

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(self.names.get(int(c), int(c)))  # 클래스명
                hyp.hypothesis.score = float(score)                            # confidence
                det.results.append(hyp)

                out.detections.append(det)

        self.pub_det.publish(out)

        if self.publish_annotated:
            annotated = r.plot()                            # 박스+ID 그려진 BGR
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            img_msg.header = header
            self.pub_img.publish(img_msg)


def main():
    rclpy.init()
    node = YoloTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
