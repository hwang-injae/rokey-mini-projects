#!/usr/bin/env python3
"""SearchAndTrack action server  (runs on the GPU laptop, next to Nav2).

Flow:

  goal -> UNDOCKING        navigator.undock()  (lidar is off while docked)
       -> setInitialPose(undocked pose) + waitUntilNav2Active
       -> NAV_TO_SEARCH    goToPose(search_pose)              [/navigate_to_pose]
       -> SEARCHING        spin(360) + YOLO track,            [/spin]
                           lock one `target_class` (RC) track_id, see _update_lock()
       -> APPROACHING /     loop @follow_period_s:
          FOLLOWING           RGB bbox centre -> depth -> camera 3D -> TF -> map
                              if target moved >= goal_update_distance -> goToPose(new)
                              if depth distance < stop_distance -> HOLDING
       -> HOLDING          cancel nav, hold; if target moves away -> back to APPROACHING
       -> (any of the above) target unseen >= target_lost_timeout -> re-SEARCH -> LOST

Depth->3D->map (TF) and undock/initial-pose handling follow the patterns of
the course day3 examples.

Camera topics verified on robot2 (704x704, pixel aligned):
  <ns>/oakd/rgb/image_raw/compressed            (jpeg)
  <ns>/oakd/stereo/image_raw/compressedDepth    (16UC1 PNG transport)
  <ns>/oakd/rgb/camera_info

RGB and depth are paired with ApproximateTimeSynchronizer before YOLO/depth
processing. The decoded 16UC1 depth image is republished for inspection.
"""
import math
import threading
import time

import cv2
import numpy as np
import rclpy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform)
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from ultralytics import YOLO

from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from rc_tracker_interfaces.action import SearchAndTrack
from rc_tracker.detection import (median_depth_m, pixel_to_camera_point,
                                  quaternion_to_yaw, yaw_between,
                                  yaw_to_quaternion)
from rc_tracker.model_paths import resolve_model_path

FB = SearchAndTrack.Feedback
RES = SearchAndTrack.Result


class Track:
    __slots__ = ('track_id', 'conf', 'bbox', 'center', 'distance', 'map_xy', 'stamp')

    def __init__(self, track_id, conf, bbox, center, distance, map_xy, stamp):
        self.track_id = track_id
        self.conf = conf
        self.bbox = bbox            # (x1, y1, x2, y2)
        self.center = center        # (u, v)
        self.distance = distance    # [m], -1 if unknown
        self.map_xy = map_xy        # (x, y) in map, or None
        self.stamp = stamp          # time.monotonic()


class SearchAndTrackServer(Node):

    def __init__(self):
        super().__init__('search_and_track_server')

        # ---------------- parameters ----------------
        self.declare_parameter('robot_namespace', '/robot2')
        self.declare_parameter('rgb_topic', '')
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('info_topic', '')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', '')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('tracker_cfg', 'bytetrack.yaml')
        self.declare_parameter('target_class', 'car')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('lock_consecutive_frames', 3)

        self.declare_parameter('depth_min_m', 0.2)
        self.declare_parameter('depth_max_m', 5.0)
        self.declare_parameter('stop_distance_m', 0.80)
        self.declare_parameter('stop_hysteresis_m', 0.10)
        self.declare_parameter('goal_update_distance_m', 0.25)
        self.declare_parameter('follow_period_s', 0.5)
        self.declare_parameter('spin_angle_rad', 2.0 * math.pi)
        self.declare_parameter('target_lost_timeout_s', 5.0)
        self.declare_parameter('hold_finish_s', 5.0)
        self.declare_parameter('perceive_rate_hz', 10.0)

        self.declare_parameter('set_initial_pose', False)
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])   # x, y, yaw
        self.declare_parameter('relock_on_reacquire', True)
        # lock a target as soon as an RC is seen, without waiting for SEARCHING.
        # Handy for standalone perception testing (no robot / no mission goal);
        # a real mission goal resets the lock and re-locks during its spin anyway.
        self.declare_parameter('lock_on_start', True)

        p = self.get_parameter
        ns = p('robot_namespace').value or ''
        ns_clean = ns.strip('/')
        self.ns_prefix = f'/{ns_clean}' if ns_clean else ''

        self.rgb_topic = p('rgb_topic').value or \
            f'{self.ns_prefix}/oakd/rgb/image_raw/compressed'
        self.depth_topic = p('depth_topic').value or \
            f'{self.ns_prefix}/oakd/stereo/image_raw/compressedDepth'
        self.info_topic = p('info_topic').value or \
            f'{self.ns_prefix}/oakd/rgb/camera_info'
        self.camera_frame_override = str(p('camera_frame').value or '')
        self.map_frame = p('map_frame').value
        self.base_frame = p('robot_base_frame').value or \
            (f'{ns_clean}/base_link' if ns_clean else 'base_link')

        self.tracker_cfg = p('tracker_cfg').value
        self.target_class = p('target_class').value
        self.conf_threshold = float(p('conf_threshold').value)
        self.lock_consecutive_frames = int(p('lock_consecutive_frames').value)
        self.depth_min_m = float(p('depth_min_m').value)
        self.depth_max_m = float(p('depth_max_m').value)
        self.stop_distance_m = float(p('stop_distance_m').value)
        self.stop_hysteresis_m = float(p('stop_hysteresis_m').value)
        self.goal_update_distance_m = float(p('goal_update_distance_m').value)
        self.follow_period_s = float(p('follow_period_s').value)
        self.spin_angle_rad = float(p('spin_angle_rad').value)
        self.target_lost_timeout_s = float(p('target_lost_timeout_s').value)
        self.hold_finish_s = float(p('hold_finish_s').value)
        self.set_initial_pose = bool(p('set_initial_pose').value)
        self.initial_pose = [float(v) for v in p('initial_pose').value]
        self.relock_on_reacquire = bool(p('relock_on_reacquire').value)

        # ---------------- perception state (lock-protected) ----------------
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.K = None
        # (bgr, depth_mm, depth_stamp, RGB optical frame)
        self.latest_pair = None
        self.sync_count = 0
        self.tracks = {}             # track_id -> Track
        self.target_id = None
        self.locking_enabled = bool(p('lock_on_start').value)
        self._streak_id = None
        self._streak_n = 0
        self.last_target_seen = 0.0
        self.phase = FB.PHASE_IDLE
        self.search_progress = 0.0
        self._nav_remaining = 0.0
        self._last_distance = -1.0
        self._perceive_busy = False

        self._goal_active = False    # only one mission at a time
        self._goal_lock = threading.Lock()

        # ---------------- YOLO ----------------
        self.model = YOLO(resolve_model_path(p('model_path').value))
        self.target_class_ids = [i for i, n in self.model.names.items()
                                 if n == self.target_class]
        if not self.target_class_ids:
            raise RuntimeError(
                f"class '{self.target_class}' is not included in the model")

        # ---------------- Nav2 / TurtleBot4 ----------------
        self.navigator = TurtleBot4Navigator(namespace=ns_clean)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- callback groups ----------------
        cb_subs = ReentrantCallbackGroup()
        cb_perceive = MutuallyExclusiveCallbackGroup()   # YOLO tracker is stateful
        cb_action = ReentrantCallbackGroup()

        self.create_subscription(
            CameraInfo, self.info_topic, self.on_info,
            qos_profile_sensor_data, callback_group=cb_subs)

        # Both inputs are compressed sensor messages on robot2. Synchronise them
        # before decoding and before any YOLO/depth coordinate calculation.
        self.rgb_sub = Subscriber(
            self, CompressedImage, self.rgb_topic,
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(
            self, CompressedImage, self.depth_topic,
            qos_profile=qos_profile_sensor_data)
        self.image_sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=20,
            slop=0.1,
            allow_headerless=False)
        self.image_sync.registerCallback(self.on_synced_images)

        self.debug_pub = self.create_publisher(
            Image, f'{self.ns_prefix}/search_and_track/debug_image', 1)
        self.target_pub = self.create_publisher(
            PointStamped, f'{self.ns_prefix}/search_and_track/target_point', 1)
        self.depth_pub = self.create_publisher(
            Image, f'{self.ns_prefix}/search_and_track/depth_image',
            qos_profile_sensor_data)

        self.create_timer(1.0 / max(float(p('perceive_rate_hz').value), 1.0),
                          self.perceive, callback_group=cb_perceive)

        self._action = ActionServer(
            self, SearchAndTrack, 'search_and_track',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _req: CancelResponse.ACCEPT,
            callback_group=cb_action,
        )

        self.get_logger().info(
            f'search_and_track_server ready\n'
            f'  rgb   : {self.rgb_topic}\n'
            f'  depth : {self.depth_topic}\n'
            f'  info  : {self.info_topic}\n'
            f'  base  : {self.base_frame}')

    # ================================================================
    # Sensor callbacks
    # ================================================================
    def on_info(self, msg: CameraInfo):
        with self.lock:
            if self.K is None:
                self.get_logger().info(
                    f'camera_info: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                    f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f} size={msg.width}x{msg.height}')
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _decode_compressed_depth(msg: CompressedImage):
        """Decode a ROS compressedDepth 16UC1 PNG payload to millimetres."""
        payload = bytes(msg.data)
        png_signature = b'\x89PNG\r\n\x1a\n'
        png_offset = payload.find(png_signature)
        if png_offset < 0:
            raise ValueError('compressedDepth PNG payload not found')

        encoded = np.frombuffer(payload[png_offset:], dtype=np.uint8)
        depth = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        if depth is None:
            raise ValueError('compressedDepth decode failed')
        if depth.dtype != np.uint16 or depth.ndim != 2:
            raise ValueError(
                f'expected 16UC1 depth, got dtype={depth.dtype}, '
                f'shape={depth.shape}')
        return depth

    def on_synced_images(self, rgb_msg: CompressedImage,
                         depth_msg: CompressedImage):
        """Decode and store one timestamp-matched RGB/depth pair."""
        try:
            bgr = cv2.imdecode(
                np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError('compressed RGB decode failed')

            depth_mm = self._decode_compressed_depth(depth_msg)
            if bgr.shape[:2] != depth_mm.shape[:2]:
                raise ValueError(
                    f'RGB/depth size mismatch: '
                    f'RGB={bgr.shape[1]}x{bgr.shape[0]}, '
                    f'Depth={depth_mm.shape[1]}x{depth_mm.shape[0]}')

            rgb_ns = self._stamp_to_ns(rgb_msg.header.stamp)
            depth_ns = self._stamp_to_ns(depth_msg.header.stamp)
            delta_ms = abs(rgb_ns - depth_ns) / 1_000_000.0

            # CameraInfo is RGB-based, so coordinates must carry the RGB
            # optical frame unless the launch file explicitly overrides it.
            camera_frame = (
                self.camera_frame_override
                or rgb_msg.header.frame_id
                or depth_msg.header.frame_id
                or 'oakd_rgb_camera_optical_frame')

            with self.lock:
                self.latest_pair = (
                    bgr, depth_mm, depth_msg.header.stamp, camera_frame)
                self.sync_count += 1
                count = self.sync_count

            if count == 1 or count % 30 == 0:
                self.get_logger().info(
                    f'Time Sync OK #{count}: '
                    f'{bgr.shape[1]}x{bgr.shape[0]}, '
                    f'RGB-Depth delta={delta_ms:.2f} ms')

            if self.depth_pub.get_subscription_count() > 0:
                out = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
                out.header = depth_msg.header
                self.depth_pub.publish(out)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f'synchronized image decode failed: {e}',
                throttle_duration_sec=2.0)

    # ================================================================
    # Perception timer  (single-threaded via MutuallyExclusiveCallbackGroup)
    # ================================================================
    def perceive(self):
        with self.lock:
            if self._perceive_busy:
                return
            self._perceive_busy = True
            pair = self.latest_pair
            K = None if self.K is None else self.K.copy()
            locking = self.locking_enabled
            target_id = self.target_id
        try:
            if pair is None or K is None:
                return

            bgr, depth_mm, depth_stamp, camera_frame = pair

            res = self.model.track(
                bgr, persist=True, classes=self.target_class_ids,
                conf=self.conf_threshold, tracker=self.tracker_cfg, verbose=False)[0]

            tracks = {}
            boxes = getattr(res, 'boxes', None)
            if boxes is not None and boxes.id is not None:
                ids = boxes.id.int().cpu().tolist()
                confs = boxes.conf.cpu().tolist()
                xyxy = boxes.xyxy.cpu().numpy().astype(int)
                for tid, cf, bb in zip(ids, confs, xyxy):
                    x1, y1, x2, y2 = (int(v) for v in bb.tolist())
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    z = median_depth_m(depth_mm, x1, y1, x2, y2,
                                       self.depth_min_m, self.depth_max_m)
                    dist, map_xy = -1.0, None
                    if z is not None:
                        dist = z
                        cam_pt = pixel_to_camera_point(u, v, z, K)
                        map_xy = self._to_map(cam_pt, camera_frame, depth_stamp)
                    tracks[tid] = Track(tid, float(cf), (x1, y1, x2, y2),
                                        (u, v), dist, map_xy, time.monotonic())

            newly_locked = self._update_lock(bgr, tracks, locking, target_id)

            with self.lock:
                self.tracks = tracks
                tid_now = self.target_id
                if tid_now is not None and tid_now in tracks:
                    self.last_target_seen = time.monotonic()

            if newly_locked is not None:
                self.get_logger().info(f'target locked: track_id={newly_locked}')

            self._publish_debug(bgr, tracks, tid_now)
            self._publish_target_point(tracks.get(tid_now) if tid_now is not None else None,
                                       depth_stamp)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'perceive() error: {e}', throttle_duration_sec=2.0)
        finally:
            with self.lock:
                self._perceive_busy = False

    def _update_lock(self, bgr, tracks, locking, target_id):
        """Lock the first `car` (closest to image centre) for N consecutive frames."""
        if not locking or not tracks:
            return None
        if target_id is not None and target_id in tracks:
            return None  # already have it and it is visible

        h, w = bgr.shape[:2]
        cx_img, cy_img = w / 2.0, h / 2.0
        cand = min(tracks.values(),
                   key=lambda t: (t.center[0] - cx_img) ** 2 + (t.center[1] - cy_img) ** 2)

        with self.lock:
            if self._streak_id == cand.track_id:
                self._streak_n += 1
            else:
                self._streak_id = cand.track_id
                self._streak_n = 1
            if self._streak_n >= self.lock_consecutive_frames:
                self.target_id = cand.track_id
                self.last_target_seen = time.monotonic()
                return cand.track_id
        return None

    # ================================================================
    # TF helpers
    # ================================================================
    def _to_map(self, cam_pt, frame_id, stamp):
        ps = PointStamped()
        ps.header.frame_id = frame_id
        ps.point.x, ps.point.y, ps.point.z = cam_pt
        for st in (stamp, Time().to_msg()):
            ps.header.stamp = st
            try:
                out = self.tf_buffer.transform(
                    ps, self.map_frame, timeout=Duration(seconds=0.5))
                return (out.point.x, out.point.y)
            except Exception:  # noqa: BLE001
                continue
        return None

    def _robot_xy_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.5))
            return (t.transform.translation.x, t.transform.translation.y,
                    quaternion_to_yaw(t.transform.rotation))
        except Exception:  # noqa: BLE001
            return None

    # ================================================================
    # Publishers
    # ================================================================
    def _publish_debug(self, bgr, tracks, target_id):
        if self.debug_pub.get_subscription_count() == 0:
            return
        img = bgr.copy()
        for t in tracks.values():
            x1, y1, x2, y2 = t.bbox
            is_t = (t.track_id == target_id)
            color = (0, 0, 255) if is_t else (0, 200, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3 if is_t else 1)
            label = f'id{t.track_id} {t.conf:.2f} {self.target_class}'
            if t.distance > 0:
                label += f' {t.distance:.2f}m'
            cv2.putText(img, label, (x1, max(12, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        with self.lock:
            ph = self.phase
        cv2.putText(img, f'phase={ph} target={target_id}', (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header.stamp = self.get_clock().now().to_msg()
        self.debug_pub.publish(out)

    def _publish_target_point(self, track, stamp):
        if track is None or track.map_xy is None:
            return
        pt = PointStamped()
        pt.header.frame_id = self.map_frame
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x, pt.point.y = track.map_xy
        self.target_pub.publish(pt)

    # ================================================================
    # Action plumbing
    # ================================================================
    def on_goal(self, _goal):
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warn('a mission is already running -> rejecting goal')
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def _make_pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return ps

    def _set_phase(self, ph):
        with self.lock:
            self.phase = ph

    def _publish_fb(self, goal_handle):
        fb = FB()
        with self.lock:
            fb.phase = self.phase
            fb.target_track_id = -1 if self.target_id is None else self.target_id
            fb.search_progress = float(self.search_progress)
            tr = self.tracks.get(self.target_id) if self.target_id is not None else None
        fb.distance_remaining = float(self._nav_remaining)
        fb.target_distance = float(tr.distance) if tr else -1.0
        if tr and tr.map_xy:
            fb.target_position_map.x = float(tr.map_xy[0])
            fb.target_position_map.y = float(tr.map_xy[1])
        goal_handle.publish_feedback(fb)

    def _result(self, outcome, message):
        r = RES()
        r.outcome = outcome
        with self.lock:
            r.target_track_id = -1 if self.target_id is None else self.target_id
        rp = self._robot_xy_yaw()
        fp = PoseStamped()
        fp.header.frame_id = self.map_frame
        fp.header.stamp = self.get_clock().now().to_msg()
        if rp:
            fp.pose.position.x, fp.pose.position.y = rp[0], rp[1]
            _, _, qz, qw = yaw_to_quaternion(rp[2])
            fp.pose.orientation.z, fp.pose.orientation.w = qz, qw
        r.final_pose = fp
        r.final_distance = float(self._last_distance)
        r.message = message
        self._set_phase(FB.PHASE_IDLE)
        return r

    # ================================================================
    # Mission
    # ================================================================
    def execute(self, goal_handle):
        try:
            return self._run_mission(goal_handle)
        finally:
            with self.lock:
                self.locking_enabled = False
            with self._goal_lock:
                self._goal_active = False

    def _run_mission(self, goal_handle):
        g = goal_handle.request
        stop_d = g.stop_distance if g.stop_distance > 0 else self.stop_distance_m
        hyst = self.stop_hysteresis_m
        upd_d = g.goal_update_distance if g.goal_update_distance > 0 \
            else self.goal_update_distance_m
        spin_a = g.spin_angle if g.spin_angle > 0 else self.spin_angle_rad
        lost_to = g.target_lost_timeout if g.target_lost_timeout > 0 \
            else self.target_lost_timeout_s

        with self.lock:
            self.target_id = None
            self._streak_id = None
            self._streak_n = 0
            self.tracks = {}
            self.search_progress = 0.0
            self._last_distance = -1.0

        self.get_logger().info(
            f'mission start: stop_d={stop_d} spin={spin_a:.2f} upd_d={upd_d} lost_to={lost_to}')

        def canceled():
            return goal_handle.is_cancel_requested

        # ---- UNDOCKING first ----
        # The RPLIDAR motor is powered off while the robot is on the dock, so
        # AMCL cannot localize (no /scan) until we undock. Undock -> wait for the
        # lidar to spin up -> then set the initial pose and wait for Nav2.
        self._set_phase(FB.PHASE_UNDOCKING)
        self._publish_fb(goal_handle)
        try:
            if self.navigator.getDockedStatus():
                self.get_logger().info('undocking')
                self.navigator.undock()
                time.sleep(3.0)   # let the RPLIDAR spin up / first scans arrive
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'undock skipped: {e}')

        # ---- Nav2 up (after undock so AMCL has laser scan) ----
        if self.set_initial_pose:
            ip = self._make_pose(self.initial_pose[0], self.initial_pose[1],
                                 self.initial_pose[2])
            self.navigator.setInitialPose(ip)
        self.get_logger().info('waiting for Nav2 to become active ...')
        self.navigator.waitUntilNav2Active()

        # ---- NAV_TO_SEARCH ----
        self._set_phase(FB.PHASE_NAV_TO_SEARCH)
        self.get_logger().info('navigating to search_pose')
        self.navigator.goToPose(g.search_pose)
        while not self.navigator.isTaskComplete():
            if canceled():
                self.navigator.cancelTask()
                goal_handle.canceled()
                return self._result(RES.OUTCOME_ABORTED, 'canceled during nav')
            fbk = self.navigator.getFeedback()
            if fbk is not None:
                self._nav_remaining = float(getattr(fbk, 'distance_remaining', 0.0))
            self._publish_fb(goal_handle)
            time.sleep(0.2)
        if self.navigator.getResult() != TaskResult.SUCCEEDED:
            goal_handle.abort()
            return self._result(RES.OUTCOME_NAV_FAILED, 'failed to reach search_pose')

        # ---- SEARCHING (fresh lock) ----
        self._set_phase(FB.PHASE_SEARCHING)
        if not self._spin_search(goal_handle, spin_a, fresh=True):
            if canceled():
                goal_handle.canceled()
                return self._result(RES.OUTCOME_ABORTED, 'canceled during search')
            goal_handle.abort()
            return self._result(RES.OUTCOME_TARGET_NOT_FOUND,
                                'no car found during 360 spin')

        # ---- APPROACH / FOLLOW / HOLD ----
        with self.lock:
            self.last_target_seen = time.monotonic()
        last_goal_xy = None
        holding = False
        hold_start = None

        while True:
            # pump the navigator node so getFeedback()/goToPose replies stay fresh
            # (the navigator is not on our executor; its callbacks only run when
            #  one of its blocking helpers is called)
            try:
                self.navigator.isTaskComplete()
            except Exception:  # noqa: BLE001
                pass

            if canceled():
                self.navigator.cancelTask()
                goal_handle.canceled()
                return self._result(RES.OUTCOME_STOPPED_AT_TARGET, 'canceled by client')

            with self.lock:
                tid = self.target_id
                tr = self.tracks.get(tid) if tid is not None else None
                last_seen = self.last_target_seen

            now = time.monotonic()

            # -- lost? --
            if now - last_seen > lost_to:
                self.get_logger().warn('target lost -> re-searching in place')
                self._set_phase(FB.PHASE_SEARCHING)
                self.navigator.cancelTask()
                if not self._spin_search(goal_handle, spin_a,
                                         fresh=self.relock_on_reacquire):
                    if canceled():
                        goal_handle.canceled()
                        return self._result(RES.OUTCOME_ABORTED, 'canceled during re-search')
                    goal_handle.abort()
                    return self._result(RES.OUTCOME_TARGET_LOST, 'target lost')
                with self.lock:
                    self.last_target_seen = time.monotonic()
                last_goal_xy = None
                holding = False
                hold_start = None
                continue

            if tr is None or tr.map_xy is None:
                self._publish_fb(goal_handle)
                time.sleep(self.follow_period_s)
                continue

            dist = tr.distance
            self._last_distance = dist
            tx, ty = tr.map_xy

            if holding:
                self._set_phase(FB.PHASE_HOLDING)
                if dist >= stop_d + hyst:
                    self.get_logger().info('target moved away -> resume approach')
                    holding = False
                    hold_start = None
                    last_goal_xy = None
                    continue
                if hold_start is None:
                    hold_start = now
                elif now - hold_start >= self.hold_finish_s:
                    if g.return_to_dock_on_finish:
                        self._set_phase(FB.PHASE_DOCKING)
                        self.get_logger().info('docking')
                        try:
                            self.navigator.dock()
                        except Exception as e:  # noqa: BLE001
                            self.get_logger().warn(f'dock failed: {e}')
                    goal_handle.succeed()
                    return self._result(RES.OUTCOME_STOPPED_AT_TARGET,
                                        'reached and held stop distance')
            else:
                self._set_phase(FB.PHASE_APPROACHING if dist > stop_d + hyst
                                else FB.PHASE_FOLLOWING)
                moved = (last_goal_xy is None or
                         math.hypot(tx - last_goal_xy[0], ty - last_goal_xy[1]) >= upd_d)
                if dist <= stop_d:
                    self.navigator.cancelTask()
                    self._set_phase(FB.PHASE_HOLDING)
                    holding = True
                    hold_start = None
                elif moved:
                    rp = self._robot_xy_yaw()
                    if rp is None:
                        self.get_logger().warn(
                            'map->base TF unavailable; follow goal not sent',
                            throttle_duration_sec=2.0)
                    else:
                        # Do not command Nav2 into the detected object's occupied
                        # cell. Aim for a point stop_d metres before the target.
                        dx, dy = tx - rp[0], ty - rp[1]
                        map_distance = math.hypot(dx, dy)
                        if map_distance > 1e-6:
                            stand_off = min(stop_d, map_distance)
                            goal_x = tx - stand_off * dx / map_distance
                            goal_y = ty - stand_off * dy / map_distance
                            yaw = yaw_between(rp[0], rp[1], tx, ty)
                            self.navigator.goToPose(
                                self._make_pose(goal_x, goal_y, yaw))
                            last_goal_xy = (tx, ty)

            fbk = self.navigator.getFeedback()
            if fbk is not None:
                self._nav_remaining = float(getattr(fbk, 'distance_remaining', 0.0))
            self._publish_fb(goal_handle)
            time.sleep(self.follow_period_s)

    def _spin_search(self, goal_handle, spin_angle, fresh):
        """Spin in place while the perception timer tries to (re)lock the target.

        fresh=True  : clear target_id, lock the first `car` near image centre.
        fresh=False : keep target_id, just wait for that id to reappear.
        Returns True when the target is (re)acquired, False if the spin finished first.
        """
        with self.lock:
            if fresh:
                self.target_id = None
            self._streak_id = None
            self._streak_n = 0
            self.locking_enabled = True
            self.search_progress = 0.0

        try:
            self.navigator.spin(spin_dist=float(spin_angle),
                                time_allowance=int(spin_angle / 0.4) + 15)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'spin start failed: {e}')

        acquired = False
        while not self.navigator.isTaskComplete():
            if goal_handle.is_cancel_requested:
                self.navigator.cancelTask()
                break
            with self.lock:
                tid = self.target_id
                have_it = tid is not None and (fresh or tid in self.tracks)
            fbk = self.navigator.getFeedback()
            if fbk is not None and hasattr(fbk, 'angular_distance_traveled'):
                with self.lock:
                    self.search_progress = min(
                        1.0, abs(fbk.angular_distance_traveled) / max(spin_angle, 1e-3))
            if have_it:
                self.navigator.cancelTask()
                acquired = True
                break
            self._publish_fb(goal_handle)
            time.sleep(0.1)

        with self.lock:
            self.locking_enabled = False
            if not acquired and not fresh:
                acquired = self.target_id is not None and self.target_id in self.tracks
            elif not acquired and fresh:
                acquired = self.target_id is not None
            if acquired:
                self.search_progress = 1.0
        return acquired


def main():
    rclpy.init()
    node = SearchAndTrackServer()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
