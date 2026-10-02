#!/usr/bin/env python3
"""Mission trigger  (runs on the webcam PC with webcam_detector, via station.launch.py).

Subscribes /webcam/rc_car_detected (transient_local). On the FIRST `true`,
it builds a SearchAndTrack goal from parameters and sends it once to
/search_and_track, then just logs feedback / result.
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from rc_tracker_interfaces.action import SearchAndTrack

PHASE = {0: 'IDLE', 1: 'UNDOCKING', 2: 'NAV_TO_SEARCH', 3: 'SEARCHING',
         4: 'APPROACHING', 5: 'FOLLOWING', 6: 'HOLDING', 7: 'DOCKING'}
OUTCOME = {0: 'UNKNOWN', 1: 'STOPPED_AT_TARGET', 2: 'TARGET_NOT_FOUND',
           3: 'TARGET_LOST', 4: 'NAV_FAILED', 5: 'ABORTED'}


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class MissionTrigger(Node):

    def __init__(self):
        super().__init__('mission_trigger_node')

        self.declare_parameter('trigger_topic', '/webcam/rc_car_detected')
        self.declare_parameter('action_name', '/search_and_track')
        # search_pose = [x, y, yaw(rad)] in the map frame.
        # Default only; the real value is set in config/mission_params.yaml.
        self.declare_parameter('search_pose', [1.0, 0.5, 0.0])
        self.declare_parameter('stop_distance', 0.30)
        self.declare_parameter('spin_angle', 2.0 * math.pi)
        self.declare_parameter('goal_update_distance', 0.25)
        self.declare_parameter('target_lost_timeout', 5.0)
        self.declare_parameter('return_to_dock_on_finish', False)

        p = self.get_parameter
        self.search_pose = [float(v) for v in p('search_pose').value]
        self.stop_distance = float(p('stop_distance').value)
        self.spin_angle = float(p('spin_angle').value)
        self.goal_update_distance = float(p('goal_update_distance').value)
        self.target_lost_timeout = float(p('target_lost_timeout').value)
        self.return_to_dock = bool(p('return_to_dock_on_finish').value)

        self._client = ActionClient(self, SearchAndTrack, p('action_name').value)
        self._sent = False

        self.create_subscription(
            Bool, p('trigger_topic').value, self.on_trigger, latched_qos())
        self.get_logger().info(
            'mission_trigger_node: waiting for /webcam/rc_car_detected ...')

    # ------------------------------------------------------------------
    def on_trigger(self, msg: Bool):
        if self._sent or not msg.data:
            return
        self._sent = True
        self.get_logger().info('trigger received -> sending SearchAndTrack goal')

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('SearchAndTrack action server not available')
            self._sent = False
            return

        goal = SearchAndTrack.Goal()
        goal.search_pose = self._pose(self.search_pose)
        goal.stop_distance = self.stop_distance
        goal.spin_angle = self.spin_angle
        goal.goal_update_distance = self.goal_update_distance
        goal.target_lost_timeout = self.target_lost_timeout
        goal.return_to_dock_on_finish = self.return_to_dock

        self._client.send_goal_async(
            goal, feedback_callback=self.on_feedback
        ).add_done_callback(self.on_goal_response)

    def _pose(self, xyw):
        x, y, yaw = xyw
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        return ps

    def on_feedback(self, fb):
        f = fb.feedback
        self.get_logger().info(
            f'[{PHASE.get(f.phase, f.phase)}] id={f.target_track_id} '
            f'nav_rem={f.distance_remaining:.2f}m '
            f'target_d={f.target_distance:.2f}m '
            f'search={f.search_progress * 100:.0f}%')

    def on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('goal rejected by server')
            return
        self.get_logger().info('goal accepted')
        gh.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        r = future.result().result
        self.get_logger().info(
            f'RESULT outcome={OUTCOME.get(r.outcome, r.outcome)} '
            f'id={r.target_track_id} final_d={r.final_distance:.2f}m '
            f'msg="{r.message}"')


def main():
    rclpy.init()
    node = MissionTrigger()
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
