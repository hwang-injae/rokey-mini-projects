"""turtlebot3(Gazebo)가 손 위치를 따라가도록 만드는 데모 전용 노드.

turtle_pursuit_node와 동일한 pursuit 계산(core/pursuit_controller.py)을
공유하되, turtlesim의 Pose/두 번째 turtle 대신 Gazebo가 브릿지로 제공하는
/odom(로봇의 실제 world 좌표/방향)과 tracking/object(손 위치, 카메라
픽셀 오차)로 가상의 목표 좌표를 계산해 사용한다. Gazebo에는 turtlesim의
target turtle 같은 시각적 목표 표식이 없어, 목표 좌표는 화면에 보이지
않는 값으로만 존재한다.

controller_node와 동시에 실행하면 안 된다(같은 cmd_vel을 두고 충돌).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from gesture_robot_interfaces.msg import GestureCommand, TrackedObject
from gesture_robot.core.pursuit_controller import PursuitController
from gesture_robot.core.tracking_state import TrackingStateMachine


class GazeboPursuitNode(Node):
    """손 위치(tracking/object)를 Gazebo world 좌표로 변환해 로봇이 쫓아가게 한다."""

    def __init__(self) -> None:
        super().__init__("gazebo_pursuit_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("linear_gain", 1.5),
                ("angular_gain", 4.0),
                ("max_linear_speed", 0.3),
                ("max_angular_speed", 1.5),
                ("distance_tolerance", 0.15),
                # 손 오차(px) -> 로봇 스폰 지점 기준 가상 목표 오프셋(m) 배율.
                ("pixel_range_for_full_scale", 150.0),
                ("target_range_m", 1.2),
            ],
        )
        self._controller = PursuitController(
            linear_gain=float(self.get_parameter("linear_gain").value),
            angular_gain=float(self.get_parameter("angular_gain").value),
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
            distance_tolerance=float(self.get_parameter("distance_tolerance").value),
        )
        pixel_range = float(self.get_parameter("pixel_range_for_full_scale").value)
        target_range = float(self.get_parameter("target_range_m").value)
        self._scale = target_range / pixel_range

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0
        self._target_x = 0.0
        self._target_y = 0.0
        self._have_odom = False
        self._state_machine = TrackingStateMachine()

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Odometry, "odom", self._on_odom, best_effort_qos)
        # gesture_node/object_tracking_node가 BEST_EFFORT로 발행하므로 맞춰야 한다.
        self.create_subscription(
            TrackedObject, "tracking/object", self._on_tracking_result, best_effort_qos
        )
        self.create_subscription(
            GestureCommand, "gesture/command", self._on_gesture_command, 10
        )
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.05, self._on_timer)

        self.get_logger().info("gazebo_pursuit_node 시작됨")

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_theta = self._quaternion_to_yaw(msg.pose.pose.orientation)
        self._have_odom = True

    def _on_tracking_result(self, msg: TrackedObject) -> None:
        if not msg.detected:
            return
        # 화면 오차를 로봇 스폰 지점(0,0) 기준 가상 목표 좌표로 변환한다.
        # (오른쪽 + / 아래쪽 + 이미지 규칙과 world 좌표 부호가 달라 y는 반전)
        self._target_x = msg.error_x * self._scale
        self._target_y = -msg.error_y * self._scale

    def _on_gesture_command(self, msg: GestureCommand) -> None:
        self._state_machine.update(msg.command)
        if not self._state_machine.is_active():
            self._cmd_pub.publish(Twist())

    def _on_timer(self) -> None:
        if not self._state_machine.is_active() or not self._have_odom:
            return

        velocity = self._controller.calculate(
            robot_x=self._robot_x,
            robot_y=self._robot_y,
            robot_theta=self._robot_theta,
            target_x=self._target_x,
            target_y=self._target_y,
        )
        cmd = Twist()
        cmd.linear.x = velocity.linear_x
        cmd.angular.z = velocity.angular_z
        self._cmd_pub.publish(cmd)

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboPursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
