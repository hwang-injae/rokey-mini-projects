"""turtle1이 target turtle의 실제 좌표를 추적하도록 만드는 데모 전용 노드.

controller_node의 이동 제어(tracking_controller.py)는 '카메라가 로봇 자신에게
달려 있다'는 전제(화면 오차 = 로봇이 돌아야 할 각도)로 설계되어 실제 공/Gazebo
시나리오에 맞다. 지금은 웹캠이 사람을 비추고 turtle1은 별개의 turtlesim 좌표계에
있어 그 전제가 성립하지 않으므로, turtle1과 target의 실제 pose를 direct로 비교하는
pursuit 제어(core/pursuit_controller.py)를 이 노드에서 별도로 담당한다.
controller_node와 동시에 실행하면 안 된다(같은 turtle1/cmd_vel을 두고 충돌).
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose

from gesture_robot_interfaces.msg import GestureCommand
from gesture_robot.core.pursuit_controller import PursuitController
from gesture_robot.core.tracking_state import TrackingStateMachine


class TurtlePursuitNode(Node):
    """target turtle의 위치를 turtle1이 실제로 쫓아가도록 속도를 계산한다."""

    def __init__(self) -> None:
        super().__init__("turtle_pursuit_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("linear_gain", 1.5),
                ("angular_gain", 4.0),
                ("max_linear_speed", 2.0),
                ("max_angular_speed", 3.0),
                ("distance_tolerance", 0.3),
            ],
        )
        self._controller = PursuitController(
            linear_gain=float(self.get_parameter("linear_gain").value),
            angular_gain=float(self.get_parameter("angular_gain").value),
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
            distance_tolerance=float(self.get_parameter("distance_tolerance").value),
        )

        self._turtle_pose: Pose | None = None
        self._target_pose: Pose | None = None
        self._state_machine = TrackingStateMachine()

        self.create_subscription(Pose, "turtle1/pose", self._on_turtle_pose, 10)
        self.create_subscription(Pose, "target/pose", self._on_target_pose, 10)
        self.create_subscription(
            GestureCommand, "gesture/command", self._on_gesture_command, 10
        )
        self._cmd_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.create_timer(0.05, self._on_timer)

        self.get_logger().info("turtle_pursuit_node 시작됨")

    def _on_turtle_pose(self, msg: Pose) -> None:
        self._turtle_pose = msg

    def _on_target_pose(self, msg: Pose) -> None:
        self._target_pose = msg

    def _on_gesture_command(self, msg: GestureCommand) -> None:
        self._state_machine.update(msg.command)
        if not self._state_machine.is_active():
            # target(추적 대상 표시용 turtle)의 움직임과 무관하게, 제스처가
            # STOP/NONE(초기)이면 turtle1을 그 자리에서 즉시 정지시킨다.
            self._cmd_pub.publish(Twist())

    def _on_timer(self) -> None:
        if not self._state_machine.is_active():
            return
        if self._turtle_pose is None or self._target_pose is None:
            return

        velocity = self._controller.calculate(
            robot_x=self._turtle_pose.x,
            robot_y=self._turtle_pose.y,
            robot_theta=self._turtle_pose.theta,
            target_x=self._target_pose.x,
            target_y=self._target_pose.y,
        )
        cmd = Twist()
        cmd.linear.x = velocity.linear_x
        cmd.angular.z = velocity.angular_z
        self._cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurtlePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
