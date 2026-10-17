import rclpy
from rclpy.node import Node
from gesture_robot_interfaces.msg import GestureCommand

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        # '/gesture/command' 도로에서 메시지가 들어오면 listener_callback 함수를 실행합니다.
        self.subscription = self.create_subscription(
            GestureCommand,
            '/gesture/command',
            self.listener_callback,
            10
        )
        self.get_logger().info('테스트 서브스크라이버가 대기 중입니다...')

    def listener_callback(self, msg):
        # 들어온 명령 번호에 따라 읽기 쉽게 출력해 줍니다.
        cmd_name = "UNKNOWN"
        if msg.command == GestureCommand.NONE:
            cmd_name = "NONE"
        elif msg.command == GestureCommand.START:
            cmd_name = "START"
        elif msg.command == GestureCommand.STOP:
            cmd_name = "STOP"

        self.get_logger().info(f'[수신 성공!] 명령: {msg.command} ({cmd_name}), 신뢰도: {msg.confidence:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TestSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
