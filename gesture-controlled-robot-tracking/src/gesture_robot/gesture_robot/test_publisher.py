import rclpy
from rclpy.node import Node
# 우리가 만든 커스텀 메시지를 불러옵니다!
from gesture_robot_interfaces.msg import GestureCommand

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        # '/gesture/command' 라는 도로(토픽)로 메시지를 보낼 준비를 합니다.
        self.publisher_ = self.create_publisher(GestureCommand, '/gesture/command', 10)
        # 1초마다 timer_callback 함수를 실행합니다.
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('테스트 퍼블리셔가 시작되었습니다!')

    def timer_callback(self):
        msg = GestureCommand()
        # 헤더 시간 기록
        msg.header.stamp = self.get_clock().now().to_msg()
        # 우리가 정의한 START (1) 명령과 신뢰도 95% 설정
        msg.command = GestureCommand.START
        msg.confidence = 0.95

        self.publisher_.publish(msg)
        self.get_logger().info(f'[발송] 명령: {msg.command} (START), 신뢰도: {msg.confidence}')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
