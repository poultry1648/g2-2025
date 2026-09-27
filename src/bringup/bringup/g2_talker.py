import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('g2_talker')
        self.declare_parameter('publish_period', 1.0)
        period = self.get_parameter('publish_period').value
        self._count = 0
        self._publisher = self.create_publisher(String, 'chatter', 10)
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info('g2_talker started, publishing on /chatter')

    def _on_timer(self):
        msg = String()
        msg.data = f'hello world {self._count}'
        self._publisher.publish(msg)
        self.get_logger().info(f'publishing: "{msg.data}"')
        self._count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
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
