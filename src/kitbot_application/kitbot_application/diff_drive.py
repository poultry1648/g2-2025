from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class DiffDrive(Node):
    def __init__(self):
        super().__init__('diff_drive')
        self.declare_parameter('wheel_radius', 0.0762)
        self.declare_parameter('track_width', 0.468)
        self.declare_parameter('left_direction', -1.0)
        self.declare_parameter('right_direction', 1.0)
        self.declare_parameter('max_wheel_speed', 300.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self._radius = self.get_parameter('wheel_radius').value
        self._track = self.get_parameter('track_width').value
        self._left_dir = self.get_parameter('left_direction').value
        self._right_dir = self.get_parameter('right_direction').value
        self._max_speed = self.get_parameter('max_wheel_speed').value
        self._timeout = self.get_parameter('cmd_timeout').value

        self._left_pub = self.create_publisher(Float64, 'left_wheel_cmd', 10)
        self._right_pub = self.create_publisher(Float64, 'right_wheel_cmd', 10)
        self._sub = self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self._last_cmd = None
        self._timer = self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            'diff_drive ready: wheel_radius=%.4f track_width=%.4f'
            % (self._radius, self._track))

    def _clamp(self, value):
        return max(-self._max_speed, min(self._max_speed, value))

    def _publish(self, left, right):
        self._left_pub.publish(Float64(data=self._clamp(left)))
        self._right_pub.publish(Float64(data=self._clamp(right)))

    def _on_cmd_vel(self, msg):
        half_track = self._track / 2.0
        left_linear = msg.linear.x - msg.angular.z * half_track
        right_linear = msg.linear.x + msg.angular.z * half_track
        self._publish(
            self._left_dir * left_linear / self._radius,
            self._right_dir * right_linear / self._radius,
        )
        self._last_cmd = self.get_clock().now()

    def _on_timer(self):
        if self._last_cmd is None:
            return
        age = (self.get_clock().now() - self._last_cmd).nanoseconds / 1e9
        if age > self._timeout:
            self._last_cmd = None
            self._publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDrive()
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
