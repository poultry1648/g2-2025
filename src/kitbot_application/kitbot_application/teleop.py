import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


HELP = (
    '\n'
    'kitbot teleop (keyboard mode)\n'
    '  w / s       forward / backward\n'
    '  a / d       turn left / right\n'
    '  q           toggle rollers\n'
    '  space       stop (wheels)\n'
    '  Ctrl-C      quit\n'
)


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.declare_parameter('input', 'joy')
        self.declare_parameter('linear_speed', 5)
        self.declare_parameter('angular_speed', 5.0)
        self.declare_parameter('roller_speed', 15.0)
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 2)
        self.declare_parameter('invert_linear', True)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('axis_deadzone', 0.15)
        self.declare_parameter('button_roller', 5)
        self.declare_parameter('key_timeout', 0.3)
        self.declare_parameter('publish_rate', 20.0)

        self._source = self.get_parameter('input').value
        self._linear = self.get_parameter('linear_speed').value
        self._angular = self.get_parameter('angular_speed').value
        self._roller_speed = self.get_parameter('roller_speed').value
        self._axis_linear = self.get_parameter('axis_linear').value
        self._axis_angular = self.get_parameter('axis_angular').value
        self._invert_linear = self.get_parameter('invert_linear').value
        self._invert_angular = self.get_parameter('invert_angular').value
        self._deadzone = self.get_parameter('axis_deadzone').value
        self._button_roller = self.get_parameter('button_roller').value
        self._key_timeout = self.get_parameter('key_timeout').value
        rate = self.get_parameter('publish_rate').value

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._roller_pub = self.create_publisher(Float64, 'roller_cmd', 10)

        self._active = {}
        self._seen_real = {}
        self._last_roller = None

        if self._source == 'joy':
            self._sub = self.create_subscription(Joy, 'joy', self._on_joy, 10)
            self.get_logger().info('teleop reading joystick on /joy')
        else:
            self._timer = self.create_timer(1.0 / rate, self._on_key_timer)

    def _publish_roller(self, value):
        if value != self._last_roller:
            self._last_roller = value
            self._roller_pub.publish(Float64(data=value))
            self.get_logger().info(
                'rollers ' + ('on' if value else 'off'))

    def _stick(self, index, axes):
        if index >= len(axes):
            return 0.0
        value = axes[index]
        if not self._seen_real.get(index, False):
            if abs(value) < 0.9:
                self._seen_real[index] = True
            else:
                return 0.0
        if abs(value) < self._deadzone:
            return 0.0
        return value

    def _on_joy(self, msg):
        axes = msg.axes
        sticks = [axes[i] for i in (0, 1, 2, 3) if i < len(axes)]
        if sticks and all(v == 1.0 for v in sticks):
            linear = 0.0
            angular = 0.0
        else:
            linear = self._stick(self._axis_linear, axes) * self._linear
            angular = self._stick(self._axis_angular, axes) * self._angular
            if self._invert_linear:
                linear = -linear
            if self._invert_angular:
                angular = -angular

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._cmd_pub.publish(twist)

        pressed = (self._button_roller < len(msg.buttons)
                   and msg.buttons[self._button_roller] == 1)
        self._publish_roller(self._roller_speed if pressed else 0.0)

    def _read_keys(self):
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            chars = sys.stdin.read(1)
            if not chars:
                break
            keys.append(chars)
        return keys

    def _on_key_timer(self):
        now = time.monotonic()
        for key in self._read_keys():
            key = key.lower()
            if key == 'q':
                self._publish_roller(
                    0.0 if self._last_roller else self._roller_speed)
            elif key in 'wasd ':
                self._active[key] = now

        self._active = {
            k: t for k, t in self._active.items() if now - t < self._key_timeout}

        twist = Twist()
        if 'w' in self._active:
            twist.linear.x += self._linear
        if 's' in self._active:
            twist.linear.x -= self._linear
        if 'a' in self._active:
            twist.angular.z += self._angular
        if 'd' in self._active:
            twist.angular.z -= self._angular
        if ' ' in self._active:
            twist = Twist()
        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()

    if node._source == 'joy':
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node._publish_roller(0.0)
            node._cmd_pub.publish(Twist())
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    if not sys.stdin.isatty():
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit('keyboard teleop needs an interactive terminal')

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print(HELP)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node._publish_roller(0.0)
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
