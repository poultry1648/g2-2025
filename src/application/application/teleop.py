import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")
        self._twist_cmd = self.create_publisher(Twist, "/g2/swerve/target", 10)
        self.create_subscription(Joy, "/joy", self._joystick_callback, 10)

    def _joystick_callback(self, joy: Joy):
        twist = Twist()
        twist.linear.x = float(joy.axes[1])
        twist.linear.y = float(joy.axes[0])
        self._twist_cmd.publish(twist)


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
