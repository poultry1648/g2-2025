import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")
        self._go_forward_cmd = self.create_publisher(Twist, "/g2/swerve/target", 10)
        self.create_timer(0.05, self._publish_swerve_target)

    def _publish_swerve_target(self):
        target = Twist()
        target.linear.y = 10.0
        self._go_forward_cmd.publish(target)


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
