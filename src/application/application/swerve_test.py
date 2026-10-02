from rclpy.node import Node
from motor_msgs.msg import MotorCommand, MotorState
import rclpy

class Swerve_Test(Node):
    def __init__(self):
        super().__init__("swerve_test")
        self._cmd = self.create_publisher(MotorCommand, "/g2/motors/swerve_axel_front_left_joint/command", 10)
        self.create_subscription(MotorState, "/g2/motors/swerve_axel_front_left_joint/state", self._on_state, 10)
        self.create_timer(0.02, self._publish)

    def _publish(self):
        msg = MotorCommand()
        msg.mode = MotorCommand.VELOCITY
        msg.target = 5.0
        msg.enable = True
        self._cmd.publish(msg)

    def _on_state(self, msg):
        pass

def main():
    rclpy.init()
    node = Swerve_Test()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        stop = MotorCommand()
        stop.enable = False
        for _ in range(5):
            node._cmd.publish(stop)
        node.destroy_node()
        rclpy.shutdown()
