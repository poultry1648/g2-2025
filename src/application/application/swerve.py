from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist
from motor_msgs.msg import MotorCommand, MotorState
from application import constants
import numpy
import math
import rclpy

class Swerve(Node):
    def __init__(self):
        super().__init__("swerve")
        self._swerve_wheel_front_left_cmd = self.create_publisher(MotorCommand, "/g2/motors/swerve_wheel_front_left_joint/command", 10)
        self._swerve_axel_front_left_cmd = self.create_publisher(MotorCommand, "/g2/motors/swerve_axel_front_left_joint/command", 10)
        self._swerve_wheel_front_right_cmd = self.create_publisher(MotorCommand, "/g2/motors/swerve_wheel_front_right_joint/command", 10)
        self._swerve_axel_front_right_cmd = self.create_publisher(MotorCommand, "/g2/motors/swerve_axel_front_right_joint/command", 10)
        self._swerve_wheel_back_left_cmd = self.create_publisher(MotorCommand, "/g2/motors/swerve_wheel_back_left_joint/command", 10)
        self._swerve_axel_back_left_cmd = self.create_publisher(MotorCommand, "/g2/motors/swerve_axel_back_left_joint/command", 10)
        self._swerve_wheel_back_right_cmd = self.create_publisher(MotorCommand, "/g2/motors/swerve_wheel_back_right_joint/command", 10)
        self._swerve_axel_back_right_cmd = self.create_publisher(MotorCommand, "/g2/motors/swerve_axel_back_right_joint/command", 10)

        self.create_subscription(Twist, "/g2/swerve/target", self._on_twist, 10)

    def _on_twist(self, msg: Twist):
        front_left = msg.angular.z * constants.RADIUS * constants.FRONT_LEFT_TANGENT_VECTOR
        front_right = msg.angular.z * constants.RADIUS * constants.FRONT_RIGHT_TANGENT_VECTOR
        back_left = msg.angular.z * constants.RADIUS * constants.BACK_LEFT_TANGENT_VECTOR
        back_right = msg.angular.z * constants.RADIUS * constants.BACK_RIGHT_TANGENT_VECTOR

        module_vectors = [front_left, front_right, back_left, back_right]

        module_vectors = [mod + [msg.linear.x, msg.linear.y] for mod in module_vectors]

        module_vectors = [[math.atan2(mod[1], mod[0])/(2*numpy.pi), math.hypot(mod[1], mod[0])] for mod in module_vectors]

        self._publish_swerve_commands(module_vectors[0], self._swerve_wheel_front_left_cmd, self._swerve_axel_front_left_cmd)
        self._publish_swerve_commands(module_vectors[1], self._swerve_wheel_front_right_cmd, self._swerve_axel_front_right_cmd)
        self._publish_swerve_commands(module_vectors[2], self._swerve_wheel_back_left_cmd, self._swerve_axel_back_left_cmd)
        self._publish_swerve_commands(module_vectors[3], self._swerve_wheel_back_right_cmd, self._swerve_axel_back_right_cmd)

    def _publish_swerve_commands(self, target_state, wheel_command: Publisher, axel_command: Publisher):
        wheel_msg = MotorCommand()
        axel_msg = MotorCommand()

        wheel_msg.mode = MotorCommand.VELOCITY
        wheel_msg.target = target_state[1]
        wheel_msg.enable = True

        axel_msg.mode = MotorCommand.POSITION
        axel_msg.target = target_state[0]
        axel_msg.enable = True

        wheel_command.publish(wheel_msg)
        axel_command.publish(axel_msg)


def main():
    rclpy.init()
    node = Swerve()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        stop = MotorCommand()
        stop.enable = False
        for _ in range(5):
            node._swerve_wheel_front_left_cmd.publish(stop)
            node._swerve_axel_front_left_cmd.publish(stop)
            node._swerve_wheel_front_right_cmd.publish(stop)
            node._swerve_axel_front_right_cmd.publish(stop)
            node._swerve_wheel_back_left_cmd.publish(stop)
            node._swerve_axel_back_left_cmd.publish(stop)
            node._swerve_wheel_back_right_cmd.publish(stop)
            node._swerve_axel_back_right_cmd.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

