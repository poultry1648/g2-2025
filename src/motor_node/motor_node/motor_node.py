"""
Standalone ROS 2 node that owns the robot's motors.

Robot application code talks to this node over ROS with MotorCommand and
MotorState messages, so the same application runs against the simulator and
the real robot; only the transport (Gazebo vs TalonFX) changes.
"""

from motor_msgs.msg import MotorCommand, MotorState
import rclpy
from rclpy.node import Node

from .config import load_motors
from .gazebo_transport import GazeboBus, GazeboTransport


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('config_file', '')
        self.declare_parameter('transport', '')
        self.declare_parameter('update_rate', 0.0)
        self.declare_parameter('state_rate', 0.0)

        config_file = self.get_parameter('config_file').value
        if not config_file:
            raise RuntimeError('motor_node requires the config_file parameter')
        motor_set = load_motors(config_file)

        transport = self.get_parameter('transport').value or motor_set.transport
        update_rate = self.get_parameter('update_rate').value or motor_set.update_rate
        state_rate = self.get_parameter('state_rate').value or motor_set.state_rate

        self._bus = None
        self._entries = []
        if transport == 'sim':
            self._bus = GazeboBus(self, motor_set.joint_states_topic)
            for config in motor_set.motors:
                device = GazeboTransport(config)
                self._bus.add(device)
                self._entries.append(self._bind(config, device))
        elif transport == 'talonfx':
            from .talonfx_transport import TalonFXTransport
            for config in motor_set.motors:
                device = TalonFXTransport(config)
                self._entries.append(self._bind(config, device))
        else:
            raise RuntimeError(f'unknown transport "{transport}"')

        self._dt = 1.0 / update_rate
        self.create_timer(self._dt, self._on_update)
        self.create_timer(1.0 / state_rate, self._on_publish)
        self.get_logger().info(
            f'motor_node ready: transport={transport} '
            f'motors={len(self._entries)} update_rate={update_rate:.1f}')

    def _bind(self, config, device):
        command_sub = self.create_subscription(
            MotorCommand, config.command_topic,
            lambda msg, d=device: self._on_command(msg, d), 10)
        state_pub = self.create_publisher(MotorState, config.state_topic, 10)
        return config, device, command_sub, state_pub

    def _on_command(self, msg, device):
        device.set_command(msg.mode, msg.target, msg.feedforward, msg.enable)

    def _on_update(self):
        if self._bus is not None:
            self._bus.update(self._dt)
        else:
            for _config, device, _sub, _pub in self._entries:
                device.update(self._dt)

    def _on_publish(self):
        for _config, device, _sub, pub in self._entries:
            state = device.state()
            message = MotorState()
            message.position = state.position
            message.velocity = state.velocity
            message.stator_current = state.stator_current
            message.supply_current = state.supply_current
            message.applied_voltage = state.applied_voltage
            message.torque = state.torque
            message.mode = state.mode
            message.enabled = state.enabled
            pub.publish(message)

    def shutdown(self):
        for _config, device, _sub, _pub in self._entries:
            device.close()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
