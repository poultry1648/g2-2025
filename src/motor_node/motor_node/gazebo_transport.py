"""
Simulated motor backend.

The motor model runs here, in the simulated "firmware", exactly as vendor
firmware would run it on a real robot. Torque is applied to the Gazebo joint
through the bridged ``cmd_force`` topic, and positions and velocities are read
back from the bridged ``joint_state`` topic.
"""

import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .motor_model import Pid
from .transport import ControlMode, MotorState, MotorTransport, NeutralMode


class GazeboBus:
    """
    Shared Gazebo plumbing for every simulated motor on one robot.

    One ``joint_states`` subscription and one set of effort publishers are
    created for the whole node rather than one per motor.
    """

    def __init__(self, node, joint_states_topic):
        self._node = node
        self._positions = {}
        self._velocities = {}
        self._effort_pubs = {}
        self._transports = []
        self._sub = node.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self._velocities[name] = msg.velocity[i]

    def position(self, joint):
        return self._positions.get(joint, 0.0)

    def velocity(self, joint):
        return self._velocities.get(joint, 0.0)

    def add(self, transport):
        self._transports.append(transport)
        transport.attach_bus(self)

    def effort_publisher(self, topic):
        if topic not in self._effort_pubs:
            self._effort_pubs[topic] = self._node.create_publisher(
                Float64, topic, 10)
        return self._effort_pubs[topic]

    def update(self, dt):
        for transport in self._transports:
            transport.update(dt)


class GazeboTransport(MotorTransport):
    def __init__(self, config):
        super().__init__(config)
        self._bus = None
        self._pub = None
        self._mode = ControlMode.DUTY_CYCLE
        self._target = 0.0
        self._feedforward = 0.0
        self._enabled = False
        self._state = MotorState()
        self._velocity_pid = Pid(
            config.velocity_gains, output_limit=config.motor.nominal_voltage,
            integral_limit=config.motor.nominal_voltage)
        self._position_pid = Pid(
            config.position_gains, output_limit=config.motor.nominal_voltage,
            integral_limit=config.motor.nominal_voltage)

    def attach_bus(self, bus):
        self._bus = bus
        self._pub = bus.effort_publisher(self.config.effort_topic)

    def set_command(self, mode, target, feedforward, enabled):
        if mode != self._mode:
            self._velocity_pid.reset()
            self._position_pid.reset()
        self._mode = ControlMode(mode)
        self._target = target
        self._feedforward = feedforward
        self._enabled = enabled

    def _neutral(self, motor_speed):
        if self.config.neutral_mode == NeutralMode.BRAKE:
            torque = self.config.motor.brake_torque(motor_speed)
            return torque, torque / self.config.motor.torque_constant
        return 0.0, 0.0

    def update(self, dt):
        cfg = self.config
        model = cfg.motor
        sign = cfg.sign
        joint_rad = self._bus.position(cfg.joint)
        joint_vel = self._bus.velocity(cfg.joint)
        joint_rot = joint_rad / (2.0 * math.pi)
        joint_rps = joint_vel / (2.0 * math.pi)
        motor_speed = joint_vel * cfg.gear_ratio * sign

        if not self._enabled:
            torque_motor, current = self._neutral(motor_speed)
            voltage = 0.0
        elif self._mode == ControlMode.TORQUE_CURRENT:
            torque_motor = model.current_to_torque(self._target)
            current = torque_motor / model.torque_constant
            voltage = 0.0
        else:
            if self._mode == ControlMode.DUTY_CYCLE:
                duty = max(-1.0, min(1.0, self._target))
                voltage = duty * model.nominal_voltage
            elif self._mode == ControlMode.VOLTAGE:
                voltage = max(
                    -model.nominal_voltage,
                    min(model.nominal_voltage, self._target))
            elif self._mode == ControlMode.VELOCITY:
                error = self._target - joint_rps
                voltage = sign * (
                    self._velocity_pid(error, dt) + self._feedforward)
            elif self._mode == ControlMode.POSITION:
                error = self._target - joint_rot
                voltage = sign * (
                    self._position_pid(error, dt) + self._feedforward)
            else:
                voltage = 0.0
            torque_motor, current = model.apply_voltage(voltage, motor_speed)

        torque_joint = cfg.limit_joint_torque(
            torque_motor * cfg.gear_ratio * cfg.efficiency * sign)
        self._pub.publish(Float64(data=torque_joint))

        supply_current = abs(current) * min(
            1.0, abs(voltage) / model.nominal_voltage) if model.nominal_voltage else 0.0
        self._state = MotorState(
            position=joint_rot,
            velocity=joint_rps,
            stator_current=current,
            supply_current=supply_current,
            applied_voltage=voltage,
            torque=torque_joint,
            mode=int(self._mode),
            enabled=self._enabled,
        )

    def state(self):
        return self._state

    def close(self):
        if self._pub is not None:
            self._pub.publish(Float64(data=0.0))
