"""Transport interface shared by the simulated and real motor backends."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import enum

from .motor_model import DcMotorModel


class ControlMode(enum.IntEnum):
    """Interpretation of a command target. Values match MotorCommand.msg."""

    DUTY_CYCLE = 0
    VOLTAGE = 1
    TORQUE_CURRENT = 2
    VELOCITY = 3
    POSITION = 4


class NeutralMode(enum.Enum):
    COAST = 'coast'
    BRAKE = 'brake'


@dataclass
class MotorConfig:
    """Everything needed to drive one joint, identical sim and real."""

    name: str
    joint: str
    motor: DcMotorModel = field(default_factory=DcMotorModel)
    gear_ratio: float = 1.0
    efficiency: float = 1.0
    inverted: bool = False
    neutral_mode: NeutralMode = NeutralMode.BRAKE
    command_topic: str = ''
    state_topic: str = ''
    effort_topic: str = ''
    can_id: int = -1
    can_bus: str = ''
    velocity_gains: tuple = (0.0, 0.0, 0.0)
    position_gains: tuple = (0.0, 0.0, 0.0)

    @property
    def sign(self):
        return -1.0 if self.inverted else 1.0

    @property
    def max_joint_torque(self):
        return self.motor.stall_torque * self.gear_ratio * self.efficiency

    def limit_joint_torque(self, torque):
        limit = self.max_joint_torque
        return max(-limit, min(limit, torque))


@dataclass
class MotorState:
    """Feedback on the mechanism (joint) side: rotations and rotations/second."""

    position: float = 0.0
    velocity: float = 0.0
    stator_current: float = 0.0
    supply_current: float = 0.0
    applied_voltage: float = 0.0
    torque: float = 0.0
    mode: int = ControlMode.DUTY_CYCLE
    enabled: bool = False


class MotorTransport(ABC):
    """
    One motor.

    ``update`` advances the simulated device; real devices are driven
    directly by ``set_command`` and ignore ``update``.
    """

    def __init__(self, config):
        self.config = config

    @abstractmethod
    def set_command(self, mode, target, feedforward, enabled):
        ...

    @abstractmethod
    def update(self, dt):
        ...

    @abstractmethod
    def state(self):
        ...

    def close(self):
        pass
