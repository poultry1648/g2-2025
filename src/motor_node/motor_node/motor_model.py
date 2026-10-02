"""DC motor model and a small PID used to emulate onboard closed-loop control."""

from dataclasses import dataclass, fields
import math


@dataclass
class DcMotorModel:
    """
    Brushed/brushless DC motor constants at the motor shaft.

    Defaults are a Kraken X60 on a 12 V bus.
    """

    nominal_voltage: float = 12.0
    free_speed_rpm: float = 6000.0
    stall_torque: float = 9.37
    stall_current: float = 483.0
    free_current: float = 3.6
    stator_current_limit: float = 120.0
    supply_current_limit: float = 70.0

    @property
    def free_speed(self):
        """No-load speed in rad/s at the motor shaft."""
        return self.free_speed_rpm * 2.0 * math.pi / 60.0

    @property
    def torque_constant(self):
        """N*m per amp."""
        return self.stall_torque / self.stall_current

    @property
    def resistance(self):
        """Terminal resistance in ohms, estimated from stall conditions."""
        return self.nominal_voltage / self.stall_current

    @classmethod
    def from_dict(cls, data):
        known = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in data.items() if k in known})

    def _limit(self, torque):
        limit = self.torque_constant * self.stator_current_limit
        return max(-limit, min(limit, torque))

    def apply_voltage(self, voltage, speed):
        """
        Return ``(motor_torque, stator_current)`` for a voltage and shaft speed.

        ``speed`` is the motor shaft speed in rad/s. No-load speed and stall
        torque scale linearly with applied voltage; the torque falls off
        linearly between them and is clamped by the stator current limit.
        """
        voltage = max(-self.nominal_voltage, min(self.nominal_voltage, voltage))
        if self.nominal_voltage == 0.0:
            return 0.0, 0.0
        ratio = voltage / self.nominal_voltage
        no_load_speed = self.free_speed * ratio
        if no_load_speed == 0.0:
            return 0.0, 0.0
        torque = self.stall_torque * ratio * (1.0 - speed / no_load_speed)
        torque = self._limit(torque)
        return torque, torque / self.torque_constant

    def brake_torque(self, speed):
        """Braking torque produced by shorting the phases (brake neutral mode)."""
        return self._limit(-(self.torque_constant ** 2 / self.resistance) * speed)

    def current_to_torque(self, current):
        """Torque for a requested stator current (torque-current control)."""
        return self._limit(self.torque_constant * current)


class Pid:
    """Minimal PID with output and integral clamping."""

    def __init__(self, gains=(0.0, 0.0, 0.0), output_limit=1.0, integral_limit=1.0):
        self.kp, self.ki, self.kd = gains
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self._integral = 0.0
        self._prev_error = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None

    def __call__(self, error, dt):
        if dt <= 0.0:
            return 0.0
        self._integral = max(
            -self.integral_limit,
            min(self.integral_limit, self._integral + error * dt),
        )
        derivative = 0.0
        if self._prev_error is not None:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))
