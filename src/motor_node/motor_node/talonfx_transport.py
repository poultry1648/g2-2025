"""
Real-robot motor backend for CTRE TalonFX over CAN.

This module is the only place that knows about vendor hardware. It is imported
lazily so the simulator never needs the Phoenix 6 Python package installed.
"""

from .transport import ControlMode, MotorState, MotorTransport, NeutralMode


class TalonFXTransport(MotorTransport):
    def __init__(self, config):
        super().__init__(config)
        from phoenix6 import controls, hardware, signals

        self._controls = controls
        self._signals = signals
        self._talon = hardware.TalonFX(config.can_id, config.can_bus)
        self._configure(hardware, signals)
        self._mode = ControlMode.DUTY_CYCLE
        self._enabled = False
        self._state = MotorState()

    def _configure(self, hardware, signals):
        from phoenix6 import configs

        cfg = configs.TalonFXConfiguration()
        cfg.motor_output.inverted = (
            signals.InvertedValue.CLOCKWISE_POSITIVE if self.config.inverted
            else signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
        cfg.motor_output.neutral_mode = (
            signals.NeutralModeValue.BRAKE
            if self.config.neutral_mode == NeutralMode.BRAKE
            else signals.NeutralModeValue.COAST)
        cfg.current_limits.stator_current_limit = \
            self.config.motor.stator_current_limit
        cfg.current_limits.stator_current_limit_enable = True
        cfg.current_limits.supply_current_limit = \
            self.config.motor.supply_current_limit
        cfg.current_limits.supply_current_limit_enable = True
        cfg.feedback.sensor_to_mechanism_ratio = self.config.gear_ratio
        cfg.slot0.k_p, cfg.slot0.k_i, cfg.slot0.k_d = self.config.velocity_gains
        self._talon.configurator.apply(cfg)

    def set_command(self, mode, target, feedforward, enabled):
        self._mode = ControlMode(mode)
        self._enabled = enabled
        controls = self._controls
        if not enabled:
            self._talon.set_control(controls.NeutralOut())
            return
        if self._mode == ControlMode.DUTY_CYCLE:
            request = controls.DutyCycleOut(target)
        elif self._mode == ControlMode.VOLTAGE:
            request = controls.VoltageOut(target)
        elif self._mode == ControlMode.TORQUE_CURRENT:
            request = controls.TorqueCurrentFOC(target)
        elif self._mode == ControlMode.VELOCITY:
            request = controls.VelocityVoltage(target, feedforward)
        elif self._mode == ControlMode.POSITION:
            request = controls.PositionVoltage(target, feedforward)
        else:
            request = controls.NeutralOut()
        self._talon.set_control(request)

    def update(self, dt):
        talon = self._talon
        self._state = MotorState(
            position=talon.get_position().value,
            velocity=talon.get_velocity().value,
            stator_current=talon.get_stator_current().value,
            supply_current=talon.get_supply_current().value,
            applied_voltage=talon.get_motor_voltage().value,
            torque=talon.get_torque_current().value
            * self.config.motor.torque_constant * self.config.gear_ratio,
            mode=int(self._mode),
            enabled=self._enabled,
        )

    def state(self):
        return self._state

    def close(self):
        self._talon.set_control(self._controls.NeutralOut())
