"""Load motor definitions from a YAML file."""

from dataclasses import dataclass, field

import yaml

from .motor_model import DcMotorModel
from .transport import MotorConfig, NeutralMode

COMMAND_TOPIC = '/g2/motors/{name}/command'
STATE_TOPIC = '/g2/motors/{name}/state'
EFFORT_TOPIC = '/g2/joint_effort/{joint}'


@dataclass
class MotorSet:
    transport: str = 'sim'
    joint_states_topic: str = '/joint_states'
    update_rate: float = 100.0
    state_rate: float = 50.0
    motors: list = field(default_factory=list)


def _topic(value, template, **kwargs):
    return value if value else template.format(**kwargs)


def load_motors(path):
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}

    defaults = dict(data.get('defaults', {}))
    presets = dict(data.get('motor_presets', {}))
    motors = []
    for entry in data.get('motors', []):
        merged = dict(defaults)
        merged.update(entry)
        name = merged['name']
        joint = merged.get('joint', name)
        preset = dict(presets.get(merged.get('motor'), {}))
        preset.update(merged.get('motor_overrides', {}))
        config = MotorConfig(
            name=name,
            joint=joint,
            motor=DcMotorModel.from_dict(preset),
            gear_ratio=float(merged.get('gear_ratio', 1.0)),
            efficiency=float(merged.get('efficiency', 1.0)),
            inverted=bool(merged.get('inverted', False)),
            neutral_mode=NeutralMode(str(merged.get('neutral_mode', 'brake')).lower()),
            command_topic=_topic(
                merged.get('command_topic'), COMMAND_TOPIC, name=name),
            state_topic=_topic(
                merged.get('state_topic'), STATE_TOPIC, name=name),
            effort_topic=_topic(
                merged.get('effort_topic'), EFFORT_TOPIC, joint=joint),
            can_id=int(merged.get('can_id', -1)),
            can_bus=str(merged.get('can_bus', '')),
            velocity_gains=tuple(
                float(g) for g in merged.get('velocity_gains', (0.0, 0.0, 0.0))),
            position_gains=tuple(
                float(g) for g in merged.get('position_gains', (0.0, 0.0, 0.0))),
        )
        motors.append(config)

    return MotorSet(
        transport=str(data.get('transport', 'sim')).lower(),
        joint_states_topic=str(data.get('joint_states_topic', '/joint_states')),
        update_rate=float(data.get('update_rate', 100.0)),
        state_rate=float(data.get('state_rate', 50.0)),
        motors=motors,
    )
