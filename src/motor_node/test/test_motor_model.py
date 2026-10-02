# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from motor_node.motor_model import DcMotorModel, Pid


def test_free_speed_and_constants():
    model = DcMotorModel()
    assert math.isclose(model.free_speed, 628.3185, rel_tol=1e-4)
    assert math.isclose(model.torque_constant, 9.37 / 483.0, rel_tol=1e-9)


def test_stall_torque_is_current_limited():
    model = DcMotorModel(stator_current_limit=100.0)
    torque, current = model.apply_voltage(12.0, 0.0)
    expected = model.torque_constant * 100.0
    assert math.isclose(torque, expected, rel_tol=1e-9)
    assert math.isclose(current, 100.0, rel_tol=1e-9)


def test_no_load_speed_has_no_torque():
    model = DcMotorModel(stator_current_limit=483.0)
    torque, _ = model.apply_voltage(12.0, model.free_speed)
    assert abs(torque) < 1e-6


def test_zero_voltage_has_no_torque():
    model = DcMotorModel()
    assert model.apply_voltage(0.0, 100.0) == (0.0, 0.0)


def test_brake_torque_opposes_motion():
    model = DcMotorModel()
    assert model.brake_torque(100.0) < 0.0
    assert model.brake_torque(-100.0) > 0.0


def test_current_to_torque_clamps():
    model = DcMotorModel(stator_current_limit=100.0)
    torque = model.current_to_torque(1000.0)
    assert math.isclose(torque, model.torque_constant * 100.0, rel_tol=1e-9)


def test_pid_clamps_output():
    pid = Pid((1.0, 0.0, 0.0), output_limit=5.0, integral_limit=5.0)
    assert pid(100.0, 0.1) == 5.0
    assert pid(-100.0, 0.1) == -5.0
