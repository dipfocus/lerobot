#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

import numpy as np
import pytest

from lerobot.teleoperators.gello import GelloTeleoperator, GelloTeleoperatorConfig


class DummyGelloAgent:
    def __init__(self, joints):
        self._joints = np.asarray(joints, dtype=float)

    def act(self, _):
        return self._joints


def test_get_action_binarizes_last_joint():
    cfg = GelloTeleoperatorConfig(port="dummy", button_threshold=0.5)
    teleop = GelloTeleoperator(cfg)

    teleop._agent = DummyGelloAgent([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8])
    action = teleop.get_action()
    assert action["gripper.pos"] == 1.0

    teleop._agent = DummyGelloAgent([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.2])
    action = teleop.get_action()
    assert action["gripper.pos"] == 0.0


def test_get_action_respects_joint_names_and_length():
    cfg = GelloTeleoperatorConfig(port="dummy", joint_names=("a", "b", "c"), button_threshold=None)
    teleop = GelloTeleoperator(cfg)

    teleop._agent = DummyGelloAgent([1.0, 2.0, 3.0, 4.0])
    action = teleop.get_action()
    assert set(action.keys()) == {"a.pos", "b.pos", "c.pos"}
    assert action["a.pos"] == 1.0
    assert action["b.pos"] == 2.0
    assert action["c.pos"] == 3.0


def test_get_action_raises_if_insufficient_joints():
    cfg = GelloTeleoperatorConfig(port="dummy")
    teleop = GelloTeleoperator(cfg)
    teleop._agent = DummyGelloAgent([0.1, 0.2])

    with pytest.raises(RuntimeError):
        _ = teleop.get_action()

