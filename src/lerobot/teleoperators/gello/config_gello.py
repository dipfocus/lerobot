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

from dataclasses import dataclass

from ..config import TeleoperatorConfig

# Matches the default port used in the standalone gello_sender.py script
DEFAULT_GELLO_JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
    "gripper",
)


@TeleoperatorConfig.register_subclass("gello")
@dataclass
class GelloTeleoperatorConfig(TeleoperatorConfig):
    # Serial port to the GELLO hardware
    port: str
    # Threshold applied to the last joint to convert it into a binary button (0/1). Set to None to disable.
    button_threshold: float | None = 0.5
    # Names used for the 7 arm joints and the gripper/button
    joint_names: tuple[str, ...] = DEFAULT_GELLO_JOINT_NAMES
