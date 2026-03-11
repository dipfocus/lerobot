#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass, field
from typing import Literal

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("nero")
@dataclass
class NeroConfig(RobotConfig):
    channel: str = "can0"
    interface: str = "socketcan"
    bitrate: int = 1_000_000
    enable_check_can: bool = True
    auto_connect: bool = True
    timeout: float = 1.0

    firmware_version: str = "default"
    speed_percent: int = 10
    start_read_thread: bool = True
    joint_command_mode: Literal["j", "js"] = "j"

    # Set this to a positive scalar to cap per-joint target deltas in radians.
    max_relative_target: float | dict[str, float] | None = None

    # Optional single-DOF AGX gripper support.
    effector: Literal["none", "agx_gripper"] = "none"
    gripper_force: float = 1.0
    disable_gripper_on_disconnect: bool = False
    disable_arm_on_disconnect: bool = False

    # Optional timestamp skew monitoring between joint observations and camera frames.
    observation_timestamp_skew_error_s: float | None = None
    observation_timestamp_skew_error_interval_s: float = 5.0

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super().__post_init__()

        if not self.channel:
            raise ValueError("channel must be a non-empty string")
        if not 0 <= self.speed_percent <= 100:
            raise ValueError(f"speed_percent must be in [0, 100], got {self.speed_percent}")
        if self.timeout <= 0:
            raise ValueError(f"timeout must be > 0, got {self.timeout}")
        if not 0.0 <= self.gripper_force <= 3.0:
            raise ValueError(f"gripper_force must be in [0.0, 3.0], got {self.gripper_force}")
        if self.observation_timestamp_skew_error_s is not None and self.observation_timestamp_skew_error_s <= 0:
            raise ValueError(
                "observation_timestamp_skew_error_s must be > 0 when provided, "
                f"got {self.observation_timestamp_skew_error_s}"
            )
        if self.observation_timestamp_skew_error_interval_s <= 0:
            raise ValueError(
                "observation_timestamp_skew_error_interval_s must be > 0, "
                f"got {self.observation_timestamp_skew_error_interval_s}"
            )
