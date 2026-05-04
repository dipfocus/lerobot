#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

from lerobot.cameras.configs import CameraConfig

from ..config import RobotConfig


def eggbot_cameras_config() -> dict[str, CameraConfig]:
    return {
        # Example:
        # "front": OpenCVCameraConfig(
        #     index_or_path="/dev/video0",
        #     fps=30,
        #     width=640,
        #     height=480,
        #     rotation=Cv2Rotation.NO_ROTATION,
        # ),
    }


@RobotConfig.register_subclass("eggbot")
@dataclass
class EggbotConfig(RobotConfig):
    port: str = "/dev/ttyACM0"

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the relative positional target for safety.
    # It can be a single scalar or a dictionary keyed by arm motor name.
    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=eggbot_cameras_config)

    # Set to `True` to use degree-normalized arm motor positions.
    use_degrees: bool = False

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            "forward": "i",
            "backward": "k",
            "left": "j",
            "right": "l",
            "rotate_left": "u",
            "rotate_right": "o",
            "speed_up": "n",
            "speed_down": "m",
            "quit": "b",
        }
    )
