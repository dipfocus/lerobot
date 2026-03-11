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

from dataclasses import dataclass
from typing import Literal

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("nero_leader")
@dataclass
class NeroLeaderConfig(TeleoperatorConfig):
    channel: str = "can0"
    interface: str = "socketcan"
    bitrate: int = 1_000_000
    enable_check_can: bool = True
    auto_connect: bool = True
    timeout: float = 1.0

    firmware_version: str = "default"
    start_read_thread: bool = True

    effector: Literal["none", "agx_gripper"] = "none"
    disable_gripper_on_disconnect: bool = False

    def __post_init__(self) -> None:
        if not self.channel:
            raise ValueError("channel must be a non-empty string")
        if self.timeout <= 0:
            raise ValueError(f"timeout must be > 0, got {self.timeout}")
