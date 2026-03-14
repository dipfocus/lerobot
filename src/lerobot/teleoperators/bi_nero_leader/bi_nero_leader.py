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

import logging
from functools import cached_property
from typing import Any

from lerobot.processor import RobotAction
from lerobot.teleoperators.nero_leader import NeroLeader, NeroLeaderConfig
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_bi_nero_leader import BiNeroLeaderConfig

logger = logging.getLogger(__name__)


class BiNeroLeader(Teleoperator):
    """Bimanual NERO leader teleoperator composed from two single-arm leaders."""

    config_class = BiNeroLeaderConfig
    name = "bi_nero_leader"

    def __init__(self, config: BiNeroLeaderConfig):
        super().__init__(config)
        self.config = config

        left_arm_config = NeroLeaderConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            channel=config.left_arm_config.channel,
            interface=config.left_arm_config.interface,
            bitrate=config.left_arm_config.bitrate,
            enable_check_can=config.left_arm_config.enable_check_can,
            auto_connect=config.left_arm_config.auto_connect,
            timeout=config.left_arm_config.timeout,
            firmware_version=config.left_arm_config.firmware_version,
            start_read_thread=config.left_arm_config.start_read_thread,
            effector=config.left_arm_config.effector,
            disable_gripper_on_disconnect=config.left_arm_config.disable_gripper_on_disconnect,
        )

        right_arm_config = NeroLeaderConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            channel=config.right_arm_config.channel,
            interface=config.right_arm_config.interface,
            bitrate=config.right_arm_config.bitrate,
            enable_check_can=config.right_arm_config.enable_check_can,
            auto_connect=config.right_arm_config.auto_connect,
            timeout=config.right_arm_config.timeout,
            firmware_version=config.right_arm_config.firmware_version,
            start_read_thread=config.right_arm_config.start_read_thread,
            effector=config.right_arm_config.effector,
            disable_gripper_on_disconnect=config.right_arm_config.disable_gripper_on_disconnect,
        )

        self.left_arm = NeroLeader(left_arm_config)
        self.right_arm = NeroLeader(right_arm_config)

    @staticmethod
    def _prefix_mapping(values: dict[str, Any], prefix: str) -> dict[str, Any]:
        return {f"{prefix}{key}": value for key, value in values.items()}

    def _cleanup_after_connect_error(self) -> None:
        for arm, name in ((self.right_arm, "right"), (self.left_arm, "left")):
            if arm.is_connected:
                try:
                    arm.disconnect()
                except Exception:
                    logger.exception("Failed to disconnect %s NERO leader after connect error", name)

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **self._prefix_mapping(self.left_arm.action_features, "left_"),
            **self._prefix_mapping(self.right_arm.action_features, "right_"),
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        try:
            self.left_arm.connect(calibrate)
            self.right_arm.connect(calibrate)
        except Exception:
            self._cleanup_after_connect_error()
            raise

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        return {
            **self._prefix_mapping(self.left_arm.get_action(), "left_"),
            **self._prefix_mapping(self.right_arm.get_action(), "right_"),
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError("Feedback is not implemented for bimanual NERO leader.")

    def disconnect(self) -> None:
        errors = []
        for arm, name in ((self.right_arm, "right"), (self.left_arm, "left")):
            if arm.is_connected:
                try:
                    arm.disconnect()
                except Exception as exc:
                    logger.exception("Failed to disconnect %s NERO leader", name)
                    errors.append(exc)

        if errors:
            raise errors[0]
