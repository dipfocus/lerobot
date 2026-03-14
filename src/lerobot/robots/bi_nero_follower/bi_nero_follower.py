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

from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.nero import Nero, NeroConfig
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_bi_nero_follower import BiNeroFollowerConfig

logger = logging.getLogger(__name__)


class BiNeroFollower(Robot):
    """Bimanual NERO follower arms composed from two single-arm NERO instances."""

    config_class = BiNeroFollowerConfig
    name = "bi_nero_follower"

    def __init__(self, config: BiNeroFollowerConfig):
        super().__init__(config)
        self.config = config

        left_arm_config = NeroConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            channel=config.left_arm_config.channel,
            interface=config.left_arm_config.interface,
            bitrate=config.left_arm_config.bitrate,
            enable_check_can=config.left_arm_config.enable_check_can,
            auto_connect=config.left_arm_config.auto_connect,
            timeout=config.left_arm_config.timeout,
            firmware_version=config.left_arm_config.firmware_version,
            speed_percent=config.left_arm_config.speed_percent,
            start_read_thread=config.left_arm_config.start_read_thread,
            joint_command_mode=config.left_arm_config.joint_command_mode,
            max_relative_target=config.left_arm_config.max_relative_target,
            effector=config.left_arm_config.effector,
            gripper_force=config.left_arm_config.gripper_force,
            disable_gripper_on_disconnect=config.left_arm_config.disable_gripper_on_disconnect,
            observation_timestamp_skew_error_s=config.left_arm_config.observation_timestamp_skew_error_s,
            observation_timestamp_skew_error_interval_s=(
                config.left_arm_config.observation_timestamp_skew_error_interval_s
            ),
            cameras=config.left_arm_config.cameras,
        )

        right_arm_config = NeroConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            channel=config.right_arm_config.channel,
            interface=config.right_arm_config.interface,
            bitrate=config.right_arm_config.bitrate,
            enable_check_can=config.right_arm_config.enable_check_can,
            auto_connect=config.right_arm_config.auto_connect,
            timeout=config.right_arm_config.timeout,
            firmware_version=config.right_arm_config.firmware_version,
            speed_percent=config.right_arm_config.speed_percent,
            start_read_thread=config.right_arm_config.start_read_thread,
            joint_command_mode=config.right_arm_config.joint_command_mode,
            max_relative_target=config.right_arm_config.max_relative_target,
            effector=config.right_arm_config.effector,
            gripper_force=config.right_arm_config.gripper_force,
            disable_gripper_on_disconnect=config.right_arm_config.disable_gripper_on_disconnect,
            observation_timestamp_skew_error_s=config.right_arm_config.observation_timestamp_skew_error_s,
            observation_timestamp_skew_error_interval_s=(
                config.right_arm_config.observation_timestamp_skew_error_interval_s
            ),
            cameras=config.right_arm_config.cameras,
        )

        self.left_arm = Nero(left_arm_config)
        self.right_arm = Nero(right_arm_config)
        self.cameras = {
            **self._prefix_mapping(self.left_arm.cameras, "left_"),
            **self._prefix_mapping(self.right_arm.cameras, "right_"),
        }

    @staticmethod
    def _prefix_mapping(values: dict[str, Any], prefix: str) -> dict[str, Any]:
        return {f"{prefix}{key}": value for key, value in values.items()}

    def _cleanup_after_connect_error(self) -> None:
        for arm, name in ((self.right_arm, "right"), (self.left_arm, "left")):
            if arm.is_connected:
                try:
                    arm.disconnect()
                except Exception:
                    logger.exception("Failed to disconnect %s NERO arm after connect error", name)

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {
            **self._prefix_mapping(self.left_arm.observation_features, "left_"),
            **self._prefix_mapping(self.right_arm.observation_features, "right_"),
        }

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **self._prefix_mapping(self.left_arm.action_features, "left_"),
            **self._prefix_mapping(self.right_arm.action_features, "right_"),
        }

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
    def get_observation(self) -> RobotObservation:
        return {
            **self._prefix_mapping(self.left_arm.get_observation(), "left_"),
            **self._prefix_mapping(self.right_arm.get_observation(), "right_"),
        }

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        left_action = {
            key.removeprefix("left_"): value for key, value in action.items() if key.startswith("left_")
        }
        right_action = {
            key.removeprefix("right_"): value for key, value in action.items() if key.startswith("right_")
        }

        sent_action_left = self.left_arm.send_action(left_action)
        sent_action_right = self.right_arm.send_action(right_action)

        return {
            **self._prefix_mapping(sent_action_left, "left_"),
            **self._prefix_mapping(sent_action_right, "right_"),
        }

    def disconnect(self) -> None:
        errors = []
        for arm, name in ((self.right_arm, "right"), (self.left_arm, "left")):
            if arm.is_connected:
                try:
                    arm.disconnect()
                except Exception as exc:
                    logger.exception("Failed to disconnect %s NERO arm", name)
                    errors.append(exc)

        if errors:
            raise errors[0]
