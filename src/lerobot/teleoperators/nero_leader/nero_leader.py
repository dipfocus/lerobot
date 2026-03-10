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

from __future__ import annotations

import logging
import time
from functools import cached_property
from typing import TYPE_CHECKING, Any

from lerobot.processor import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.import_utils import _pyagxarm_available

from ..teleoperator import Teleoperator
from .config_nero_leader import NeroLeaderConfig

if TYPE_CHECKING or _pyagxarm_available:
    from pyAgxArm import AgxArmFactory, create_agx_arm_config
else:
    AgxArmFactory = None
    create_agx_arm_config = None

logger = logging.getLogger(__name__)

NERO_JOINTS = tuple(f"joint{i}" for i in range(1, 8))


class NeroLeader(Teleoperator):
    """Read joint states from a NERO arm and expose them as teleoperation actions."""

    config_class = NeroLeaderConfig
    name = "nero_leader"

    def __init__(self, config: NeroLeaderConfig):
        super().__init__(config)
        self.config = config
        self.arm = None
        self.end_effector = None
        self._is_connected = False
        self._last_joint_positions: list[float] | None = None
        self._last_gripper_width: float = 0.0

    @cached_property
    def action_features(self) -> dict[str, type]:
        features = {f"{joint}.pos": float for joint in NERO_JOINTS}
        if self.config.effector == "agx_gripper":
            features["gripper.pos"] = float
        return features

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def _require_sdk(self) -> None:
        if create_agx_arm_config is None or AgxArmFactory is None:
            raise ImportError(
                "NERO leader support requires the optional `pyAgxArm` package. "
                "Install the vendor SDK first, then retry."
            )

    def _wait_until_enabled(self) -> None:
        deadline = time.monotonic() + self.config.timeout
        while True:
            if self.arm.enable():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out enabling NERO leader after {self.config.timeout}s")
            time.sleep(0.05)

    def _read_joint_positions(self) -> list[float]:
        joint_msg = self.arm.get_joint_angles()
        if joint_msg is None:
            if self._last_joint_positions is None:
                raise RuntimeError("NERO leader joint feedback unavailable")
            logger.warning("Falling back to last known NERO leader joint positions")
            return list(self._last_joint_positions)

        joint_positions = [float(value) for value in joint_msg.msg]
        if len(joint_positions) != len(NERO_JOINTS):
            raise RuntimeError(f"Expected {len(NERO_JOINTS)} joints, got {len(joint_positions)}")

        self._last_joint_positions = joint_positions
        return list(joint_positions)

    def _read_gripper_width(self) -> float:
        if self.end_effector is None:
            raise RuntimeError("NERO leader gripper is not configured")

        gripper_msg = self.end_effector.get_gripper_status()
        if gripper_msg is None:
            raise RuntimeError("NERO leader gripper feedback unavailable")

        self._last_gripper_width = float(gripper_msg.msg.width)
        return self._last_gripper_width

    def _wait_for_joint_feedback(self) -> None:
        deadline = time.monotonic() + self.config.timeout
        while True:
            try:
                self._last_joint_positions = self._read_joint_positions()
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f"Timed out waiting for NERO leader joint feedback after {self.config.timeout}s"
                    )
                time.sleep(0.05)

    def _wait_for_gripper_feedback(self) -> None:
        if self.config.effector != "agx_gripper":
            return

        deadline = time.monotonic() + self.config.timeout
        while True:
            try:
                self._last_gripper_width = self._read_gripper_width()
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f"Timed out waiting for NERO leader gripper feedback after {self.config.timeout}s"
                    )
                time.sleep(0.05)

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate

        self._require_sdk()
        cfg = create_agx_arm_config(
            robot="nero",
            comm="can",
            firmeware_version=self.config.firmware_version,
            channel=self.config.channel,
            interface=self.config.interface,
            bitrate=self.config.bitrate,
            enable_check_can=self.config.enable_check_can,
            auto_connect=self.config.auto_connect,
            timeout=self.config.timeout,
        )
        if not isinstance(cfg, dict):
            raise RuntimeError(f"create_agx_arm_config returned unexpected type: {type(cfg).__name__}")

        arm = AgxArmFactory.create_arm(cfg)
        if arm is None:
            raise RuntimeError("AgxArmFactory.create_arm returned None for NERO leader")

        end_effector = None
        if self.config.effector == "agx_gripper":
            end_effector = arm.init_effector(arm.OPTIONS.EFFECTOR.AGX_GRIPPER)
            if end_effector is None:
                raise RuntimeError("Failed to initialize AGX gripper effector for NERO leader")

        try:
            arm.connect(start_read_thread=self.config.start_read_thread)
            self.arm = arm
            self.end_effector = end_effector
            self.configure()
            self._wait_until_enabled()
            self._wait_for_joint_feedback()
            self._wait_for_gripper_feedback()
            self._is_connected = True
            logger.info("%s connected on %s", self.name, self.config.channel)
        except Exception:
            self.arm = None
            self.end_effector = None
            self._is_connected = False
            raise

    @check_if_not_connected
    def calibrate(self) -> None:
        logger.info("NERO leader does not require LeRobot-side calibration; skipping.")

    def configure(self) -> None:
        if self.arm is not None:
            self.arm.set_normal_mode()

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        joint_positions = self._read_joint_positions()
        action: RobotAction = {
            f"{joint}.pos": value for joint, value in zip(NERO_JOINTS, joint_positions, strict=True)
        }

        if self.config.effector == "agx_gripper":
            try:
                action["gripper.pos"] = self._read_gripper_width()
            except RuntimeError:
                action["gripper.pos"] = self._last_gripper_width

        return action

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError("Feedback is not implemented for NERO leader.")

    @check_if_not_connected
    def disconnect(self) -> None:
        if self.end_effector is not None and self.config.disable_gripper_on_disconnect:
            try:
                self.end_effector.disable_gripper()
            except Exception:
                logger.exception("Failed to disable NERO leader gripper during disconnect")

        if self.arm is not None and self.config.disable_arm_on_disconnect:
            deadline = time.monotonic() + self.config.timeout
            while True:
                try:
                    if self.arm.disable():
                        break
                except Exception:
                    logger.exception("Failed to disable NERO leader arm during disconnect")
                    break
                if time.monotonic() >= deadline:
                    logger.warning("Timed out disabling NERO leader arm during disconnect")
                    break
                time.sleep(0.05)

        self.arm = None
        self.end_effector = None
        self._is_connected = False
        logger.info("%s disconnected", self.name)
