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

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.import_utils import _pyagxarm_available

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_nero import NeroConfig

if TYPE_CHECKING or _pyagxarm_available:
    from pyAgxArm import AgxArmFactory, create_agx_arm_config
else:
    AgxArmFactory = None
    create_agx_arm_config = None

logger = logging.getLogger(__name__)

NERO_JOINTS = tuple(f"joint{i}" for i in range(1, 8))
NERO_GRIPPER_MAX_WIDTH_M = 0.1


class Nero(Robot):
    """NERO 7-DOF arm driven through the vendor `pyAgxArm` CAN SDK."""

    config_class = NeroConfig
    name = "nero"

    def __init__(self, config: NeroConfig):
        super().__init__(config)
        self.config = config
        self.arm = None
        self.end_effector = None
        self.cameras = make_cameras_from_configs(config.cameras)
        self._is_connected = False
        self._last_joint_positions: list[float] | None = None
        self._last_gripper_width: float = 0.0

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {
            **self.motor_features,
            "joints.timestamp": float,
            **{f"{cam}.timestamp": float for cam in self.cameras},
            **self.camera_features,
        }

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self.motor_features

    @property
    def motor_features(self) -> dict[str, type]:
        features = {f"{joint}.pos": float for joint in NERO_JOINTS}
        if self.config.effector == "agx_gripper":
            features["gripper.pos"] = float
        return features

    @property
    def camera_features(self) -> dict[str, tuple[int | None, int | None, int]]:
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    @property
    def is_connected(self) -> bool:
        return self._is_connected and all(cam.is_connected for cam in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        # The vendor SDK exposes enable/homing flows rather than motor calibration.
        return True

    def _require_sdk(self) -> None:
        if create_agx_arm_config is None or AgxArmFactory is None:
            raise ImportError(
                "NERO support requires the optional `pyAgxArm` package. "
                "Install the vendor SDK first, then retry."
            )

    def _wait_until_enabled(self) -> None:
        deadline = time.monotonic() + self.config.timeout
        while True:
            if self.arm.enable():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out enabling NERO after {self.config.timeout}s")
            time.sleep(0.05)

    def _wait_for_joint_feedback(self) -> None:
        deadline = time.monotonic() + self.config.timeout
        while True:
            try:
                self._last_joint_positions = self._read_joint_positions()
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Timed out waiting for NERO joint feedback after {self.config.timeout}s")
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
                        f"Timed out waiting for NERO gripper feedback after {self.config.timeout}s"
                    )
                time.sleep(0.05)

    def connect(self, calibrate: bool = True) -> None:
        del calibrate

        if self.is_connected:
            return

        try:
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

            self.arm = AgxArmFactory.create_arm(cfg)
            if self.arm is None:
                raise RuntimeError("AgxArmFactory.create_arm returned None for NERO")

            if self.config.effector == "agx_gripper":
                self.end_effector = self.arm.init_effector(self.arm.OPTIONS.EFFECTOR.AGX_GRIPPER)
                if self.end_effector is None:
                    raise RuntimeError("Failed to initialize AGX gripper effector for NERO")

            self.arm.connect(start_read_thread=self.config.start_read_thread)

            for cam in self.cameras.values():
                cam.connect()

            self.configure()
            self._wait_until_enabled()
            self._wait_for_joint_feedback()
            self._wait_for_gripper_feedback()
            self._is_connected = True
            logger.info("%s connected on %s", self.name, self.config.channel)
        except Exception:
            for cam in self.cameras.values():
                try:
                    cam.disconnect()
                except Exception:
                    logger.exception("Failed to disconnect NERO camera after connect error")
            self.arm = None
            self.end_effector = None
            self._is_connected = False
            raise

    def calibrate(self) -> None:
        logger.info("NERO does not require LeRobot-side calibration; skipping.")

    def configure(self) -> None:
        if self.arm is None:
            return

        self.arm.set_normal_mode()
        if self.config.joint_command_mode == "j":
            self.arm.set_motion_mode("j")
            self.arm.set_speed_percent(self.config.speed_percent)

    def _read_joint_positions(self) -> list[float]:
        joint_msg = self.arm.get_joint_angles()
        if joint_msg is None:
            if self._last_joint_positions is None:
                raise RuntimeError("NERO joint feedback unavailable")
            logger.warning("Falling back to last known NERO joint positions")
            return list(self._last_joint_positions)

        joint_positions = [float(value) for value in joint_msg.msg]
        if len(joint_positions) != len(NERO_JOINTS):
            raise RuntimeError(f"Expected {len(NERO_JOINTS)} joints, got {len(joint_positions)}")
        self._last_joint_positions = joint_positions
        return list(joint_positions)

    def _read_gripper_width(self) -> float:
        if self.end_effector is None:
            raise RuntimeError("NERO gripper is not configured")

        gripper_msg = self.end_effector.get_gripper_status()
        if gripper_msg is None:
            raise RuntimeError("NERO gripper feedback unavailable")

        self._last_gripper_width = float(gripper_msg.msg.width)
        return self._last_gripper_width

    def get_observation(self) -> RobotObservation:
        if not self.is_connected:
            raise RuntimeError("NERO is not connected")

        joint_positions = self._read_joint_positions()
        joint_timestamp = time.perf_counter()
        obs: RobotObservation = {f"{joint}.pos": value for joint, value in zip(NERO_JOINTS, joint_positions, strict=True)}
        obs["joints.timestamp"] = joint_timestamp

        if self.config.effector == "agx_gripper":
            try:
                obs["gripper.pos"] = self._read_gripper_width()
            except RuntimeError:
                obs["gripper.pos"] = self._last_gripper_width

        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()
            cam_timestamp = getattr(cam, "latest_timestamp", None)
            obs[f"{cam_key}.timestamp"] = (
                float(cam_timestamp) if cam_timestamp is not None else time.perf_counter()
            )

        return obs

    def _build_joint_targets(self, action: RobotAction) -> dict[str, float]:
        current_positions = dict(zip(NERO_JOINTS, self._read_joint_positions(), strict=True))
        target_positions = current_positions.copy()
        for joint in NERO_JOINTS:
            key = f"{joint}.pos"
            if key in action:
                target_positions[joint] = float(action[key])

        if self.config.max_relative_target is not None:
            goal_present_pos = {
                joint: (target_positions[joint], current_positions[joint]) for joint in NERO_JOINTS
            }
            target_positions = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        return target_positions

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self.is_connected:
            raise RuntimeError("NERO is not connected")

        sent_action: RobotAction = {}
        has_joint_action = any(f"{joint}.pos" in action for joint in NERO_JOINTS)
        if has_joint_action:
            target_positions = self._build_joint_targets(action)
            target_list = [target_positions[joint] for joint in NERO_JOINTS]

            if self.config.joint_command_mode == "js":
                self.arm.move_js(target_list)
            else:
                self.arm.move_j(target_list)
            self._last_joint_positions = target_list

            sent_action = {f"{joint}.pos": target_positions[joint] for joint in NERO_JOINTS}

        if self.config.effector == "agx_gripper" and "gripper.pos" in action:
            if self.end_effector is None:
                raise RuntimeError("NERO gripper command requested but no gripper is configured")

            width = max(0.0, min(float(action["gripper.pos"]), NERO_GRIPPER_MAX_WIDTH_M))
            self.end_effector.move_gripper(width=width, force=self.config.gripper_force)
            self._last_gripper_width = width
            sent_action["gripper.pos"] = width

        return sent_action

    def disconnect(self) -> None:
        if self.end_effector is not None and self.config.disable_gripper_on_disconnect:
            try:
                self.end_effector.disable_gripper()
            except Exception:
                logger.exception("Failed to disable NERO gripper during disconnect")

        if self.arm is not None and self.config.disable_arm_on_disconnect:
            deadline = time.monotonic() + self.config.timeout
            while True:
                try:
                    if self.arm.disable():
                        break
                except Exception:
                    logger.exception("Failed to disable NERO arm during disconnect")
                    break
                if time.monotonic() >= deadline:
                    logger.warning("Timed out disabling NERO arm during disconnect")
                    break
                time.sleep(0.05)

        for cam in self.cameras.values():
            cam.disconnect()

        self.arm = None
        self.end_effector = None
        self._is_connected = False
        logger.info("%s disconnected", self.name)
