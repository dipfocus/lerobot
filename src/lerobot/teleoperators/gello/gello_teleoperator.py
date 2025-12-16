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

import logging
from typing import Any

import numpy as np
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from .config_gello import (
    DEFAULT_GELLO_JOINT_NAMES,
    GelloTeleoperatorConfig,
)

logger = logging.getLogger(__name__)


class GelloTeleoperator(Teleoperator):
    """
    Teleoperator for the GELLO wearable arm.

    This wraps the `GelloAgent` from the `gello_software` project to expose joint
    angles as a standard LeRobot teleoperator.
    """

    config_class = GelloTeleoperatorConfig
    name = "gello"

    def __init__(self, config: GelloTeleoperatorConfig):
        super().__init__(config)
        self.config = config
        self._agent: Any | None = None
        self._initial_joints: np.ndarray | None = None
        self._joint_names = tuple(config.joint_names) if config.joint_names else DEFAULT_GELLO_JOINT_NAMES

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in self._joint_names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._agent is not None

    def _import_gello_agent(self):
        try:
            from gello.agents.gello_agent import GelloAgent  # type: ignore
        except ImportError as e:
            raise ImportError(
                "Could not import GelloAgent. Please install gello_software in your environment."
            ) from e
        return GelloAgent

    def connect(self, calibrate: bool = True) -> None:
        del calibrate  # Calibration is not handled by this wrapper

        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        GelloAgent = self._import_gello_agent()
        self._agent = GelloAgent(port=self.config.port)

        # Cache initial joints for logging/diagnostics
        try:
            self._initial_joints = self._read_raw_joints()
            logger.info("Connected to GELLO on %s. Initial joints: %s", self.config.port, self._initial_joints)
        except Exception as exc:  # pragma: no cover - hardware dependent
            logger.warning("GELLO connected but failed to read initial joints: %s", exc)

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        # Calibration is handled by the GELLO firmware/software itself.
        pass

    def configure(self) -> None:
        # No additional configuration required.
        pass

    def _read_raw_joints(self) -> np.ndarray:
        if self._agent is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        joints = np.asarray(self._agent.act({}), dtype=float).squeeze()
        if joints.ndim == 0:
            joints = np.array([float(joints)], dtype=float)
        return joints

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        joints = self._read_raw_joints()
        expected = len(self._joint_names)
        if joints.shape[0] < expected:
            raise RuntimeError(f"Expected at least {expected} joints from GELLO, got {joints.shape[0]}")

        joints = joints[:expected].astype(float, copy=True)

        # Convert the last joint into a binary button if requested
        if self.config.button_threshold is not None and expected > 0:
            raw_gripper = joints[-1]
            joints[-1] = 1.0 if raw_gripper >= self.config.button_threshold else 0.0

        return {f"{name}.pos": float(val) for name, val in zip(self._joint_names, joints, strict=False)}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError("Feedback is not supported for GELLO teleoperation.")

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        try:
            close = getattr(self._agent, "close", None)
            if callable(close):
                close()
        finally:
            self._agent = None
            logger.info("Disconnected from GELLO.")
