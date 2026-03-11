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

import time

from lerobot.robots.nero import Nero, NeroConfig
from lerobot.teleoperators.nero_leader import NeroLeader, NeroLeaderConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

FPS = 30

# Update these constants for your setup before running the example.
FOLLOWER_CHANNEL = "can0"
LEADER_CHANNEL = "can1"
EFFECTOR = "none"  # Set to "agx_gripper" if both arms have the AGX gripper installed.
FOLLOWER_SPEED_PERCENT = 10
FOLLOWER_MAX_RELATIVE_TARGET = 0.1


def main():
    robot_config = NeroConfig(
        channel=FOLLOWER_CHANNEL,
        effector=EFFECTOR,
        speed_percent=FOLLOWER_SPEED_PERCENT,
        max_relative_target=FOLLOWER_MAX_RELATIVE_TARGET,
    )
    teleop_config = NeroLeaderConfig(
        channel=LEADER_CHANNEL,
        effector=EFFECTOR,
    )

    robot = Nero(robot_config)
    teleop = NeroLeader(teleop_config)

    try:
        robot.connect()
        teleop.connect()

        init_rerun(session_name="nero_teleop")

        if not robot.is_connected or not teleop.is_connected:
            raise ValueError("Robot or teleop is not connected!")

        print("Starting teleop loop. Align the follower arm before moving the leader arm...")
        while True:
            t0 = time.perf_counter()

            observation = robot.get_observation()
            action = teleop.get_action()
            _ = robot.send_action(action)

            log_rerun_data(observation=observation, action=action)
            precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
    finally:
        if teleop.is_connected:
            teleop.disconnect()
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
