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

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.processor import make_default_processors
from lerobot.robots.nero import Nero, NeroConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.nero_leader import NeroLeader, NeroLeaderConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun

NUM_EPISODES = 2
FPS = 30
EPISODE_TIME_SEC = 30
RESET_TIME_SEC = 15
TASK_DESCRIPTION = "Move the object to the target area."
HF_REPO_ID = "<hf_username>/<dataset_repo_id>"
PUSH_TO_HUB = False

# Update these constants for your setup before running the example.
FOLLOWER_CHANNEL = "can0"
LEADER_CHANNEL = "can1"
EFFECTOR = "none"  # Set to "agx_gripper" if both arms have the AGX gripper installed.
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FOLLOWER_SPEED_PERCENT = 10
FOLLOWER_MAX_RELATIVE_TARGET = 0.1


def main():
    camera_config = {
        "front": OpenCVCameraConfig(
            index_or_path=CAMERA_INDEX,
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
            fps=FPS,
        )
    }
    robot_config = NeroConfig(
        channel=FOLLOWER_CHANNEL,
        effector=EFFECTOR,
        cameras=camera_config,
        speed_percent=FOLLOWER_SPEED_PERCENT,
        max_relative_target=FOLLOWER_MAX_RELATIVE_TARGET,
    )
    teleop_config = NeroLeaderConfig(
        channel=LEADER_CHANNEL,
        effector=EFFECTOR,
    )

    robot = Nero(robot_config)
    teleop = NeroLeader(teleop_config)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}

    dataset = LeRobotDataset.create(
        repo_id=HF_REPO_ID,
        fps=FPS,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4 * len(robot.cameras),
    )

    listener = None
    events = None

    try:
        robot.connect()
        teleop.connect()

        listener, events = init_keyboard_listener()
        init_rerun(session_name="nero_record")

        if not robot.is_connected or not teleop.is_connected:
            raise ValueError("Robot or teleop is not connected!")

        print("Starting record loop...")
        episode_idx = 0
        while episode_idx < NUM_EPISODES and not events["stop_recording"]:
            log_say(f"Recording episode {episode_idx + 1} of {NUM_EPISODES}")

            record_loop(
                robot=robot,
                events=events,
                fps=FPS,
                teleop=teleop,
                dataset=dataset,
                control_time_s=EPISODE_TIME_SEC,
                single_task=TASK_DESCRIPTION,
                display_data=True,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
            )

            if not events["stop_recording"] and (
                episode_idx < NUM_EPISODES - 1 or events["rerecord_episode"]
            ):
                log_say("Reset the environment")
                record_loop(
                    robot=robot,
                    events=events,
                    fps=FPS,
                    teleop=teleop,
                    control_time_s=RESET_TIME_SEC,
                    single_task=TASK_DESCRIPTION,
                    display_data=True,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                )

            if events["rerecord_episode"]:
                log_say("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            episode_idx += 1
    finally:
        log_say("Stop recording")
        if teleop.is_connected:
            teleop.disconnect()
        if robot.is_connected:
            robot.disconnect()
        if listener is not None:
            listener.stop()

        dataset.finalize()
        if PUSH_TO_HUB:
            dataset.push_to_hub()


if __name__ == "__main__":
    main()
