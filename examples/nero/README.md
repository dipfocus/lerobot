# NERO Examples

These examples are meant to be the shortest path for bringing up NERO teleoperation and data collection.

## Quick Start

If you just want to run NERO as quickly as possible, start with the CLI.

### Teleoperate

```bash
lerobot-teleoperate \
  --robot.type=nero \
  --robot.channel=can0 \
  --robot.effector=none \
  --robot.speed_percent=10 \
  --robot.max_relative_target=0.1 \
  --teleop.type=nero_leader \
  --teleop.channel=can1 \
  --teleop.effector=none \
  --fps=30 \
  --display_data=true
```

### Record

```bash
lerobot-record \
  --robot.type=nero \
  --robot.channel=can0 \
  --robot.effector=none \
  --robot.speed_percent=10 \
  --robot.max_relative_target=0.1 \
  --robot.cameras='{
    front: {type: intelrealsense, serial_number_or_name: "233522074606", width: 640, height: 480, fps: 30, use_depth: true},
    wrist: {type: intelrealsense, serial_number_or_name: "233522074607", width: 640, height: 480, fps: 30, use_depth: true}
  }' \
  --teleop.type=nero_leader \
  --teleop.channel=can1 \
  --teleop.effector=none \
  --dataset.repo_id=<hf_username>/<dataset_repo_id> \
  --dataset.num_episodes=2 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=15 \
  --dataset.single_task="Move the object to the target area." \
  --dataset.push_to_hub=false \
  --display_data=true
```

Use the Python examples below when you want a file you can edit directly instead of a long command.

## Files

- `teleoperate.py`: direct joint teleoperation from one NERO arm (`nero_leader`) to another (`nero`).
- `record.py`: teleoperate while recording a training-oriented dataset with front + wrist Intel RealSense D435i RGB-D streams.

## Before You Run

- Install the vendor `pyAgxArm` SDK first. `Nero` and `NeroLeader` will raise an import error without it.
- Install the RealSense dependency and verify your D435i is visible. `lerobot-find-cameras realsense` will list available serial numbers.
- Put the follower and leader on separate CAN channels, then update `FOLLOWER_CHANNEL` and `LEADER_CHANNEL`.
- Keep `EFFECTOR` aligned between the two scripts. If both arms have the AGX gripper, set it to `"agx_gripper"`. Otherwise leave it at `"none"`.
- For `record.py`, update `FRONT_REALSENSE_SERIAL_OR_NAME` and `WRIST_REALSENSE_SERIAL_OR_NAME`.
- `record.py` writes `meta/camera_setup.json` next to the dataset so the camera layout, intrinsics, and depth-to-color extrinsics are preserved with the recording.
- `record.py` now defaults to `local/<timestamp>` for the dataset path so each run lands in a fresh local batch directory.
- If you want to append to a fixed dataset path or push to the Hub later, replace `DATASET_REPO_ID` with your own stable repo-style name.

## Run The Python Examples

```bash
python examples/nero/teleoperate.py
python examples/nero/record.py
```

## Notes

- Start with the follower arm physically close to the leader arm. The example enables a conservative `max_relative_target` to reduce large jumps, but it is still a direct joint-space mapping.
- This example now records dual-view RGB-D observations, robot state, actions, and timestamps. RGB streams are stored as videos, while depth maps are stored as single-channel image features.
- Camera intrinsics and depth-to-color extrinsics are recorded in `meta/camera_setup.json`.
- Stock ACT / Diffusion / GR00T configs in LeRobot are still RGB-first. Using the depth channels in training will require custom feature selection and model input handling.
- In `record.py`, press `Esc` to stop recording and `Left Arrow` to discard and re-record the current episode.
