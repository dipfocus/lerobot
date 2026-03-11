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
  --robot.cameras='{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
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
- `record.py`: teleoperate while recording a LeRobot dataset with an optional OpenCV camera.

## Before You Run

- Install the vendor `pyAgxArm` SDK first. `Nero` and `NeroLeader` will raise an import error without it.
- Put the follower and leader on separate CAN channels, then update `FOLLOWER_CHANNEL` and `LEADER_CHANNEL`.
- Keep `EFFECTOR` aligned between the two scripts. If both arms have the AGX gripper, set it to `"agx_gripper"`. Otherwise leave it at `"none"`.
- For `record.py`, update `CAMERA_INDEX` or replace `camera_config` with an empty dict if you do not want to record vision.
- `HF_REPO_ID` is still required for local datasets because LeRobot stores the recording under a repo-shaped path. `record.py` keeps `PUSH_TO_HUB = False` by default so you can validate the flow locally first.

## Run The Python Examples

```bash
python examples/nero/teleoperate.py
python examples/nero/record.py
```

## Notes

- Start with the follower arm physically close to the leader arm. The example enables a conservative `max_relative_target` to reduce large jumps, but it is still a direct joint-space mapping.
- In `record.py`, press `Esc` to stop recording and `Left Arrow` to discard and re-record the current episode.
