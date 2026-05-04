#!/usr/bin/env python3
"""
YOLO target approach for an Eggbot with:
- one mobile base,
- one SO101 arm on the base bus,
- one camera mounted on the base.

Example:
    PYTHONPATH=src python examples/xlerobot/6_eggbot_yolo_base_follow.py \
        --port /dev/ttyACM1 \
        --camera 0 \
        --targets bottle
"""

from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from ultralytics import YOLO

import cv2
import numpy as np


from lerobot.cameras.configs import ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.eggbot import Eggbot, EggbotConfig

logger = logging.getLogger(__name__)

DEFAULT_ROBOT_ID = "eggbot_yolo_base_follow"
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_CAMERA_NAME = "base_camera"
DEFAULT_CAMERA = "0"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30
DEFAULT_TARGETS = "bottle"
DEFAULT_CONF = 0.35
DEFAULT_IMGSZ = 640
DEFAULT_CONTROL_HZ = 15.0
DEFAULT_CENTER_DEADBAND = 0.08
DEFAULT_FORWARD_ALIGNMENT = 0.35
DEFAULT_STOP_HEIGHT_RATIO = 0.45
DEFAULT_MIN_LINEAR = 0.03
DEFAULT_MAX_LINEAR = 0.12
DEFAULT_TURN_KP = 60.0
DEFAULT_MAX_ANGULAR = 45.0
DEFAULT_LOST_TIMEOUT = 0.4
DEFAULT_SEARCH_ANGULAR = 0.0
DEFAULT_MAX_RELATIVE_TARGET: int | None = None
DEFAULT_USE_DEGREES = False


@dataclass
class Detection:
    label: str
    confidence: float
    box_xyxy: tuple[int, int, int, int]
    center_xy: tuple[int, int]
    area_ratio: float
    height_ratio: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_camera_source(value: str) -> Any:
    if value.isdigit():
        return int(value)
    return value


def parse_targets(value: str) -> set[str] | None:
    if not value or value.strip().lower() == "any":
        return None
    targets = {item.strip().lower() for item in value.split(",") if item.strip()}
    return targets or None


def default_model_path() -> str:
    local_model = Path(__file__).with_name("yolo11x.pt")
    return str(local_model if local_model.exists() else "yolo11x.pt")


def build_robot(args: argparse.Namespace) -> Eggbot:
    camera_config = OpenCVCameraConfig(
        index_or_path=parse_camera_source(args.camera),
        fps=DEFAULT_FPS,
        width=DEFAULT_WIDTH,
        height=DEFAULT_HEIGHT,
        color_mode=ColorMode.BGR,
    )
    robot_config = EggbotConfig(
        id=DEFAULT_ROBOT_ID,
        port=args.port,
        cameras={DEFAULT_CAMERA_NAME: camera_config},
        max_relative_target=DEFAULT_MAX_RELATIVE_TARGET,
        use_degrees=DEFAULT_USE_DEGREES,
    )
    return Eggbot(robot_config)


def get_arm_hold_action(obs: dict[str, Any]) -> dict[str, float]:
    return {
        key: float(value)
        for key, value in obs.items()
        if key.startswith("arm_") and key.endswith(".pos")
    }


def select_target_detection(
    result: Any, targets: set[str] | None, frame_shape: tuple[int, int, int]
) -> Detection | None:
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return None

    frame_h, frame_w = frame_shape[:2]
    frame_area = float(frame_h * frame_w)
    detections: list[Detection] = []

    for box in boxes:
        cls_id = int(box.cls[0])
        label = str(result.names[cls_id])
        if targets is not None and label.lower() not in targets:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        x1 = max(0, min(frame_w - 1, x1))
        x2 = max(0, min(frame_w - 1, x2))
        y1 = max(0, min(frame_h - 1, y1))
        y2 = max(0, min(frame_h - 1, y2))
        box_w = max(0, x2 - x1)
        box_h = max(0, y2 - y1)
        if box_w == 0 or box_h == 0:
            continue

        confidence = float(box.conf[0]) if getattr(box, "conf", None) is not None else 0.0
        detections.append(
            Detection(
                label=label,
                confidence=confidence,
                box_xyxy=(x1, y1, x2, y2),
                center_xy=((x1 + x2) // 2, (y1 + y2) // 2),
                area_ratio=(box_w * box_h) / frame_area,
                height_ratio=box_h / float(frame_h),
            )
        )

    if not detections:
        return None

    return max(detections, key=lambda det: (det.confidence, det.area_ratio))


def compute_base_action(
    detection: Detection,
    frame_shape: tuple[int, int, int],
) -> tuple[dict[str, float], str, float]:
    frame_h, frame_w = frame_shape[:2]
    del frame_h

    dx_norm = (detection.center_xy[0] - frame_w / 2.0) / (frame_w / 2.0)
    if abs(dx_norm) < DEFAULT_CENTER_DEADBAND:
        theta_vel = 0.0
    else:
        theta_vel = clamp(-DEFAULT_TURN_KP * dx_norm, -DEFAULT_MAX_ANGULAR, DEFAULT_MAX_ANGULAR)

    distance_error = DEFAULT_STOP_HEIGHT_RATIO - detection.height_ratio
    if distance_error <= 0.0 and abs(dx_norm) <= DEFAULT_CENTER_DEADBAND:
        return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}, "arrived", dx_norm

    if distance_error <= 0.0:
        x_vel = 0.0
        state = "centering"
    elif abs(dx_norm) > DEFAULT_FORWARD_ALIGNMENT:
        x_vel = 0.0
        state = "turning"
    else:
        distance_scale = clamp(distance_error / DEFAULT_STOP_HEIGHT_RATIO, 0.0, 1.0)
        alignment_scale = clamp(1.0 - abs(dx_norm) / DEFAULT_FORWARD_ALIGNMENT, 0.0, 1.0)
        x_vel = DEFAULT_MAX_LINEAR * distance_scale * alignment_scale
        if x_vel > 0.0:
            x_vel = max(DEFAULT_MIN_LINEAR, x_vel)
        state = "approaching"

    return {"x.vel": x_vel, "y.vel": 0.0, "theta.vel": theta_vel}, state, dx_norm


def draw_status(
    frame: np.ndarray,
    detection: Detection | None,
    state: str,
    action: dict[str, float],
    dx_norm: float | None,
) -> np.ndarray:
    annotated = frame.copy()

    if detection is not None:
        x1, y1, x2, y2 = detection.box_xyxy
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(annotated, detection.center_xy, 4, (0, 255, 0), -1)
        cv2.putText(
            annotated,
            f"{detection.label} {detection.confidence:.2f} h={detection.height_ratio:.2f}",
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    x_vel = action.get("x.vel", 0.0)
    theta_vel = action.get("theta.vel", 0.0)
    dx_text = "none" if dx_norm is None else f"{dx_norm:+.2f}"
    cv2.putText(
        annotated,
        f"{state}  x={x_vel:+.2f}m/s  theta={theta_vel:+.1f}deg/s  dx={dx_text}",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    return annotated


def stop_base(robot: Eggbot, arm_hold_action: dict[str, float] | None = None) -> None:
    action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
    if arm_hold_action:
        action.update(arm_hold_action)
    robot.send_action(action)


def follow_loop(robot: Eggbot, model: Any, args: argparse.Namespace) -> None:
    targets = parse_targets(args.targets)
    control_period = 1.0 / DEFAULT_CONTROL_HZ
    last_seen_at = 0.0

    obs = robot.get_observation()
    arm_hold_action = get_arm_hold_action(obs)
    if arm_hold_action and not args.dry_run:
        robot.send_action({**arm_hold_action, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})

    print(f"Following targets: {sorted(targets) if targets else 'any detected class'}")
    print("Press q or ESC in the video window, or Ctrl+C in the terminal, to stop.")

    while True:
        loop_start = time.perf_counter()
        obs = robot.get_observation()
        frame = obs.get(DEFAULT_CAMERA_NAME)
        if frame is None:
            raise RuntimeError(f"Camera frame '{DEFAULT_CAMERA_NAME}' was not found in robot observation.")

        frame = np.ascontiguousarray(frame)
        results = model.predict(frame, imgsz=DEFAULT_IMGSZ, conf=DEFAULT_CONF, verbose=False)
        result = results[0]
        detection = select_target_detection(result, targets, frame.shape)

        if detection is None:
            seen_recently = time.monotonic() - last_seen_at < DEFAULT_LOST_TIMEOUT
            if seen_recently:
                base_action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
                state = "lost"
            else:
                base_action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": DEFAULT_SEARCH_ANGULAR}
                state = "searching" if DEFAULT_SEARCH_ANGULAR else "stopped"
            dx_norm = None
        else:
            last_seen_at = time.monotonic()
            base_action, state, dx_norm = compute_base_action(detection, frame.shape)

        action = dict(base_action)
        if arm_hold_action:
            action.update(arm_hold_action)
        if args.dry_run:
            print(f"[DRY RUN] {state}: {base_action}")
        else:
            robot.send_action(action)

        if args.display:
            annotated = draw_status(frame, detection, state, base_action, dx_norm)
            cv2.imshow("Eggbot YOLO Base Follow", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

        elapsed = time.perf_counter() - loop_start
        if elapsed < control_period:
            time.sleep(control_period - elapsed)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move an Eggbot base toward a YOLO target from a base camera."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help="Serial port for the SO101 arm + base Feetech bus.",
    )
    parser.add_argument(
        "--camera",
        default=DEFAULT_CAMERA,
        help="OpenCV camera index or path, for example 0 or /dev/video0.",
    )
    parser.add_argument("--model", default=default_model_path())
    parser.add_argument("--targets", default=DEFAULT_TARGETS, help="Comma-separated class names, or 'any'.")
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run detection and control math without sending robot commands.",
    )
    parser.set_defaults(display=True)
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    args = parse_args()

    robot = build_robot(args)

    model = YOLO(args.model)

    try:
        robot.connect()
        follow_loop(robot, model, args)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        try:
            if robot.is_connected and not args.dry_run:
                obs = robot.get_observation()
                stop_base(robot, get_arm_hold_action(obs))
        finally:
            if robot.is_connected:
                robot.disconnect()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
