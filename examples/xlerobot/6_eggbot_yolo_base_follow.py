#!/usr/bin/env python3
"""
YOLO target approach for an Eggbot with:
- one mobile base,
- one SO101 arm on the base bus,
- one camera mounted on the base.

Example:
    On the viewing machine:
        python examples/xlerobot/6_eggbot_yolo_stream_viewer.py --port 5001

    On the Jetson:
    python examples/xlerobot/6_eggbot_yolo_base_follow.py \
        --port /dev/ttyACM1 \
        --camera 0 \
        --targets bottle \
        --stream-host 192.168.3.252
"""

from __future__ import annotations

import argparse
import logging
import queue
import socket
import struct
import threading
import time
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from ultralytics import YOLO

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
DEFAULT_DEVICE = "cuda:0"
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
DEFAULT_STREAM_HOST = "192.168.3.252"
DEFAULT_STREAM_PORT = 5001
DEFAULT_STREAM_QUALITY = 80
DEFAULT_STREAM_EVERY_N = 1
DEFAULT_STREAM_SCALE = 1.0
DEFAULT_STREAM_RECONNECT_S = 2.0
DEFAULT_STREAM_TIMEOUT_S = 0.2
DEFAULT_STARTUP_BASE_LINEAR = 0.04
DEFAULT_STARTUP_BASE_MOVE_S = 0.25
DEFAULT_STARTUP_ARM_DELTA = 3.0
DEFAULT_STARTUP_ARM_MOVE_S = 0.35
STARTUP_ARM_TEST_JOINT = "arm_wrist_roll.pos"


@dataclass
class Detection:
    label: str
    confidence: float
    box_xyxy: tuple[int, int, int, int]
    center_xy: tuple[int, int]
    area_ratio: float
    height_ratio: float


class RemoteFrameStreamer:
    def __init__(
        self,
        host: str,
        port: int,
        jpeg_quality: int,
        scale: float = DEFAULT_STREAM_SCALE,
        reconnect_interval_s: float = DEFAULT_STREAM_RECONNECT_S,
        timeout_s: float = DEFAULT_STREAM_TIMEOUT_S,
    ):
        self.host = host
        self.port = port
        self.jpeg_quality = jpeg_quality
        self.scale = scale
        self.reconnect_interval_s = reconnect_interval_s
        self.timeout_s = timeout_s
        self._socket: socket.socket | None = None
        self._last_connect_attempt = 0.0
        self._warned_disconnected = False
        self._frames: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="eggbot-yolo-streamer", daemon=True)
        self._thread.start()

    def _connect(self) -> bool:
        if self._socket is not None:
            return True

        now = time.monotonic()
        if now - self._last_connect_attempt < self.reconnect_interval_s:
            return False
        self._last_connect_attempt = now

        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        except OSError as exc:
            if not self._warned_disconnected:
                print(f"Waiting for stream receiver at {self.host}:{self.port}: {exc}")
                self._warned_disconnected = True
            return False

        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256 * 1024)
        sock.settimeout(self.timeout_s)
        self._socket = sock
        self._warned_disconnected = False
        print(f"Streaming annotated video to {self.host}:{self.port}")
        return True

    def send(self, frame: np.ndarray) -> None:
        if self._stop_event.is_set():
            return

        if self._frames.full():
            with suppress(queue.Empty):
                self._frames.get_nowait()

        with suppress(queue.Full):
            self._frames.put_nowait(frame)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self._frames.get(timeout=0.1)
            except queue.Empty:
                continue

            while True:
                try:
                    frame = self._frames.get_nowait()
                except queue.Empty:
                    break

            self._send_frame(frame)

    def _send_frame(self, frame: np.ndarray) -> None:
        if not self._connect():
            return

        if self.scale != 1.0:
            frame = cv2.resize(
                frame,
                None,
                fx=self.scale,
                fy=self.scale,
                interpolation=cv2.INTER_AREA,
            )

        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            logger.warning("Failed to encode stream frame as JPEG.")
            return

        payload = encoded.tobytes()
        packet = struct.pack("!I", len(payload)) + payload

        try:
            assert self._socket is not None
            self._socket.sendall(packet)
        except OSError as exc:
            print(f"Stream receiver disconnected: {exc}")
            self._close_socket()

    def _close_socket(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None

    def close(self) -> None:
        self._stop_event.set()
        self._close_socket()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)


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


def choose_startup_arm_target(
    robot: Eggbot, arm_hold_action: dict[str, float]
) -> tuple[str, float] | None:
    if not arm_hold_action:
        return None

    joint = STARTUP_ARM_TEST_JOINT
    if joint not in arm_hold_action:
        joint = next(
            (key for key in arm_hold_action if key != "arm_gripper.pos"),
            next(iter(arm_hold_action)),
        )

    current = float(arm_hold_action[joint])
    if robot.config.use_degrees:
        step = -DEFAULT_STARTUP_ARM_DELTA if current > 0.0 else DEFAULT_STARTUP_ARM_DELTA
        return joint, current + step

    lower, upper = (0.0, 100.0) if joint == "arm_gripper.pos" else (-100.0, 100.0)
    step = DEFAULT_STARTUP_ARM_DELTA
    if current > upper - DEFAULT_STARTUP_ARM_DELTA:
        step = -DEFAULT_STARTUP_ARM_DELTA
    target = clamp(current + step, lower, upper)
    if target == current and current > lower + DEFAULT_STARTUP_ARM_DELTA:
        target = current - DEFAULT_STARTUP_ARM_DELTA
    return joint, target


def run_startup_motion_check(robot: Eggbot, dry_run: bool = False) -> None:
    obs = robot.get_observation()
    arm_hold_action = get_arm_hold_action(obs)
    base_stop_action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

    print("Running startup hardware check: base forward/back, arm wrist twitch.")
    if dry_run:
        arm_target = choose_startup_arm_target(robot, arm_hold_action)
        print(
            "[DRY RUN] Startup check would send "
            f"base +/-{DEFAULT_STARTUP_BASE_LINEAR:.2f} m/s and arm target {arm_target}."
        )
        return

    try:
        robot.send_action({**arm_hold_action, **base_stop_action})
        time.sleep(0.1)

        for x_vel in (DEFAULT_STARTUP_BASE_LINEAR, -DEFAULT_STARTUP_BASE_LINEAR):
            robot.send_action({**arm_hold_action, "x.vel": x_vel, "y.vel": 0.0, "theta.vel": 0.0})
            time.sleep(DEFAULT_STARTUP_BASE_MOVE_S)
        stop_base(robot, arm_hold_action)
        time.sleep(0.1)

        arm_target = choose_startup_arm_target(robot, arm_hold_action)
        if arm_target is None:
            print("Startup hardware check skipped arm motion: no arm position in observation.")
        else:
            joint, target = arm_target
            robot.send_action({**arm_hold_action, **base_stop_action, joint: target})
            time.sleep(DEFAULT_STARTUP_ARM_MOVE_S)
            robot.send_action({**arm_hold_action, **base_stop_action})
            time.sleep(DEFAULT_STARTUP_ARM_MOVE_S)
    finally:
        stop_base(robot, arm_hold_action)

    print("Startup hardware check complete.")


def follow_loop(robot: Eggbot, model: Any, args: argparse.Namespace) -> None:
    targets = parse_targets(args.targets)
    control_period = 1.0 / DEFAULT_CONTROL_HZ
    last_seen_at = 0.0
    frame_index = 0
    streamer = (
        RemoteFrameStreamer(args.stream_host, args.stream_port, args.stream_quality)
        if args.stream_host
        else None
    )

    obs = robot.get_observation()
    arm_hold_action = get_arm_hold_action(obs)
    if arm_hold_action and not args.dry_run:
        robot.send_action({**arm_hold_action, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})

    print(f"Following targets: {sorted(targets) if targets else 'any detected class'}")
    if streamer is None:
        print("No stream receiver configured. Pass --stream-host <receiver-ip> to send annotated video.")
    print("Press Ctrl+C in the terminal to stop.")

    try:
        while True:
            loop_start = time.perf_counter()
            obs = robot.get_observation()
            frame = obs.get(DEFAULT_CAMERA_NAME)
            if frame is None:
                raise RuntimeError(f"Camera frame '{DEFAULT_CAMERA_NAME}' was not found in robot observation.")

            frame = np.ascontiguousarray(frame)
            results = model.predict(
                frame,
                imgsz=DEFAULT_IMGSZ,
                conf=DEFAULT_CONF,
                device=DEFAULT_DEVICE,
                verbose=False,
            )
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

            if streamer is not None and frame_index % args.stream_every_n == 0:
                annotated = draw_status(frame, detection, state, base_action, dx_norm)
                streamer.send(annotated)
            frame_index += 1

            elapsed = time.perf_counter() - loop_start
            if elapsed < control_period:
                time.sleep(control_period - elapsed)
    finally:
        if streamer is not None:
            streamer.close()


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
    parser.add_argument(
        "--stream-host",
        default=DEFAULT_STREAM_HOST,
        help="IP address or hostname of the machine running 6_eggbot_yolo_stream_viewer.py.",
    )
    parser.add_argument(
        "--stream-port",
        type=int,
        default=DEFAULT_STREAM_PORT,
        help="TCP port used by the stream receiver.",
    )
    parser.add_argument(
        "--stream-quality",
        type=int,
        default=DEFAULT_STREAM_QUALITY,
        help="JPEG quality for the annotated video stream, from 1 to 100.",
    )
    parser.add_argument(
        "--stream-every-n",
        type=int,
        default=DEFAULT_STREAM_EVERY_N,
        help="Send one annotated frame every N control frames.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run detection and control math without sending robot commands.",
    )
    parser.add_argument(
        "--skip-startup-check",
        action="store_true",
        help="Skip the short base and arm motion check after connecting.",
    )
    args = parser.parse_args()
    args.stream_quality = int(clamp(args.stream_quality, 1, 100))
    args.stream_every_n = max(1, args.stream_every_n)
    return args


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    args = parse_args()

    robot = build_robot(args)

    try:
        robot.connect()
        if not args.skip_startup_check:
            run_startup_motion_check(robot, dry_run=args.dry_run)
        model = YOLO(args.model)
        model.to(DEFAULT_DEVICE)
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


if __name__ == "__main__":
    main()
