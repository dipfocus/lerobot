#!/usr/bin/env python3
"""
Receive and display annotated frames from 6_eggbot_yolo_base_follow.py.

Example:
    python examples/xlerobot/6_eggbot_yolo_stream_viewer.py --port 5001
"""

from __future__ import annotations

import argparse
import queue
import socket
import struct
import threading
from contextlib import suppress

import cv2
import numpy as np

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5001
HEADER_SIZE = 4


def recv_exact(conn: socket.socket, size: int, stop_event: threading.Event) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        if stop_event.is_set():
            return None
        try:
            chunk = conn.recv(size - len(chunks))
        except TimeoutError:
            continue
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


def put_latest(frames: queue.Queue[bytes], payload: bytes) -> None:
    if frames.full():
        with suppress(queue.Empty):
            frames.get_nowait()

    with suppress(queue.Full):
        frames.put_nowait(payload)


def read_frames(conn: socket.socket, frames: queue.Queue[bytes], stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        header = recv_exact(conn, HEADER_SIZE, stop_event)
        if header is None:
            break

        frame_size = struct.unpack("!I", header)[0]
        payload = recv_exact(conn, frame_size, stop_event)
        if payload is None:
            break

        put_latest(frames, payload)


def receive_frames(host: str, port: int) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)

    print(f"Waiting for stream on {host}:{port}")
    print("Press q or ESC in the video window to stop.")

    try:
        while True:
            conn, addr = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
            conn.settimeout(0.5)
            print(f"Stream connected from {addr[0]}:{addr[1]}")
            with conn:
                frames: queue.Queue[bytes] = queue.Queue(maxsize=1)
                stop_event = threading.Event()
                reader = threading.Thread(
                    target=read_frames,
                    args=(conn, frames, stop_event),
                    name="eggbot-yolo-viewer-reader",
                    daemon=True,
                )
                reader.start()

                while reader.is_alive() or not frames.empty():
                    try:
                        payload = frames.get(timeout=0.05)
                    except queue.Empty:
                        try:
                            key = cv2.waitKey(1) & 0xFF
                        except cv2.error as exc:
                            raise RuntimeError(
                                "The stream receiver needs OpenCV GUI support to show the video window."
                            ) from exc
                        if key in (ord("q"), 27):
                            stop_event.set()
                            return
                        continue

                    while True:
                        try:
                            payload = frames.get_nowait()
                        except queue.Empty:
                            break

                    encoded = np.frombuffer(payload, dtype=np.uint8)
                    frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                    if frame is None:
                        continue

                    try:
                        cv2.imshow("Eggbot YOLO Base Follow", frame)
                        key = cv2.waitKey(1) & 0xFF
                    except cv2.error as exc:
                        raise RuntimeError(
                            "The stream receiver needs OpenCV GUI support to show the video window."
                        ) from exc
                    if key in (ord("q"), 27):
                        stop_event.set()
                        return

                stop_event.set()
                reader.join(timeout=1.0)
                print("Stream disconnected.")
    finally:
        server.close()
        with suppress(cv2.error):
            cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive Eggbot YOLO annotated video stream.")
    parser.add_argument("--host", default=DEFAULT_HOST, help="Local interface to bind.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="TCP port to listen on.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    receive_frames(args.host, args.port)


if __name__ == "__main__":
    main()
