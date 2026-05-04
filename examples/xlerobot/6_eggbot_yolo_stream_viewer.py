#!/usr/bin/env python3
"""
Receive and display annotated frames from 6_eggbot_yolo_base_follow.py.

Example:
    PYTHONPATH=src python examples/xlerobot/6_eggbot_yolo_stream_viewer.py --port 5001
"""

from __future__ import annotations

import argparse
import socket
import struct

import cv2
import numpy as np

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5001
HEADER_SIZE = 4


def recv_exact(conn: socket.socket, size: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = conn.recv(size - len(chunks))
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


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
            print(f"Stream connected from {addr[0]}:{addr[1]}")
            with conn:
                while True:
                    header = recv_exact(conn, HEADER_SIZE)
                    if header is None:
                        print("Stream disconnected.")
                        break

                    frame_size = struct.unpack("!I", header)[0]
                    payload = recv_exact(conn, frame_size)
                    if payload is None:
                        print("Stream disconnected.")
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
                        return
    finally:
        server.close()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


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
