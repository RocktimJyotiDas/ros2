"""Save one scene photo (and the camera intrinsics) from a ROS image topic.

The arm is first sent to home_q, so it is in the same pose in every photo (it may be in view, but
must not hide the objects). Several frames
are averaged to cut sensor noise (the masks come from differencing against an empty-table shot).

    ros2 run slatphys_real capture --out data/real/captures/_empty      # once, empty table
    ros2 run slatphys_real capture --out data/real/captures/heavy_01    # per scene
"""

import argparse
import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from slatphys_real import common

ENCODINGS = {"rgb8": (3, False), "bgr8": (3, True), "rgba8": (4, False), "bgra8": (4, True)}


def image_to_rgb(msg):
    if msg.encoding not in ENCODINGS:
        raise ValueError(f"unsupported image encoding {msg.encoding}; use an rgb8/bgr8 stream")
    ch, bgr = ENCODINGS[msg.encoding]
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, : msg.width * ch]
    a = a.reshape(msg.height, msg.width, ch)[..., :3]
    return a[..., ::-1].copy() if bgr else a.copy()


class Grabber(Node):
    def __init__(self, image_topic, info_topic, n):
        super().__init__("slatphys_real_capture")
        self.frames, self.info, self.n = [], None, n
        self.done = threading.Event()
        self.create_subscription(Image, image_topic, self.on_image, 5)
        self.create_subscription(CameraInfo, info_topic, self.on_info, 5)

    def on_info(self, msg):
        self.info = {"K": np.array(msg.k).reshape(3, 3).tolist(), "D": list(msg.d), "distortion_model": msg.distortion_model,
                     "resolution": [msg.width, msg.height], "frame_id": msg.header.frame_id}

    def on_image(self, msg):
        if len(self.frames) < self.n:
            self.frames.append(image_to_rgb(msg).astype(np.float32))
        if len(self.frames) >= self.n and self.info is not None:
            self.done.set()


def grab(cfg, n=10, timeout=15.0):
    if not rclpy.ok():
        rclpy.init()
    node = Grabber(cfg["topics"]["image"], cfg["topics"]["camera_info"], n)
    t0 = time.time()
    while not node.done.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > timeout:
            raise TimeoutError(f"no frames on {cfg['topics']['image']} / {cfg['topics']['camera_info']}")
    rgb = np.clip(np.mean(node.frames, axis=0) + 0.5, 0, 255).astype(np.uint8)
    info = node.info
    node.destroy_node()
    return rgb, info


def main(argv=None):
    from PIL import Image as PILImage

    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--no_home", action="store_true", help="don't move the arm to home_q first")
    ap.add_argument("--execute", metavar="MOVE_REAL_UR3E", help="arming phrase required to move the arm home")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])
    cfg = common.load_config(args.config)
    if not args.no_home:
        if args.execute != "MOVE_REAL_UR3E":
            raise SystemExit("moving home requires --execute MOVE_REAL_UR3E (or --no_home if the arm is out of view)")
        from slatphys_real.arm import start

        arm = start(cfg)
        arm.move_joints(np.array(cfg["home_q"]), cfg["protocol"]["durations"]["home"])
        time.sleep(1.0)
        arm.release()
    rgb, info = grab(cfg, args.frames)
    os.makedirs(args.out, exist_ok=True)
    PILImage.fromarray(rgb).save(os.path.join(args.out, "rgb.png"))
    json.dump(info, open(os.path.join(args.out, "camera_info.json"), "w"), indent=2)
    print(f"wrote {args.out}/rgb.png ({rgb.shape[1]}x{rgb.shape[0]}) and camera_info.json")


if __name__ == "__main__":
    main()
