#!/usr/bin/env python3
"""Save the recording cameras' images (bridged from Gazebo to ROS) as JPEGs, one folder per
camera, and log each frame's simulation and wall-clock time (frames.csv) so make_video.py can
put the run_tasks steps under the right frames.

    python3 scripts/record_frames.py --out runs/<world>/frames --cams overview closeup
"""

import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, out, cams):
        super().__init__("slatphys_recorder")
        self.out = out
        self.csv = open(os.path.join(out, "frames.csv"), "a")
        for c in cams:
            os.makedirs(os.path.join(out, c), exist_ok=True)
            self.create_subscription(Image, f"/rec/{c}/image", lambda m, c=c: self.save(c, m), 5)

    def save(self, cam, m):
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[..., :3]
        if m.encoding == "rgb8":
            a = a[..., ::-1]
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        name = f"{int(round(t * 1000)):09d}.jpg"
        cv2.imwrite(os.path.join(self.out, cam, name), a, [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.csv.write(f"{cam},{t:.3f},{time.time():.3f},{name}\n")
        self.csv.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--cams", nargs="+", default=["overview", "closeup"])
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    rclpy.spin(Recorder(args.out, args.cams))


if __name__ == "__main__":
    main()
