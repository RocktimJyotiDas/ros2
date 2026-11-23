#!/usr/bin/env python3
"""Copy a Gazebo world and add two recording cameras (images on /rec/<name>/image, simulation time):
an overview of the arm and the table, and a close-up of the objects and the hold point.

    python3 scripts/record_world.py --world <world_dir>/world.sdf --out runs/<name>/world_rec.sdf \
        --frames runs/<name>/frames [--fps 10]

Gazebo renders a camera only while something subscribes to it: scripts/sim_demo.sh bridges the
images to ROS and scripts/record_frames.py saves them. Rendering needs a display (scripts/sim_demo.sh starts Xvfb for RECORD=1) and uses ogre2; on a
machine without a GPU it renders in software (slow, but frames are taken in simulation time).
"""

import argparse
import math
import os

# World frame = the robot's base_link; the objects lie around (-0.16, 0.21) (UR `base` (x, y) = world (-x, -y)).
CAMERAS = {
    # both look across the base -> objects line, so the robot's base is never behind the objects
    "overview": {"eye": (-0.78, -0.28, 0.52), "target": (-0.08, 0.12, 0.14), "fov": 1.0},
    "closeup": {"eye": (-0.42, 0.00, 0.17), "target": (-0.16, 0.21, 0.08), "fov": 0.80},
}


def look_at(eye, target):
    """SDF pose rpy for a camera (looking along its +x) at `eye` pointing at `target`."""
    d = [t - e for t, e in zip(target, eye)]
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))  # positive pitch looks down
    return (*eye, 0.0, pitch, yaw)


def camera_model(name, cam, frames, fps, w, h):
    pose = " ".join(f"{v:.4f}" for v in look_at(cam["eye"], cam["target"]))
    return f"""
    <model name="rec_{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <sensor name="{name}" type="camera">
          <always_on>true</always_on>
          <update_rate>{fps}</update_rate>
          <topic>/rec/{name}/image</topic>
          <camera>
            <horizontal_fov>{cam["fov"]}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>0.02</near><far>10</far></clip>
          </camera>
        </sensor>
      </link>
    </model>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", required=True)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--size", type=int, nargs=2, default=[640, 480])
    args = ap.parse_args()
    frames = os.path.abspath(args.frames)
    for n in CAMERAS:
        os.makedirs(os.path.join(frames, n), exist_ok=True)
    s = open(args.world).read()
    sensors = ('    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
               "<render_engine>ogre2</render_engine></plugin>\n")
    s = s.replace('    <plugin filename="gz-sim-physics-system"', sensors + '    <plugin filename="gz-sim-physics-system"', 1)
    cams = "".join(camera_model(n, c, frames, args.fps, *args.size) for n, c in CAMERAS.items())
    s = s.replace("  </world>", cams + "\n  </world>")
    open(args.out, "w").write(s)
    print(f"recording world -> {args.out}, frames -> {frames}/{{{','.join(CAMERAS)}}}")


if __name__ == "__main__":
    main()
