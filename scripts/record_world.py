#!/usr/bin/env python3
"""Copy a Gazebo world and add recording cameras (images on /rec/<name>/image, simulation time).

  overview  the arm and the table, from the side
  closeup   the objects and the hold point
  front     the scene camera's own view (same pose and field of view as the world's scene_cam,
            i.e. what the lab camera sees); needs a world made with make_world.py --camera

    python3 scripts/record_world.py --world <world_dir>/world.sdf --out runs/<name>/world_rec.sdf \
        --frames runs/<name>/frames [--cams overview closeup] [--fps 10] [--size 640 360]

Gazebo renders a camera only while something subscribes to it: the scripts bridge the images to
ROS and scripts/record_frames.py saves them. Rendering needs a display (the scripts start Xvfb)
and uses ogre2; without a GPU it renders in software (slow, but frames are in simulation time).
"""

import argparse
import math
import os
import re

# World frame = the robot's base_link; the objects lie around (-0.16, 0.21) (UR `base` (x, y) = world (-x, -y)).
CAMERAS = {
    # both look across the base -> objects line, so the robot's base is never behind the objects
    "overview": {"eye": (-0.78, -0.28, 0.52), "target": (-0.08, 0.12, 0.14), "fov": 1.0},
    "closeup": {"eye": (-0.42, 0.00, 0.17), "target": (-0.16, 0.21, 0.08), "fov": 0.80},
}
SENSORS = ('    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
           "<render_engine>ogre2</render_engine></plugin>\n")


def look_at(eye, target):
    """SDF pose rpy for a camera (looking along its +x) at `eye` pointing at `target`."""
    d = [t - e for t, e in zip(target, eye)]
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))  # positive pitch looks down
    return (*eye, 0.0, pitch, yaw)


def scene_cam(world):
    """Pose (6) and horizontal FOV of the world's scene camera (make_world.py --camera)."""
    m = re.search(r'<model name="scene_cam">.*?<pose>([^<]+)</pose>.*?<horizontal_fov>([^<]+)</horizontal_fov>', world, re.S)
    if not m:
        raise SystemExit("this world has no scene_cam (make it with make_world.py --camera front|top)")
    return tuple(float(v) for v in m.group(1).split()), float(m.group(2))


def camera_model(name, pose, fov, fps, w, h):
    return f"""
    <model name="rec_{name}">
      <static>true</static>
      <pose>{" ".join(f"{v:.5f}" for v in pose)}</pose>
      <link name="link">
        <sensor name="{name}" type="camera">
          <always_on>true</always_on>
          <update_rate>{fps}</update_rate>
          <topic>/rec/{name}/image</topic>
          <camera>
            <horizontal_fov>{fov}</horizontal_fov>
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
    ap.add_argument("--cams", nargs="+", default=["overview", "closeup"], choices=["overview", "closeup", "front"])
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--size", type=int, nargs=2, default=[640, 480])
    args = ap.parse_args()
    frames = os.path.abspath(args.frames)
    s = open(args.world).read()
    if "gz-sim-sensors-system" not in s:
        s = s.replace('    <plugin filename="gz-sim-physics-system"', SENSORS + '    <plugin filename="gz-sim-physics-system"', 1)
    cams = ""
    for n in args.cams:
        os.makedirs(os.path.join(frames, n), exist_ok=True)
        if n == "front":
            pose, fov = scene_cam(s)
        else:
            pose, fov = look_at(CAMERAS[n]["eye"], CAMERAS[n]["target"]), CAMERAS[n]["fov"]
        cams += camera_model(n, pose, fov, args.fps, *args.size)
    s = s.replace("  </world>", cams + "\n  </world>")
    open(args.out, "w").write(s)
    print(f"recording world -> {args.out}, frames -> {frames}/{{{','.join(args.cams)}}}")


if __name__ == "__main__":
    main()
