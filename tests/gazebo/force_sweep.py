"""Gazebo bench check: grip each object at several forces and record the finger opening over
time and the finger joint efforts, to see the force-width curve the objects really have.

    SWEEP="10 20 30" scripts/sim_demo.sh <world_dir>       (launches the bench, then runs this)
"""

import argparse
import json
import re
import subprocess
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

from slatphys_real import common
from slatphys_real.arm import start
from slatphys_real.probe import Prober

ap = argparse.ArgumentParser()
ap.add_argument("--grasps", required=True)
ap.add_argument("--forces", type=float, nargs="+", default=[10, 20, 30, 40])
ap.add_argument("--world", default="slatphys", help="Gazebo world name (make_world.py writes 'slatphys')")
ap.add_argument("--seconds", type=float, default=4.0, help="hold each force this long (wall clock) after it settles")
args = ap.parse_args()

cfg = common.load_config()
arm = start(cfg)
g = common.make_gripper(cfg)
pr = Prober(cfg, arm, g)
def spring_q(name):
    """The object's spring joint position in Gazebo (compression = -q), from its JointStatePublisher."""
    try:
        out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", f"/world/{args.world}/model/{name}/joint_state"],
                             capture_output=True, text=True, timeout=5).stdout
        m = re.search(r"axis1 \{.*?position: ([-0-9.e]+)", out, re.S)
        return float(m.group(1)) if m else None
    except subprocess.TimeoutExpired:
        return None


eff = {}
g.create_subscription(JointState, "/joint_states",
                      lambda m: eff.update(zip(m.name, m.effort)) if m.effort else None, 50)
spec = json.load(open(args.grasps))
pr.go_home()
for k, o in enumerate(spec["objects"]):
    above, at, yaw = pr.geometry(o)
    g.open()
    arm.move_tcp(above, yaw, 3.0, linear=False)
    arm.move_tcp(at, yaw, 2.5)
    print(f"object {k + 1} ({o['name']}), width {o['grasp']['width'] * 1000:.1f} mm")
    for f in args.forces:
        g._command(-f)
        g._settle()
        t0, trace = time.time(), []
        while time.time() - t0 < args.seconds:
            trace.append((time.time() - t0, g.width(1)[0]))
            time.sleep(0.05)
        w = np.array([x for _, x in trace])
        e = [eff.get(j) for j in g.joints]
        q = spring_q(o["name"])
        print(f"  {f:4.0f} N: width t=0.5s {np.interp(0.5, *zip(*trace)) * 1000:6.2f}  t=1s {np.interp(1.0, *zip(*trace)) * 1000:6.2f}"
              f"  t=2s {np.interp(2.0, *zip(*trace)) * 1000:6.2f}  end {w[-1] * 1000:6.2f} mm"
              f"  (last 1 s range {np.ptp(w[-20:]) * 1000:.2f} mm)  spring compression "
              f"{'n/a' if q is None else f'{-q * 1000:.2f} mm'} (fingers: {(o['grasp']['width'] - w[-1]) * 1000:.2f} mm)", flush=True)
    g.open()
    arm.move_tcp(above, yaw, 2.0)
pr.go_home()
g.shutdown()
arm.close()
