"""Sample the real task scenes from the object inventory, with the simulation's task rules.

  heavy       2 objects, masses differ by >= --min_ratio, none lighter than --min_mass_g
  deformable  1 soft + 1 rigid object
  sort        3 objects, consecutive masses differ by >= --min_ratio

Positions are sampled on the simulation's reach band (0.24-0.38 m in front of the robot, +-0.15 m
to the side) and written in the real robot's `base` frame, rotated toward
configs/real.yaml world.workspace_yaw_deg; at least --gap m between object footprints.

    python real2sim/plan_scenes.py --objects configs/objects.yaml --per_task 10 --out data/real/plans
"""

import argparse
import itertools
import json
import os

import numpy as np
import sys

import yaml

# Frame conventions live in the ROS package (slatphys_real/common.py, no ROS imports) so the robot
# and these offline scripts share one definition.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src/slatphys_real"))
from slatphys_real import common  # noqa: E402

# The simulation's reach band, as offsets from the robot in its world frame (objects toward -x).
REACH_X = (-0.38, -0.24)
REACH_Y = (-0.15, 0.15)
TASKS = {
    "heavy": ("Pick up each object one at a time and tell me which one is heavier.", 2),
    "deformable": ("Pick up each object one at a time and tell me which one is deformable.", 2),
    "sort": ("Pick up each object one at a time and sort them from lightest to heaviest.", 3),
}


def valid(task, objs, min_ratio, min_mass_g):
    m = sorted(o["mass_g"] for o in objs)
    if task == "deformable":
        return sum(o["soft"] for o in objs) == 1
    return m[0] >= min_mass_g and all(b / a >= min_ratio for a, b in zip(m, m[1:]))


def place(objs, rng, gap, tries=2000):
    radii = [np.hypot(*o["size_mm"][:2]) / 2000 for o in objs]
    for _ in range(tries):
        xy = np.column_stack([rng.uniform(*REACH_X, len(objs)), rng.uniform(*REACH_Y, len(objs))])
        if all(np.linalg.norm(xy[i] - xy[j]) >= radii[i] + radii[j] + gap
               for i, j in itertools.combinations(range(len(objs)), 2)):
            return xy
    raise RuntimeError(f"could not place {[o['id'] for o in objs]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--objects", default="configs/objects.yaml")
    ap.add_argument("--config", default="configs/real.yaml")
    ap.add_argument("--per_task", type=int, default=10)
    ap.add_argument("--min_ratio", type=float, default=2.0, help="mass ratio for heavy / sort (sim: 1.3)")
    ap.add_argument("--min_mass_g", type=float, default=20.0,
                    help="heavy / sort: lightest object; below ~20 g the wrist F/T can't resolve the load")
    ap.add_argument("--gap", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="data/real/plans")
    args = ap.parse_args()
    inv = yaml.safe_load(open(args.objects))["objects"]
    cfg = yaml.safe_load(open(args.config))
    base_xy = np.asarray(cfg["world"]["robot_base_xy"], float)
    rng = np.random.default_rng(args.seed)
    os.makedirs(args.out, exist_ok=True)

    for task, (instruction, n) in TASKS.items():
        combos = [c for c in itertools.combinations(inv, n) if valid(task, c, args.min_ratio, args.min_mass_g)]
        if not combos:
            print(f"{task}: NO valid combination in the inventory; add objects")
            continue
        # Use every combination once before repeating; shuffle object order (= probing order).
        idx = rng.permutation(len(combos))
        picks = [combos[idx[i % len(combos)]] for i in range(args.per_task)]
        if len(combos) < args.per_task:
            print(f"{task}: only {len(combos)} distinct combinations for {args.per_task} scenes; some repeat"
                  " with new positions and order")
        for s, objs in enumerate(picks):
            objs = [objs[i] for i in rng.permutation(n)]
            xy = place(objs, rng, args.gap)
            sid = f"{task}_{s + 1:02d}"
            plan = {"scene_id": sid, "task": task, "instruction": instruction,
                    "objects": [{**o, "xy_base": np.round(common.world_xy_to_base(cfg, base_xy + d), 3).tolist()}
                                for o, d in zip(objs, xy)]}
            json.dump(plan, open(os.path.join(args.out, f"{sid}.json"), "w"), indent=2)
            print(f"{sid}: " + ", ".join(f"{o['id']} ({o['mass_g']:g} g{', soft' if o['soft'] else ''})" for o in objs))
        print(f"{task}: {len(combos)} valid combinations")


if __name__ == "__main__":
    main()
