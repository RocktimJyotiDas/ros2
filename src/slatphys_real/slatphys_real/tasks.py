"""The two validation tasks, answered from probing readings (no ROS imports: the robot, the
offline scripts and the tests share it).

  heavy       "which object is heavier?"    -> the object with the largest wrist load
  deformable  "which object is deformable?" -> the object with the largest squeeze compression

The same rule as the paper's simulation (SLAT-Phys-Scene/slatscene/score_readings.py) and
real2sim/correlate.py, on readings.json of either source (same schema):
  load         |F_with - F_without| of the wrist force at the hold pose (N). F_without is the
               mean of the empty-gripper readings before and after the object when both exist.
  compression  finger width at the lowest squeeze force minus at the highest (m)

    ros2 run slatphys_real answer --readings data/real/robot/heavy_img4827/readings.json \
        [--sim data/photo/sim/slatphys/heavy_img4827/readings.json] [--plan data/real/plans/img4827.json]
"""

import argparse
import json
import sys

import numpy as np

TASKS = ("heavy", "deformable")
G = 9.81


def load_N(r):
    w = r["wrist_force_torque"]
    f0 = np.array(w["without_object"][:3], float)
    if w.get("without_object_after") is not None:
        f0 = (f0 + np.array(w["without_object_after"][:3], float)) / 2
    return float(np.linalg.norm(np.array(w["with_object"][:3], float) - f0))


def compression_m(r):
    sq = r["gripper_squeeze"]
    return float(sq[0]["width_m"] - sq[-1]["width_m"])


def measures(readings):
    """Per object: load (N), compression (m), lifted (bool or None)."""
    rs = readings["readings"]
    diag = readings.get("diagnostics", [{}] * len(rs))
    lifted = [d.get("weigh", {}).get("lifted") for d in diag]
    return [load_N(r) for r in rs], [compression_m(r) for r in rs], lifted


def answer(task, load, compression):
    """1-based index of the answer."""
    if task == "heavy":
        return int(np.argmax(load)) + 1
    if task == "deformable":
        return int(np.argmax(compression)) + 1
    raise ValueError(f"unknown task {task}")


def truth(task, objects):
    """1-based ground-truth answer from the plan's / scene's measured objects, or None."""
    vals = [o.get("real", o) for o in objects]
    if task == "heavy" and all(v.get("mass_g") is not None for v in vals):
        return int(np.argmax([v["mass_g"] for v in vals])) + 1
    if task == "deformable" and all(v.get("soft") is not None for v in vals):
        soft = [bool(v["soft"]) for v in vals]
        return soft.index(True) + 1 if sum(soft) == 1 else None
    return None


def report(readings, names, tasks=TASKS, objects=None, label="real"):
    load, comp, lifted = measures(readings)
    out = {"source": label, "objects": names, "load_N": load, "compression_m": comp, "lifted": lifted, "answers": {}}
    for k, n in enumerate(names):
        print(f"  [{label}] {k + 1} {n:16s} load {load[k]:7.3f} N (~{load[k] / G * 1000:5.0f} g)  "
              f"compression {comp[k] * 1000:6.2f} mm  lifted {lifted[k]}")
    for t in tasks:
        a = answer(t, load, comp)
        gt = truth(t, objects) if objects else None
        flag = "" if gt is None else ("  CORRECT" if a == gt else f"  WRONG (truth: {names[gt - 1]})")
        warn = "  (an object was not lifted: load unreliable)" if t == "heavy" and False in lifted else ""
        print(f"  [{label}] {t:10s} -> {names[a - 1]}{flag}{warn}")
        out["answers"][t] = {"answer": a, "name": names[a - 1], "truth": gt}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--readings", help="real robot readings.json")
    ap.add_argument("--sim", help="simulated readings.json of the same scene (SLAT-Phys twin)")
    ap.add_argument("--plan", help="plan / scene json with the measured objects (mass_g, soft)")
    ap.add_argument("--tasks", nargs="+", default=list(TASKS), choices=TASKS)
    ap.add_argument("--out", help="write the answers here (json)")
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])
    if not (args.readings or args.sim):
        raise SystemExit("give --readings and/or --sim")
    objects = json.load(open(args.plan))["objects"] if args.plan else None
    res = {}
    for label, path in (("real", args.readings), ("sim", args.sim)):
        if path:
            r = json.load(open(path))
            names = ([o["name"] for o in objects] if objects else
                     [d.get("weigh", d.get("squeeze", {})).get("name", f"object {k + 1}")
                      for k, d in enumerate(r.get("diagnostics", r["readings"]))])
            res[label] = report(r, names, args.tasks, objects, label)
    if "real" in res and "sim" in res:
        for t in args.tasks:
            agree = res["real"]["answers"][t]["answer"] == res["sim"]["answers"][t]["answer"]
            print(f"  {t:10s} sim and real {'AGREE' if agree else 'DISAGREE'}")
    if args.out:
        json.dump(res, open(args.out, "w"), indent=2)


if __name__ == "__main__":
    main()
