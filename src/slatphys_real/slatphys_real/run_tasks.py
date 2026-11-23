"""Both validation tasks (heavy, deformable) on one real scene with one probing run.

One squeeze pass + one weigh pass over the scene's objects (slatphys_real.probe, unchanged)
yields both readings each task needs, so the two tasks share the run:
  heavy       <- weigh pass   (wrist load while holding each object)
  deformable  <- squeeze pass (finger width at 20 N and 60 N on each object)

Before anything moves, every grasp is checked against the hardware: width inside the Hand-E's
stroke, an IK solution above and at the grasp, the safety floor and reach. Then the probe runs,
and the answers (and, with --sim, the SLAT-Phys twin's answers) are printed and saved.

    ros2 run slatphys_real run_tasks --grasps data/real/sim/slatphys/img4827/grasps.json \
        --plan data/real/plans/img4827.json --out data/real/robot/img4827 --dry_run
    ros2 run slatphys_real run_tasks ... --execute MOVE_REAL_UR3E --activate_gripper --step
"""

import argparse
import json
import os
import sys

import numpy as np

from slatphys_real import common, tasks
from slatphys_real import ur_kinematics as K

STROKE_M = 0.050
WIDTH_MARGIN_M = 0.004  # fingers must clear the object on the way down


def preflight(cfg, spec, tz):
    """Problems that would make a grasp fail or unsafe, one string each (empty = go)."""
    P, s = cfg["protocol"], cfg["safety"]
    tcp_z, q_home = cfg["gripper"]["tcp_z"], np.array(cfg["home_q"], float)
    floor = tz + s["min_tcp_above_table"]
    problems = []
    if spec.get("preview_from_uncalibrated_photo") and cfg.get("motion_backend", "enme480") == "enme480":
        problems.append("these grasps come from an uncalibrated photo (preview_from_uncalibrated_photo): their "
                        "positions are not robot coordinates. Use the grasps.json from a calibrated capture.")
    for k, o in enumerate(spec["objects"]):
        g, name = o["grasp"], f"object {k + 1} ({o['name']})"
        if g["width"] > STROKE_M - WIDTH_MARGIN_M:
            problems.append(f"{name}: grasp width {g['width'] * 1000:.0f} mm > {(STROKE_M - WIDTH_MARGIN_M) * 1000:.0f} mm "
                            f"(Hand-E stroke {STROKE_M * 1000:.0f} mm minus margin): it cannot be grasped top-down")
        z_at = max(tz + cfg["gripper"]["pad_half"] + P["tip_clearance"], g["top_z"] - 0.5 * g["height"])
        for label, z in (("above", g["top_z"] + P["above"]), ("at", z_at)):
            p = np.array([*g["centre_xy"], z])
            if z < floor:
                problems.append(f"{name}: TCP {label} the grasp at z {z:.3f} is below the safety floor {floor:.3f}")
            if np.hypot(*p[:2]) > s["max_reach"]:
                problems.append(f"{name}: {np.round(p, 3).tolist()} is beyond max reach {s['max_reach']} m")
            if all(K.solve_tcp(p, y, tcp_z, q_home) is None for y in (g["yaw"], g["yaw"] + np.pi)):
                problems.append(f"{name}: no IK {label} the grasp at {np.round(p, 3).tolist()}")
    return problems


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--grasps", required=True, help="grasps.json (real2sim/materialize_real.py)")
    ap.add_argument("--plan", help="plan json with the measured objects (mass_g, soft): scores the answers")
    ap.add_argument("--sim", help="the SLAT-Phys twin's readings.json for the same scene: compares answers")
    ap.add_argument("--out", required=True, help="output directory (readings*.json, answers.json)")
    ap.add_argument("--readings_only", action="store_true", help="skip probing: answer from <out>/readings.json")
    args, probe_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])

    cfg = common.load_config(args.config)
    spec = json.load(open(args.grasps))
    if not args.readings_only:
        problems = preflight(cfg, spec, common.table_z(cfg))
        for p in problems:
            print("PREFLIGHT:", p)
        if problems:
            raise SystemExit(f"{len(problems)} preflight problem(s); nothing moved")
        print(f"preflight ok: {len(spec['objects'])} objects")
        from slatphys_real import probe

        cfg_args = ["--config", args.config] if args.config else []
        probe.main(cfg_args + ["--grasps", args.grasps, "--out", args.out, "--phase", "both"] + probe_args)

    r = json.load(open(os.path.join(args.out, "readings.json")))
    objects = json.load(open(args.plan))["objects"] if args.plan else None
    names = [o["name"] for o in (objects or spec["objects"])]
    res = {"scene_id": spec["scene_id"], "real": tasks.report(r, names, tasks.TASKS, objects, "real")}
    if args.sim:
        res["sim"] = tasks.report(json.load(open(args.sim)), names, tasks.TASKS, objects, "sim")
        for t in tasks.TASKS:
            agree = res["real"]["answers"][t]["answer"] == res["sim"]["answers"][t]["answer"]
            print(f"  {t:10s} sim and real {'AGREE' if agree else 'DISAGREE'}")
    json.dump(common.to_jsonable(res), open(os.path.join(args.out, "answers.json"), "w"), indent=2)
    print("wrote", os.path.join(args.out, "answers.json"))


if __name__ == "__main__":
    main()
